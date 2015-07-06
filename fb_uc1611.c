/*
 * FB driver for the UltraChip UC1611 LCD controller
 *
 * The display is 4-bit grayscale (16 shades) 240x160.
 *
 * Copyright (C) 2015 Henri Chain
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>

#include "fbtft.h"

#define DRVNAME		"fb_uc1611"
#define WIDTH		240
#define HEIGHT		160
#define BPP		8

// #define RGB565_TO_4BIT(RGB) \
// 	(19595 * red + 38470 * green +
//                                 7471 * blue) >> 16

/*
static unsigned 3wire = 0;
module_param(3wire, uint, 0);
MODULE_PARM_DESC(3wire, "Use 3-wire (9-bit) SPI (default: 0)");
*/

/*
 * LCD voltage is a combination of ratio, gain, pot and temp
 *
 * V_LCD = V_BIAS * ratio
 * V_LCD = (C_V0 + C_PM × pot) * (1 + (T - 25) * temp)
 * C_V0 and C_PM depend on ratio and gain
 * T is ambient temperature
 */

/* BR -> actual ratio: 0-3 -> 5, 10, 11, 13 */
static unsigned ratio = 2;
module_param(ratio, uint, 0);
MODULE_PARM_DESC(ratio, "BR[1:0] Bias voltage ratio: 0-3 (default: 2)");

//static unsigned gain = 3;
static unsigned gain = 0;
module_param(gain, uint, 0);
MODULE_PARM_DESC(gain, "GN[1:0] Bias voltage gain: 0-3 (default: 3)");

//static unsigned pot = 16;
static unsigned pot = 28;
module_param(pot, uint, 0);
MODULE_PARM_DESC(pot, "PM[6:0] Bias voltage pot.: 0-63 (default: 16)");

/* TC -> % compensation per deg C: 0-3 -> -.05, -.10, -.015, -.20 */
static unsigned temp = 0;
module_param(temp, uint, 0);
MODULE_PARM_DESC(temp, "TC[1:0] Temperature compensation: 0-3 (default: 0)");

/* PC[1:0] -> LCD capacitance: 0-3 -> <20nF, 20-28 nF, 29-40 nF, 40-56 nF */
static unsigned load = 1;
module_param(load, uint, 0);
MODULE_PARM_DESC(load, "PC[1:0] Panel Loading: 0-3 (default: 1)");

/* PC[3:2] -> V_LCD: 0, 1, 3 -> ext., int. with ratio = 5, int. standard */
static unsigned pump = 3;
module_param(pump, uint, 0);
MODULE_PARM_DESC(pump, "PC[3:2] Pump control: 0,1,3 (default: 3)");

static int init_display(struct fbtft_par *par) {
	int ret;

	fbtft_par_dbg(DEBUG_INIT_DISPLAY, par, "%s()\n", __func__);

	/* Set CS active high */
	par->spi->mode |= SPI_CS_HIGH;
	ret = par->spi->master->setup(par->spi);
	if (ret) {
		dev_err(par->info->device, "Could not set SPI_CS_HIGH\n");
		return ret;
	}

	/* Reset controller */
	write_reg(par, 0xE2);

	/* Set bias ratio */
	write_reg(par, 0xE8 | (ratio & 0x03));

	/* Set bias gain and potentiometer */
	write_reg(par, 0x81);
	write_reg(par, (gain & 0x03) << 6 | (pot & 0x3F));

	/* Set temperature compensation */
	write_reg(par, 0x24 | (temp & 0x03));

	/* Set panel loading */
	write_reg(par, 0x28 | (load & 0x03));

	/* Set pump control */
	write_reg(par, 0x2C | (pump & 0x03));

	// Redundant with set_addr_win()
	// /* Reset page address */
	// write_reg(par, 0x60 | (0x00 & 0x0F));
	// write_reg(par, 0x70 | (0x00 >> 4));

	// /* Reset column address */
	// write_reg(par, 0x00 & 0x0F);
	// write_reg(par, 0x10 | (0x00 >> 4));

	/* Set 4-bit grayscale mode */
	write_reg(par, 0xD0 | (0x02 & 0x03));

	// Redundant with set_var()
	// /* Set LCD mapping */
	// write_reg(par, 0xC0
	//                | (0x0 & 0x1) << 2 // Mirror Y OFF
	//                | (0x1 & 0x1) << 1 // Mirror X ON
	//                | (0x0 & 0x1) );   // MS nibble last (default)

	// /* Set RAM address control */
	// write_reg(par, 0x88
	//                | (0x0 & 0x1) << 2 // Increment positively
	//                | (0x0 & 0x1) << 1 // Increment column first
	//                | (0x1 & 0x1) );   // Wrap around (default)

	/* Set Display enable */
	write_reg(par, 0xA8 | 0x07);

	return 0;
}

static void mkdirty(struct fb_info *info, int y, int height) {
	struct fbtft_par *par = info->par;
	struct fb_deferred_io *fbdefio = info->fbdefio;

	/* UC1611 can only set page address (a page is two lines) */
	if (y % 2) {
		y--;
		height++;
	}

	/* special case, needed ? */
	if (y == -1) {
		y = 0;
		height = info->var.yres - 1;
	}

	/* Mark display lines/area as dirty */
	spin_lock(&par->dirty_lock);
	if (y < par->dirty_lines_start)
		par->dirty_lines_start = y;
	if (y + height - 1 > par->dirty_lines_end)
		par->dirty_lines_end = y + height - 1;
	spin_unlock(&par->dirty_lock);

	/* Schedule deferred_io to update display (no-op if already on queue)*/
	schedule_delayed_work(&info->deferred_work, fbdefio->delay);
}


static void set_addr_win(struct fbtft_par *par, int xs, int ys, int xe, int ye) {
	fbtft_par_dbg(DEBUG_SET_ADDR_WIN, par,
		"%s(xs=%d, ys=%d, xe=%d, ye=%d)\n", __func__, xs, ys, xe, ye);

	/* Set column address */
	write_reg(par, xs & 0x0F);
	write_reg(par, 0x10 | (xs >> 4));

	/* Set page address */
	write_reg(par, 0x60 | ((ys >> 1) & 0x0F));
	write_reg(par, 0x70 | (ys >> 5));

	/* Set max column address for wrap-around? */
}

/*static int blank(struct fbtft_par *par, bool on) {
	fbtft_par_dbg(DEBUG_BLANK, par, "%s(blank=%s)\n",
		__func__, on ? "true" : "false");

	if (on)
		write_reg(par, 0xAE);
	else
		write_reg(par, 0xAF);
	return 0;
}*/

/* Gamma is used to control Contrast */
// static int set_gamma(struct fbtft_par *par, unsigned long *curves) {
// 	fbtft_par_dbg(DEBUG_INIT_DISPLAY, par, "%s()\n", __func__);

// 	/* apply mask */
// 	curves[0] &= 0xFF;

// 	/* Set Contrast Control for BANK0 */
// 	write_reg(par, 0x81);
// 	write_reg(par, curves[0]);

// 	return 0;
// }

static int set_var(struct fbtft_par *par)
{
	fbtft_par_dbg(DEBUG_INIT_DISPLAY, par, "%s()\n", __func__);

	par->info->var.grayscale = 1;

	switch (par->info->var.rotate) {
	case 90:
		/* Set RAM address control */
		write_reg(par, 0x88
		               | (0x0 & 0x1) << 2 // Increment positively
		               | (0x1 & 0x1) << 1 // Increment page first
		               | (0x1 & 0x1) );   // Wrap around (default)

		/* Set LCD mapping */
		write_reg(par, 0xC0
		               | (0x0 & 0x1) << 2 // Mirror Y OFF
		               | (0x0 & 0x1) << 1 // Mirror X OFF
		               | (0x0 & 0x1) );   // MS nibble last (default)
		break;
	case 180:
		/* Set RAM address control */
		write_reg(par, 0x88
		               | (0x0 & 0x1) << 2 // Increment positively
		               | (0x0 & 0x1) << 1 // Increment column first
		               | (0x1 & 0x1) );   // Wrap around (default)

		/* Set LCD mapping */
		write_reg(par, 0xC0
		               | (0x1 & 0x1) << 2 // Mirror Y ON
		               | (0x0 & 0x1) << 1 // Mirror X OFF
		               | (0x0 & 0x1) );   // MS nibble last (default)
		break;
	case 270:
		/* Set RAM address control */
		write_reg(par, 0x88
		               | (0x0 & 0x1) << 2 // Increment positively
		               | (0x1 & 0x1) << 1 // Increment page first
		               | (0x1 & 0x1) );   // Wrap around (default)

		/* Set LCD mapping */
		write_reg(par, 0xC0
		               | (0x1 & 0x1) << 2 // Mirror Y ON
		               | (0x1 & 0x1) << 1 // Mirror X ON
		               | (0x0 & 0x1) );   // MS nibble last (default)
		break;
	default:
		/* Set RAM address control */
		write_reg(par, 0x88
		               | (0x0 & 0x1) << 2 // Increment positively
		               | (0x0 & 0x1) << 1 // Increment column first
		               | (0x1 & 0x1) );   // Wrap around (default)

		/* Set LCD mapping */
		write_reg(par, 0xC0
		               | (0x0 & 0x1) << 2 // Mirror Y OFF
		               | (0x1 & 0x1) << 1 // Mirror X ON
		               | (0x0 & 0x1) );   // MS nibble last (default)
		break;
	}

	return 0;
}

static int write_vmem(struct fbtft_par *par, size_t offset, size_t len) {
	u8 *vmem8 = (8 *)(par->info->screen_base + offset);
	u8 *buf = par->txbuf.buf;
	int x, y, i = 0;
	int ret = 0;
	int xres = par->info->var.xres;
	int yres = par->info->var.yres;

	fbtft_par_dbg(DEBUG_WRITE_VMEM, par, "%s()\n", __func__);

	// switch order of loops in case of rotate? probably not...
	y = 0;
	while (y < yres && i < len) {
		x = 0;
		while (x < xres && i < len) {
			*buf = vmem8[y * xres + x] >> 4;
			*buf |= vmem8[y * xres + xres + x] & 0xF0;
			buf++;
			i += 2;
			x++;
		}
		y += 2;
	}

	/* Write data */
	gpio_set_value(par->gpio.dc, 1);
	ret = par->fbtftops.write(par, par->txbuf.buf, len / 2);

	if (ret < 0)
		dev_err(par->info->device, "write failed and returned: %d\n",
			ret);

	return ret;
}


static struct fbtft_display display = {
	.txbuflen = -1,
	.regwidth = 8,
	.width = WIDTH,
	.height = HEIGHT,
	.bpp = BPP,
	.fbtftops = {
		// .write = fbtft_write_spi,
		.write_vmem = write_vmem,
		.init_display = init_display,
		.set_addr_win = set_addr_win,
		.set_var = set_var,
		//.blank = blank,
		.mkdirty = mkdirty,
	},
};

FBTFT_REGISTER_DRIVER(DRVNAME, "ultrachip,uc1611", &display);

MODULE_ALIAS("spi:" DRVNAME);
MODULE_ALIAS("platform:" DRVNAME);
MODULE_ALIAS("spi:uc1611");
MODULE_ALIAS("platform:uc1611");

MODULE_DESCRIPTION("FB driver for the UC1611 LCD controller");
MODULE_AUTHOR("Henri Chain");
MODULE_LICENSE("GPL");
