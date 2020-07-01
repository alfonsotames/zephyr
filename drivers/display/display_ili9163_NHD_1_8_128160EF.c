/*
 * Copyright (c) 2018 - 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include "display_ili9163.h"

/*
 * Derived from Seeed 2.8 inch TFT Touch Shield v2.0 sample code.
 *
 * https://github.com/Seeed-Studio/TFT_Touch_Shield_V2
 */




void ili9163_lcd_init(struct ili9163_data *p_ili9163) {



    //Software reset -----
    ili9163_transmit(p_ili9163, ILI9163_CMD_SWRESET,NULL, 0);
    k_msleep(122); //500
    ili9163_transmit(p_ili9163, ILI9163_CMD_SLPOUT,NULL, 0);
    k_msleep(5);
    ili9163_transmit(p_ili9163, ILI9163_CMD_IDLEOF,NULL, 0);
    ili9163_transmit(p_ili9163, ILI9163_CMD_DISPOFF,NULL, 0);

    uint8_t pwc1[] = {0x1D, 0x02};
    ili9163_transmit(p_ili9163, ILI9163_CMD_PWCTR1, &pwc1, 2);

    uint8_t pwc2[] = {0x02};
    ili9163_transmit(p_ili9163, ILI9163_CMD_PWCTR2, &pwc2, 1);

    uint8_t pwc3[] = {0x01};
    ili9163_transmit(p_ili9163, ILI9163_CMD_PWCTR3, &pwc3, 1);

    uint8_t pwc4[] = {0x01};
    ili9163_transmit(p_ili9163, ILI9163_CMD_PWCTR4, &pwc4, 1);

    uint8_t pwc5[] = {0x01};
    ili9163_transmit(p_ili9163, ILI9163_CMD_PWCTR5, &pwc5, 1);

    uint8_t vcom1[] = {0x24, 0x48};
    ili9163_transmit(p_ili9163, ILI9163_CMD_VCOMCTR1, &vcom1, 2);

    uint8_t dfunc[] = {0x3F, 0x01};
    ili9163_transmit(p_ili9163, ILI9163_CMD_DFUNCTR, &dfunc, 2);

    uint8_t frmctr1[] = {0x08, 0x02};
    ili9163_transmit(p_ili9163, ILI9163_CMD_FRMCTR1, &frmctr1, 2);

    uint8_t frmctr2[] = {0x08, 0x02};
    ili9163_transmit(p_ili9163, ILI9163_CMD_FRMCTR2, &frmctr2, 2);

    uint8_t frmctr3[] = {0x08, 0x02};
    ili9163_transmit(p_ili9163, ILI9163_CMD_FRMCTR3, &frmctr3, 2);

    uint8_t pxl_fmt = {0x65};
    ili9163_transmit(p_ili9163, ILI9163_CMD_PIXFMT, &pxl_fmt, 1);

    uint8_t gammac[] = {0x08};
    ili9163_transmit(p_ili9163, ILI9163_CMD_GAMMASET, &gammac, 1);

    uint8_t gammad[] = {0x01};
    ili9163_transmit(p_ili9163, ILI9163_CMD_GAMRSEL, &gammad, 1);

    uint8_t pgammac[] = {0x3F, 0x26, 0x23, 0x30, 0x28, 0x10, 0x55, 0xB7, 0x40, 0x19, 0x10, 0x1E, 0x02, 0x01, 0x00};
    ili9163_transmit(p_ili9163, ILI9163_CMD_PGAMMAC, &pgammac, 15);

    uint8_t ngammac[] = {0x09, 0x18, 0x2D, 0x0D, 0x13, 0x15, 0x40, 0x48, 0x53, 0x0C, 0x1D, 0x25, 0x2E, 0x24, 0x29};
    ili9163_transmit(p_ili9163, ILI9163_CMD_NGAMMAC, &ngammac, 15);

    uint8_t coladdset[] = {0x00, 0x00, 0x00, 0x80};
    ili9163_transmit(p_ili9163, ILI9163_CMD_CLMADRS, &coladdset, 4);

    uint8_t pageaddset[] = {0x00, 0x00, 0x00, 0xA0};
    ili9163_transmit(p_ili9163, ILI9163_CMD_PGEADRS, &pageaddset, 4);

    uint8_t dinvctr[] = {0x07};
    ili9163_transmit(p_ili9163, ILI9163_CMD_DINVCTR, &dinvctr, 1);

    ili9163_transmit(p_ili9163,ILI9163_CMD_NORML, NULL, 0);

    ili9163_transmit(p_ili9163,ILI9163_CMD_DISPON, NULL, 0);

    //setRotation(0);
    //setScrollDirection(0);
    k_msleep(10);
    uint8_t vscrolldata[] = {0x00, 0x00, 0x00, 0xA0, 0x00, 0x00};
    ili9163_transmit(p_ili9163, ILI9163_CMD_VSCLLDEF, &vscrolldata, 6);
    uint8_t madctl[] = {0x08};
    ili9163_transmit(p_ili9163, ILI9163_CMD_MADCTL, &madctl, 1);
    
uint8_t cmd;
	/* Sleep Out */
	 cmd = ILI9163_CMD_IDLEOF;
	ili9163_transmit(p_ili9163, cmd, NULL, 0);

	k_sleep(K_MSEC(120));

	/* Display Off */
	 cmd = ILI9163_CMD_DISPOFF;
	ili9163_transmit(p_ili9163, cmd, NULL, 0);

        
        
        
}
