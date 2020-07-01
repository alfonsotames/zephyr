/*
 * Copyright (c) 2017 Jan Van Winkel <jan.van_winkel@dxplore.eu>
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_DRIVERS_DISPLAY_DISPLAY_ILI9163_H_
#define ZEPHYR_DRIVERS_DISPLAY_DISPLAY_ILI9163_H_

#include <zephyr.h>

//ILI9163C registers-----------------------
#define ILI9163_CMD_NOP     	0x00//Non operation
#define ILI9163_CMD_SWRESET 	0x01//Soft Reset
#define ILI9163_CMD_SLPIN   	0x10//Sleep ON
#define ILI9163_CMD_SLPOUT  	0x11//Sleep OFF
#define ILI9163_CMD_PTLON   	0x12//Partial Mode ON
#define ILI9163_CMD_NORML   	0x13//Normal Display ON
#define ILI9163_CMD_DINVOF  	0x20//Display Inversion OFF
#define ILI9163_CMD_DINVON   	0x21//Display Inversion ON
#define ILI9163_CMD_GAMMASET 	0x26//Gamma Set (0x01[1],0x02[2],0x04[3],0x08[4])
#define ILI9163_CMD_DISPOFF 	0x28//Display OFF
#define ILI9163_CMD_DISPON  	0x29//Display ON
#define ILI9163_CMD_IDLEON  	0x39//Idle Mode ON
#define ILI9163_CMD_IDLEOF  	0x38//Idle Mode OFF
#define ILI9163_CMD_CLMADRS   0x2A//Column Address Set
#define ILI9163_CMD_PGEADRS   0x2B//Page Address Set

#define ILI9163_CMD_RAMWR   	0x2C//Memory Write
#define ILI9163_CMD_RAMRD   	0x2E//Memory Read
#define ILI9163_CMD_CLRSPACE  0x2D//Color Space : 4K/65K/262K
#define ILI9163_CMD_PARTAREA	0x30//Partial Area
#define ILI9163_CMD_VSCLLDEF	0x33//Vertical Scroll Definition
#define ILI9163_CMD_TEFXLON	0x35//Tearing Effect Line ON
#define ILI9163_CMD_TEFXLOF	0x34//Tearing Effect Line OFF
#define ILI9163_CMD_MADCTL  	0x36//Memory Access Control
#define ILI9163_CMD_VSSTADRS	0x37//Vertical Scrolling Start address
#define ILI9163_CMD_PIXFMT  	0x3A//Interface Pixel Format
#define ILI9163_CMD_FRMCTR1 	0xB1//Frame Rate Control (In normal mode/Full colors)
#define ILI9163_CMD_FRMCTR2 	0xB2//Frame Rate Control(In Idle mode/8-colors)
#define ILI9163_CMD_FRMCTR3 	0xB3//Frame Rate Control(In Partial mode/full colors)
#define ILI9163_CMD_DINVCTR	0xB4//Display Inversion Control
#define ILI9163_CMD_RGBBLK	0xB5//RGB Interface Blanking Porch setting
#define ILI9163_CMD_DFUNCTR 	0xB6//Display Fuction set 5
#define ILI9163_CMD_SDRVDIR 	0xB7//Source Driver Direction Control
#define ILI9163_CMD_GDRVDIR 	0xB8//Gate Driver Direction Control 

#define ILI9163_CMD_PWCTR1  	0xC0//Power_Control1
#define ILI9163_CMD_PWCTR2  	0xC1//Power_Control2
#define ILI9163_CMD_PWCTR3  	0xC2//Power_Control3
#define ILI9163_CMD_PWCTR4  	0xC3//Power_Control4
#define ILI9163_CMD_PWCTR5  	0xC4//Power_Control5
#define ILI9163_CMD_VCOMCTR1  0xC5//VCOM_Control 1
#define ILI9163_CMD_VCOMCTR2  0xC6//VCOM_Control 2
#define ILI9163_CMD_VCOMOFFS  0xC7//VCOM Offset Control
#define ILI9163_CMD_PGAMMAC	0xE0//Positive Gamma Correction Setting
#define ILI9163_CMD_NGAMMAC	0xE1//Negative Gamma Correction Setting
#define ILI9163_CMD_GAMRSEL	0xF2//GAM_R_SEL

#define ILI9163_CMD_SOFTWARE_RESET 0x01
#define ILI9163_CMD_ENTER_SLEEP 0x10
#define ILI9163_CMD_EXIT_SLEEP 0x11
#define ILI9163_CMD_GAMMA_SET 0x26
#define ILI9163_CMD_DISPLAY_OFF 0x28
#define ILI9163_CMD_DISPLAY_ON 0x29
#define ILI9163_CMD_COLUMN_ADDR 0x2a
#define ILI9163_CMD_PAGE_ADDR 0x2b
#define ILI9163_CMD_MEM_WRITE 0x2c
#define ILI9163_CMD_MEM_ACCESS_CTRL 0x36
#define ILI9163_CMD_PIXEL_FORMAT_SET 0x3A
#define ILI9163_CMD_FRAME_CTRL_NORMAL_MODE 0xB1
#define ILI9163_CMD_DISPLAY_FUNCTION_CTRL 0xB6
#define ILI9163_CMD_POWER_CTRL_1 0xC0
#define ILI9163_CMD_POWER_CTRL_2 0xC1
#define ILI9163_CMD_VCOM_CTRL_1 0xC5
#define ILI9163_CMD_VCOM_CTRL_2 0xC7
#define ILI9163_CMD_POSITIVE_GAMMA_CORRECTION 0xE0
#define ILI9163_CMD_NEGATIVE_GAMMA_CORRECTION 0xE1

#define ILI9341_CMD_POWER_CTRL_A 0xCB
#define ILI9341_CMD_POWER_CTRL_B 0xCF
#define ILI9341_CMD_DRVR_TIMING_CTRL_A_I 0xE8
#define ILI9341_CMD_DRVR_TIMING_CTRL_A_E 0xE9
#define ILI9341_CMD_DRVR_TIMING_CTRL_B 0xEA
#define ILI9341_CMD_POWER_ON_SEQ_CTRL 0xED
#define ILI9341_CMD_ENABLE_3G 0xF2
#define ILI9341_CMD_PUMP_RATIO_CTRL 0xF7

#define ILI9163_DATA_MEM_ACCESS_CTRL_MY 0x80
#define ILI9163_DATA_MEM_ACCESS_CTRL_MX 0x40
#define ILI9163_DATA_MEM_ACCESS_CTRL_MV 0x20
#define ILI9163_DATA_MEM_ACCESS_CTRL_ML 0x10
#define ILI9163_DATA_MEM_ACCESS_CTRL_BGR 0x08
#define ILI9163_DATA_MEM_ACCESS_CTRL_MH 0x04

#define ILI9163_DATA_PIXEL_FORMAT_RGB_18_BIT 0x60
#define ILI9163_DATA_PIXEL_FORMAT_RGB_16_BIT 0x50
#define ILI9163_DATA_PIXEL_FORMAT_MCU_18_BIT 0x06
#define ILI9163_DATA_PIXEL_FORMAT_MCU_16_BIT 0x05





struct ili9163_data;

/**
 * Send data to ILI9163 display controller
 *
 * @param data Device data structure
 * @param cmd Command to send to display controller
 * @param tx_data Data to transmit to the display controller
 * In case no data should be transmitted pass a NULL pointer
 * @param tx_len Number of bytes in tx_data buffer
 *
 */
void ili9163_transmit(struct ili9163_data *data, uint8_t cmd, void *tx_data,
		      size_t tx_len);

/**
 * Perform LCD specific initialization
 *
 * @param data Device data structure
 */
void ili9163_lcd_init(struct ili9163_data *data);

#endif /* ZEPHYR_DRIVERS_DISPLAY_DISPLAY_ILI9163_H_ */
