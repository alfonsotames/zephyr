/*
 * Copyright (c) 2017 Jan Van Winkel <jan.van_winkel@dxplore.eu>
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ilitek_ili9163

#include "display_ili9163.h"
#include <drivers/display.h>

#define LOG_LEVEL CONFIG_DISPLAY_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(display_ili9163);

#include <drivers/gpio.h>
#include <sys/byteorder.h>
#include <drivers/spi.h>
#include <string.h>

struct ili9163_data {
#if DT_INST_NODE_HAS_PROP(0, reset_gpios)
	struct device *reset_gpio;
#endif
	struct device *command_data_gpio;
	struct device *spi_dev;
	struct spi_config spi_config;
#if DT_INST_SPI_DEV_HAS_CS_GPIOS(0)
	struct spi_cs_control cs_ctrl;
#endif
};

#define ILI9163_CMD_DATA_PIN_COMMAND 0
#define ILI9163_CMD_DATA_PIN_DATA 1


// The number of bytes taken by a RGB pixel 
#ifdef CONFIG_ILI9163_RGB565
#define ILI9163_RGB_SIZE 2U
#else
#define ILI9163_RGB_SIZE 3U
#endif

        


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



void ili9163_lcd_init(struct ili9163_data *p_ili9163) {

    LOG_INF("ili9163_lcd_init...");


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
        
        
        LOG_INF("ili9163_lcd_init complete...");

        ili9163_transmit(p_ili9163, ILI9163_CMD_DISPON,NULL, 0);
        ili9163_transmit(p_ili9163, ILI9163_CMD_RAMWR, NULL, 0);
        
        
}














        
static void ili9163_exit_sleep(struct ili9163_data *data)
{
	ili9163_transmit(data, ILI9163_CMD_EXIT_SLEEP, NULL, 0);
	k_sleep(K_MSEC(120));
}

static int ili9163_init(struct device *dev)
{
	struct ili9163_data *data = (struct ili9163_data *)dev->driver_data;

	LOG_DBG("Initializing display driver");

	data->spi_dev = device_get_binding(DT_INST_BUS_LABEL(0));
	if (data->spi_dev == NULL) {
		LOG_ERR("Could not get SPI device for ILI9163");
		return -EPERM;
	}

	data->spi_config.frequency = DT_INST_PROP(0, spi_max_frequency);
	data->spi_config.operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8);
	data->spi_config.slave = DT_INST_REG_ADDR(0);

#if DT_INST_SPI_DEV_HAS_CS_GPIOS(0)
	data->cs_ctrl.gpio_dev =
		device_get_binding(DT_INST_SPI_DEV_CS_GPIOS_LABEL(0));
	data->cs_ctrl.gpio_pin = DT_INST_SPI_DEV_CS_GPIOS_PIN(0);
	data->cs_ctrl.delay = 0U;
	data->spi_config.cs = &(data->cs_ctrl);
#else
	data->spi_config.cs = NULL;
#endif

#if DT_INST_NODE_HAS_PROP(0, reset_gpios)
	data->reset_gpio =
		device_get_binding(DT_INST_GPIO_LABEL(0, reset_gpios));
	if (data->reset_gpio == NULL) {
		LOG_ERR("Could not get GPIO port for ILI9163 reset");
		return -EPERM;
	}

	gpio_pin_configure(data->reset_gpio, DT_INST_GPIO_PIN(0, reset_gpios),
			   GPIO_OUTPUT_INACTIVE |
			   DT_INST_GPIO_FLAGS(0, reset_gpios));
#endif

	data->command_data_gpio =
		device_get_binding(DT_INST_GPIO_LABEL(0, cmd_data_gpios));
	if (data->command_data_gpio == NULL) {
		LOG_ERR("Could not get GPIO port for ILI9163 command/data");
		return -EPERM;
	}

	gpio_pin_configure(data->command_data_gpio, DT_INST_GPIO_PIN(0, cmd_data_gpios),
			   GPIO_OUTPUT |
			   DT_INST_GPIO_FLAGS(0, cmd_data_gpios));

#if DT_INST_NODE_HAS_PROP(0, reset_gpios)
	LOG_DBG("Resetting display driver");
	k_sleep(K_MSEC(1));
	gpio_pin_set(data->reset_gpio,
		     DT_INST_GPIO_PIN(0, reset_gpios), 1);
	k_sleep(K_MSEC(1));
	gpio_pin_set(data->reset_gpio,
		     DT_INST_GPIO_PIN(0, reset_gpios), 0);
	k_sleep(K_MSEC(5));
#endif

	LOG_DBG("Initializing LCD");
	ili9163_lcd_init(data);

	LOG_DBG("Exiting sleep mode");
	ili9163_exit_sleep(data);

	return 0;
}

static void ili9163_set_mem_area(struct ili9163_data *data, const uint16_t x,
				 const uint16_t y, const uint16_t w, const uint16_t h)
{
	uint16_t spi_data[2];

	spi_data[0] = sys_cpu_to_be16(x);
	spi_data[1] = sys_cpu_to_be16(x + w - 1);
	ili9163_transmit(data, ILI9163_CMD_COLUMN_ADDR, &spi_data[0], 4);

	spi_data[0] = sys_cpu_to_be16(y);
	spi_data[1] = sys_cpu_to_be16(y + h - 1);
	ili9163_transmit(data, ILI9163_CMD_PAGE_ADDR, &spi_data[0], 4);
}

static int ili9163_write(const struct device *dev, const uint16_t x,
			 const uint16_t y,
			 const struct display_buffer_descriptor *desc,
			 const void *buf)
{
	struct ili9163_data *data = (struct ili9163_data *)dev->driver_data;
	const uint8_t *write_data_start = (uint8_t *) buf;
	struct spi_buf tx_buf;
	struct spi_buf_set tx_bufs;
	uint16_t write_cnt;
	uint16_t nbr_of_writes;
	uint16_t write_h;

	__ASSERT(desc->width <= desc->pitch, "Pitch is smaller then width");
	__ASSERT((desc->pitch * ILI9163_RGB_SIZE * desc->height) <= desc->bu_size,
			"Input buffer to small");

	LOG_DBG("Writing %dx%d (w,h) @ %dx%d (x,y)", desc->width, desc->height,
			x, y);
	ili9163_set_mem_area(data, x, y, desc->width, desc->height);

	if (desc->pitch > desc->width) {
		write_h = 1U;
		nbr_of_writes = desc->height;
	} else {
		write_h = desc->height;
		nbr_of_writes = 1U;
	}

	ili9163_transmit(data, ILI9163_CMD_MEM_WRITE,
			 (void *) write_data_start,
			 desc->width * ILI9163_RGB_SIZE * write_h);

	tx_bufs.buffers = &tx_buf;
	tx_bufs.count = 1;

	write_data_start += (desc->pitch * ILI9163_RGB_SIZE);
	for (write_cnt = 1U; write_cnt < nbr_of_writes; ++write_cnt) {
		tx_buf.buf = (void *)write_data_start;
		tx_buf.len = desc->width * ILI9163_RGB_SIZE * write_h;
		spi_write(data->spi_dev, &data->spi_config, &tx_bufs);
		write_data_start += (desc->pitch * ILI9163_RGB_SIZE);
	}

	return 0;
}

static int ili9163_read(const struct device *dev, const uint16_t x,
			const uint16_t y,
			const struct display_buffer_descriptor *desc,
			void *buf)
{
	LOG_ERR("Reading not supported");
	return -ENOTSUP;
}

static void *ili9163_get_framebuffer(const struct device *dev)
{
	LOG_ERR("Direct framebuffer access not supported");
	return NULL;
}

static int ili9163_display_blanking_off(const struct device *dev)
{
	struct ili9163_data *data = (struct ili9163_data *)dev->driver_data;

	LOG_DBG("Turning display blanking off");
	ili9163_transmit(data, ILI9163_CMD_DISPLAY_ON, NULL, 0);
	return 0;
}

static int ili9163_display_blanking_on(const struct device *dev)
{
	struct ili9163_data *data = (struct ili9163_data *)dev->driver_data;

	LOG_DBG("Turning display blanking on");
	ili9163_transmit(data, ILI9163_CMD_DISPLAY_OFF, NULL, 0);
	return 0;
}

static int ili9163_set_brightness(const struct device *dev,
				  const uint8_t brightness)
{
	LOG_WRN("Set brightness not implemented");
	return -ENOTSUP;
}

static int ili9163_set_contrast(const struct device *dev, const uint8_t contrast)
{
	LOG_ERR("Set contrast not supported");
	return -ENOTSUP;
}

static int ili9163_set_pixel_format(const struct device *dev,
				    const enum display_pixel_format
				    pixel_format)
{
#ifdef CONFIG_ILI9163_RGB565
	if (pixel_format == PIXEL_FORMAT_RGB_565) {
#else
	if (pixel_format == PIXEL_FORMAT_RGB_888) {
#endif
		return 0;
	}
	LOG_ERR("Pixel format change not implemented");
	return -ENOTSUP;
}

static int ili9163_set_orientation(const struct device *dev,
				   const enum display_orientation orientation)
{
	if (orientation == DISPLAY_ORIENTATION_NORMAL) {
		return 0;
	}
	LOG_ERR("Changing display orientation not implemented");
	return -ENOTSUP;
}

static void ili9163_get_capabilities(const struct device *dev,
				     struct display_capabilities *capabilities)
{
	memset(capabilities, 0, sizeof(struct display_capabilities));
	capabilities->x_resolution = 320U;
	capabilities->y_resolution = 240U;
#ifdef CONFIG_ILI9163_RGB565
	capabilities->supported_pixel_formats = PIXEL_FORMAT_RGB_565;
	capabilities->current_pixel_format = PIXEL_FORMAT_RGB_565;
#else
	capabilities->supported_pixel_formats = PIXEL_FORMAT_RGB_888;
	capabilities->current_pixel_format = PIXEL_FORMAT_RGB_888;
#endif
	capabilities->current_orientation = DISPLAY_ORIENTATION_NORMAL;
}

void ili9163_transmit(struct ili9163_data *data, uint8_t cmd, void *tx_data,
		      size_t tx_len)
{
	struct spi_buf tx_buf = { .buf = &cmd, .len = 1 };
	struct spi_buf_set tx_bufs = { .buffers = &tx_buf, .count = 1 };

	gpio_pin_set(data->command_data_gpio,
		     DT_INST_GPIO_PIN(0, cmd_data_gpios),
		     ILI9163_CMD_DATA_PIN_COMMAND);
	spi_write(data->spi_dev, &data->spi_config, &tx_bufs);

	if (tx_data != NULL) {
		tx_buf.buf = tx_data;
		tx_buf.len = tx_len;
		gpio_pin_set(data->command_data_gpio,
			     DT_INST_GPIO_PIN(0, cmd_data_gpios),
			     ILI9163_CMD_DATA_PIN_DATA);
		spi_write(data->spi_dev, &data->spi_config, &tx_bufs);
	}
}

static const struct display_driver_api ili9163_api = {
	.blanking_on = ili9163_display_blanking_on,
	.blanking_off = ili9163_display_blanking_off,
	.write = ili9163_write,
	.read = ili9163_read,
	.get_framebuffer = ili9163_get_framebuffer,
	.set_brightness = ili9163_set_brightness,
	.set_contrast = ili9163_set_contrast,
	.get_capabilities = ili9163_get_capabilities,
	.set_pixel_format = ili9163_set_pixel_format,
	.set_orientation = ili9163_set_orientation,
};

static struct ili9163_data ili9163_data;

DEVICE_AND_API_INIT(ili9163, DT_INST_LABEL(0), &ili9163_init,
		    &ili9163_data, NULL, APPLICATION,
		    CONFIG_APPLICATION_INIT_PRIORITY, &ili9163_api);
