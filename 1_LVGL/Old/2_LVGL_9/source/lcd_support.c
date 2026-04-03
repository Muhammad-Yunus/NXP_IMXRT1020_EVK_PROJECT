/*
 * lcd_support.c
 *
 *  Created on: Jul 20, 2025
 *      Author: yunus
 */

#include "fsl_debug_console.h"
#include "lvgl.h"
#include "board.h"
#include "app.h"
#include "fsl_gpio.h"
#include "lcd_support.h"
#include "fsl_ili9341.h"
#include "fsl_lpspi.h"
#include "fsl_lpspi_edma.h"
#if defined(FSL_FEATURE_SOC_DMAMUX_COUNT) && FSL_FEATURE_SOC_DMAMUX_COUNT
#include "fsl_dmamux.h"
#endif
#include "fsl_common.h"
#include "fsl_cache.h"
#include "fsl_4pin_adc_touch.h"

#define LCD_DC_LOW()  GPIO_PortClear(BOARD_LCD_DC_GPIO, 1U << BOARD_LCD_DC_GPIO_PIN)
#define LCD_DC_HIGH() GPIO_PortSet(BOARD_LCD_DC_GPIO, 1U << BOARD_LCD_DC_GPIO_PIN)

AT_NONCACHEABLE_SECTION_ALIGN(static uint8_t s_frameBuffer[2][LCD_VIRTUAL_BUF_SIZE * LCD_FB_BYTE_PER_PIXEL], 64);

AT_NONCACHEABLE_SECTION_ALIGN(lpspi_master_edma_handle_t g_m_edma_handle, 64) = {0};
AT_NONCACHEABLE_SECTION_ALIGN(edma_handle_t lpspiEdmaMasterRxRegToRxDataHandle, 64);
AT_NONCACHEABLE_SECTION_ALIGN(edma_handle_t lpspiEdmaMasterTxDataToTxRegHandle, 64);

AT_NONCACHEABLE_SECTION_ALIGN(edma_config_t userConfig, 64);
AT_NONCACHEABLE_SECTION_ALIGN(lpspi_master_config_t masterConfig, 64);
AT_NONCACHEABLE_SECTION_ALIGN(lpspi_transfer_t masterXfer, 64);

volatile bool isTransferCompleted  = false;
static int touch_last_x = 0, touch_last_y = 0;

void LPSPI_MasterUserCallback(LPSPI_Type *base, lpspi_master_edma_handle_t *handle, status_t status, void *userData)
{
    if (status != kStatus_Success)
    {
        PRINTF("Error on LPSPI master edma transfer. \r\n");
    }

    isTransferCompleted = true;
}

void LCD_SPI_EDMA_Init(void)
{
	/*DMA Mux setting and EDMA init*/
#if defined(FSL_FEATURE_SOC_DMAMUX_COUNT) && FSL_FEATURE_SOC_DMAMUX_COUNT
#if defined(BOARD_LPSPI_DMAMUX_TX_CHANNEL) && defined(BOARD_LPSPI_DMAMUX_RX_CHANNEL)
	/* DMA MUX init*/
	DMAMUX_Init(EXAMPLE_LPSPI_RX_MASTER_DMA_MUX_BASE);

	DMAMUX_SetSource(EXAMPLE_LPSPI_RX_MASTER_DMA_MUX_BASE, BOARD_LPSPI_DMAMUX_RX_CHANNEL,
					 BOARD_LPSPI_DMA_RX_REQUEST_SOURCE);
	DMAMUX_EnableChannel(EXAMPLE_LPSPI_RX_MASTER_DMA_MUX_BASE, BOARD_LPSPI_DMAMUX_RX_CHANNEL);

	DMAMUX_Init(EXAMPLE_LPSPI_TX_MASTER_DMA_MUX_BASE);

	DMAMUX_SetSource(EXAMPLE_LPSPI_TX_MASTER_DMA_MUX_BASE, BOARD_LPSPI_DMAMUX_TX_CHANNEL,
					 BOARD_LPSPI_DMA_TX_REQUEST_SOURCE);
	DMAMUX_EnableChannel(EXAMPLE_LPSPI_TX_MASTER_DMA_MUX_BASE, BOARD_LPSPI_DMAMUX_TX_CHANNEL);
#else
	/* DMA MUX init*/
	DMAMUX_Init(BOARD_LPSPI_DMA_MUX_BASE);

	DMAMUX_SetSource(BOARD_LPSPI_DMA_MUX_BASE, BOARD_LPSPI_DMA_RX_CHANNEL,
					 BOARD_LPSPI_DMA_RX_REQUEST_SOURCE);
	DMAMUX_EnableChannel(BOARD_LPSPI_DMA_MUX_BASE, BOARD_LPSPI_DMA_RX_CHANNEL);

	DMAMUX_SetSource(BOARD_LPSPI_DMA_MUX_BASE, BOARD_LPSPI_DMA_TX_CHANNEL,
					 BOARD_LPSPI_DMA_TX_REQUEST_SOURCE);
	DMAMUX_EnableChannel(BOARD_LPSPI_DMA_MUX_BASE, BOARD_LPSPI_DMA_TX_CHANNEL);
#endif
#endif
	/* EDMA init*/
	EDMA_GetDefaultConfig(&userConfig);
#if defined(BOARD_GetEDMAConfig)
	BOARD_GetEDMAConfig(userConfig);
#endif
	EDMA_Init(BOARD_LPSPI_DMA_BASE, &userConfig);

    /*Master config*/
    LPSPI_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate = LPSPI_TRANSFER_BAUDRATE;
    masterConfig.whichPcs = BOARD_LPSPI_PCS_FOR_INIT;
    masterConfig.pcsToSckDelayInNanoSec        = 1000000000U / (masterConfig.baudRate * 2U);
    masterConfig.lastSckToPcsDelayInNanoSec    = 1000000000U / (masterConfig.baudRate * 2U);
    masterConfig.betweenTransferDelayInNanoSec = 1000000000U / (masterConfig.baudRate * 2U);

    LPSPI_MasterInit(BOARD_EEPROM_LPSPI_BASEADDR, &masterConfig, BOARD_LPSPI_CLK_FREQ);

    /*Set up lpspi master*/
    memset(&(lpspiEdmaMasterRxRegToRxDataHandle), 0, sizeof(lpspiEdmaMasterRxRegToRxDataHandle));
    memset(&(lpspiEdmaMasterTxDataToTxRegHandle), 0, sizeof(lpspiEdmaMasterTxDataToTxRegHandle));

    EDMA_CreateHandle(&(lpspiEdmaMasterRxRegToRxDataHandle), BOARD_LPSPI_DMA_BASE,
                      BOARD_LPSPI_DMA_RX_CHANNEL);
    EDMA_CreateHandle(&(lpspiEdmaMasterTxDataToTxRegHandle), BOARD_LPSPI_DMA_BASE,
                      BOARD_LPSPI_DMA_TX_CHANNEL);
#if defined(FSL_FEATURE_EDMA_HAS_CHANNEL_MUX) && FSL_FEATURE_EDMA_HAS_CHANNEL_MUX
    EDMA_SetChannelMux(BOARD_LPSPI_DMA_BASE, BOARD_LPSPI_DMA_TX_CHANNEL,
                       DEMO_LPSPI_TRANSMIT_EDMA_CHANNEL);
    EDMA_SetChannelMux(BOARD_LPSPI_DMA_BASE, BOARD_LPSPI_DMA_RX_CHANNEL,
                       DEMO_LPSPI_RECEIVE_EDMA_CHANNEL);
#endif
    LPSPI_MasterTransferCreateHandleEDMA(BOARD_EEPROM_LPSPI_BASEADDR, &g_m_edma_handle, LPSPI_MasterUserCallback,
                                         NULL,
										 &lpspiEdmaMasterRxRegToRxDataHandle,
                                         &lpspiEdmaMasterTxDataToTxRegHandle);

    LPSPI_MasterTransferPrepareEDMALite(BOARD_EEPROM_LPSPI_BASEADDR, &g_m_edma_handle,BOARD_LPSPI_PCS_FOR_TRANSFER | kLPSPI_MasterByteSwap | kLPSPI_MasterPcsContinuous);

}

void LCD_SPI_EDMA_TransferBlocking(const uint8_t *data, size_t length)
{
    lpspi_transfer_t xfer = {
        .txData = data,
        .rxData = NULL,
        .dataSize = length,
        .configFlags = kLPSPI_MasterPcs0 | kLPSPI_MasterPcsContinuous
    };

    isTransferCompleted = false;
    LPSPI_MasterTransferEDMALite(BOARD_EEPROM_LPSPI_BASEADDR, &g_m_edma_handle, &xfer);

    // Wait for transfer to finish
    while (!isTransferCompleted)
    {
    }
}

void LCD_WriteCmd(uint8_t cmd)
{
    LCD_DC_LOW();
    LCD_SPI_EDMA_TransferBlocking(&cmd, 1);
}

void LCD_WriteData(uint8_t data)
{
    LCD_DC_HIGH();
    LCD_SPI_EDMA_TransferBlocking(&data, 1);
}

void LCD_WriteMultiData(const uint8_t *data, size_t length)
{
    LCD_DC_HIGH();
    LCD_SPI_EDMA_TransferBlocking(data, length);
}

void LCD_Init(void){
	/* Define the init structure for the data/command and chip select output pin */
	gpio_pin_config_t dc_config = {
		kGPIO_DigitalOutput,
		1,
		kGPIO_NoIntmode,
	};

	/* Init data/command and chip select GPIO output . */
	GPIO_PinInit(BOARD_LCD_DC_GPIO, BOARD_LCD_DC_GPIO_PIN, &dc_config);

	LCD_SPI_EDMA_Init();
	ILI9341_Init(LCD_WriteData, LCD_WriteCmd);

    /* Change to landscape view. */
    LCD_WriteCmd(ILI9341_CMD_MAC);
    LCD_WriteData(0x28);

}

static void LCD_FlushDisplay(lv_display_t *disp_drv, const lv_area_t *area, uint8_t *color_p)
{

    lv_coord_t x1 = area->x1;
    lv_coord_t y1 = area->y1;
    lv_coord_t x2 = area->x2;
    lv_coord_t y2 = area->y2;

    uint8_t data[4];
    const uint8_t *pdata = (const uint8_t *)color_p;
    uint32_t send_size   = (x2 - x1 + 1) * (y2 - y1 + 1) * LCD_FB_BYTE_PER_PIXEL;

    /* Swap the 2 bytes of RGB565 color */
    //lv_draw_sw_rgb565_swap((uint16_t *)color_p, send_size);

    DCACHE_CleanByRange((uint32_t)color_p, send_size);

    /*Column addresses*/
    LCD_WriteCmd(ILI9341_CMD_COLADDR);
    data[0] = (x1 >> 8) & 0xFF;
    data[1] = x1 & 0xFF;
    data[2] = (x2 >> 8) & 0xFF;
    data[3] = x2 & 0xFF;
    LCD_WriteMultiData(data, 4);

    /*Page addresses*/
    LCD_WriteCmd(ILI9341_CMD_PAGEADDR);
    data[0] = (y1 >> 8) & 0xFF;
    data[1] = y1 & 0xFF;
    data[2] = (y2 >> 8) & 0xFF;
    data[3] = y2 & 0xFF;
    LCD_WriteMultiData(data, 4);

    /*Memory write*/
    LCD_WriteCmd(ILI9341_CMD_GRAM);
    LCD_WriteMultiData(pdata, send_size);

    lv_disp_flush_ready(disp_drv);
}

void lv_port_disp_init(void)
{
    lv_display_t * disp_drv; /*Descriptor of a display driver*/

    /*-------------------------
     * Initialize your display
     * -----------------------*/
    LCD_Init();

    disp_drv = lv_display_create(LCD_WIDTH, LCD_HEIGHT);

    lv_display_set_buffers(disp_drv, s_frameBuffer[0], s_frameBuffer[1], LCD_VIRTUAL_BUF_SIZE, LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_flush_cb(disp_drv, LCD_FlushDisplay);
}

/*Initialize your touchpad*/
static void LCD_InitTouch(void)
{
	TouchScreen_Init();
}

/* Will be called by the library to read the touchpad */
static void LCD_ReadTouch(lv_indev_t *drv, lv_indev_data_t *data)
{
    touch_point_t p = TouchScreen_GetPoint();
    if (p.z > 0) {
    	//PRINTF("x : %d\ty: %d\r\n", p.x, p.y);
    	touch_last_x = lv_map(p.x, LCD_TOUCH_MINX, LCD_TOUCH_MAXX, 0, LCD_WIDTH);
    	touch_last_y = lv_map(p.y, LCD_TOUCH_MINY, LCD_TOUCH_MAXY, 0, LCD_HEIGHT);
        data->state = LV_INDEV_STATE_PR;
    } else {
        data->state = LV_INDEV_STATE_REL;
    }
    data->point.x = touch_last_x;
    data->point.y = touch_last_y;
}
void lv_port_indev_init(void)
{
    /*Initialize your touchpad */
    LCD_InitTouch();

    /*Register a touchpad input device*/
    lv_indev_t * indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, LCD_ReadTouch);
}
