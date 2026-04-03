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
#define LCD_CS_LOW()  GPIO_PortClear(BOARD_LCD_CS_GPIO, 1U << BOARD_LCD_CS_GPIO_PIN)
#define LCD_CS_HIGH() GPIO_PortSet(BOARD_LCD_CS_GPIO, 1U << BOARD_LCD_CS_GPIO_PIN)

/*
 * Frame buffers in cacheable SDRAM for fast CPU rendering.
 * DCACHE_CleanByRange() is called before DMA to ensure coherency.
 */
AT_SDRAM_SECTION_ALIGN(static uint8_t s_frameBuffer[2][LCD_VIRTUAL_BUF_SIZE * LCD_FB_BYTE_PER_PIXEL], 64);

/* DMA handles must remain in non-cacheable memory */
AT_NONCACHEABLE_SECTION_ALIGN(lpspi_master_edma_handle_t g_m_edma_handle, 64) = {0};
AT_NONCACHEABLE_SECTION_ALIGN(edma_handle_t lpspiEdmaMasterRxRegToRxDataHandle, 64);
AT_NONCACHEABLE_SECTION_ALIGN(edma_handle_t lpspiEdmaMasterTxDataToTxRegHandle, 64);

AT_NONCACHEABLE_SECTION_ALIGN(edma_config_t userConfig, 64);
AT_NONCACHEABLE_SECTION_ALIGN(lpspi_master_config_t masterConfig, 64);
AT_NONCACHEABLE_SECTION_ALIGN(lpspi_transfer_t masterXfer, 64);

volatile bool isTransferCompleted  = true;
static int touch_last_x = 0, touch_last_y = 0;

/* Async DMA state */
static volatile bool s_dmaActive   = false;
static lv_display_t *s_pendingDisp = NULL;
static const uint8_t *s_tailPtr    = NULL;
static uint32_t       s_tailLen    = 0;

/* Forward declaration for DMA completion callback */
static void LCD_DMA_Done_CB(edma_handle_t *handle, void *userData,
                            bool transferDone, uint32_t tcds);

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

}

/*
 * Wait for TX FIFO drain + last frame to shift out.
 * With CONT=1, MBF stays set after FIFO empties (transfer "ongoing").
 * Briefly disable the module — LPSPI finishes the current frame,
 * MBF clears, then re-enable for the next operation.
 */
static inline void LCD_SPI_WaitDone(void)
{
    LPSPI_Type *spi = BOARD_EEPROM_LPSPI_BASEADDR;
    while (spi->FSR & LPSPI_FSR_TXCOUNT_MASK) {}
    spi->CR &= ~LPSPI_CR_MEN_MASK;
    while (spi->SR & LPSPI_SR_MBF_MASK) {}
    spi->CR |= LPSPI_CR_MEN_MASK;
}

void LCD_WriteCmd(uint8_t cmd)
{
    LCD_CS_LOW();
    LCD_DC_LOW();
    *(volatile uint8_t *)((uint32_t)&BOARD_EEPROM_LPSPI_BASEADDR->TDR + 3U) = cmd;
    LCD_SPI_WaitDone();
    LCD_CS_HIGH();
}

void LCD_WriteData(uint8_t data)
{
    LCD_CS_LOW();
    LCD_DC_HIGH();
    *(volatile uint8_t *)((uint32_t)&BOARD_EEPROM_LPSPI_BASEADDR->TDR + 3U) = data;
    LCD_SPI_WaitDone();
    LCD_CS_HIGH();
}

void LCD_WriteMultiData(const uint8_t *data, size_t length)
{
    volatile uint8_t *tdr = (volatile uint8_t *)((uint32_t)&BOARD_EEPROM_LPSPI_BASEADDR->TDR + 3U);
    LCD_CS_LOW();
    LCD_DC_HIGH();
    for (size_t i = 0; i < length; i++)
        *tdr = data[i];
    LCD_SPI_WaitDone();
    LCD_CS_HIGH();
}

void LCD_Init(void){
	/* Define the init structure for the data/command and chip select output pin */
	gpio_pin_config_t dc_config = {
		kGPIO_DigitalOutput,
		1,
		kGPIO_NoIntmode,
	};

	/* Init data/command GPIO output . */
	GPIO_PinInit(BOARD_LCD_DC_GPIO, BOARD_LCD_DC_GPIO_PIN, &dc_config);

	/* Init chip select GPIO output, default HIGH (deasserted). */
	gpio_pin_config_t cs_config = {
		kGPIO_DigitalOutput,
		1,
		kGPIO_NoIntmode,
	};
	GPIO_PinInit(BOARD_LCD_CS_GPIO, BOARD_LCD_CS_GPIO_PIN, &cs_config);

	LCD_SPI_EDMA_Init();

	/*
	 * Set TCR permanently for direct-write + DMA mode:
	 *   CONT=1   – continuous transfer, no inter-frame PCS delays (~37% faster)
	 *   CONTC=1  – reuse command word, no new TCR per frame
	 *   BYSW=1   – byte swap for correct byte ordering via TDR+3
	 *   RXMSK=1  – mask RX, TX-only, no RX FIFO overflow
	 * Eliminates per-flush TCR save/restore overhead entirely.
	 */
	LPSPI_Type *spi = BOARD_EEPROM_LPSPI_BASEADDR;
	uint32_t tcr = LPSPI_GetTcr(spi);
	LPSPI_Enable(spi, false);
	tcr |= LPSPI_TCR_CONT_MASK | LPSPI_TCR_CONTC_MASK |
	       LPSPI_TCR_BYSW_MASK | LPSPI_TCR_RXMSK_MASK;
	spi->TCR = tcr;
	spi->FCR = LPSPI_FCR_TXWATER(0);
	LPSPI_Enable(spi, true);

	/* Set up DMA completion interrupt for async pixel transfers */
	EDMA_SetCallback(&lpspiEdmaMasterTxDataToTxRegHandle,
	                 LCD_DMA_Done_CB, NULL);
	EnableIRQ(DMA1_DMA17_IRQn);

	ILI9341_Init(LCD_WriteData, LCD_WriteCmd);

    /* Change to landscape view. */
    LCD_WriteCmd(ILI9341_CMD_MAC);
    LCD_WriteData(0x28);

}

/*
 * DMA major-loop-complete ISR callback.
 * Called from EDMA_HandleIRQ(s_EDMAHandle[1]) via DMA1_DMA17_IRQHandler.
 * Finishes the SPI transfer (FIFO drain, tail bytes, MEN toggle),
 * deasserts CS, and signals LVGL that the buffer is free.
 */
static void LCD_DMA_Done_CB(edma_handle_t *handle, void *userData,
                            bool transferDone, uint32_t tcds)
{
    (void)handle; (void)userData; (void)transferDone; (void)tcds;
    LPSPI_Type *spi = BOARD_EEPROM_LPSPI_BASEADDR;

    LPSPI_DisableDMA(spi, kLPSPI_TxDmaEnable);

    /* Tail: at most 15 bytes left over from non-aligned length */
    if (s_tailLen > 0U)
    {
        volatile uint8_t *tdr =
            (volatile uint8_t *)((uint32_t)&spi->TDR + 3U);
        for (uint32_t i = 0; i < s_tailLen; i++)
            *tdr = s_tailPtr[i];
        s_tailLen = 0;
    }

    /* Wait for TX FIFO to drain */
    while (spi->FSR & LPSPI_FSR_TXCOUNT_MASK) {}

    /* End continuous transfer: MEN off → wait MBF clear → MEN on */
    spi->CR &= ~LPSPI_CR_MEN_MASK;
    while (spi->SR & LPSPI_SR_MBF_MASK) {}
    spi->CR |= LPSPI_CR_MEN_MASK;

    LCD_CS_HIGH();

    /* Tell LVGL the flush buffer is free → rendering can start on it */
    if (s_pendingDisp)
    {
        lv_disp_flush_ready(s_pendingDisp);
        s_pendingDisp = NULL;
    }

    s_dmaActive = false;
}

static void LCD_SPI_DMA_StartAsync(const uint8_t *data, uint32_t length)
{
    LPSPI_Type *spi = BOARD_EEPROM_LPSPI_BASEADDR;
    DMA_Type   *dma = BOARD_LPSPI_DMA_BASE;
    const uint32_t txCh = BOARD_LPSPI_DMA_TX_CHANNEL;
    const uint32_t MINOR_BYTES = 16U;

    uint32_t bulk = length & ~(MINOR_BYTES - 1U);
    uint32_t tail = length - bulk;

    /* Save tail info for ISR */
    s_tailPtr = data + bulk;
    s_tailLen = tail;

    /* Reset eDMA channel (clears TCD, disables request) */
    EDMA_ResetChannel(dma, txCh);

    if (bulk > 0U)
    {
        edma_transfer_config_t xferCfg;
        EDMA_PrepareTransferConfig(
            &xferCfg,
            (void *)(uint32_t)data, 1, 1,
            (void *)((uint32_t)&spi->TDR + 3U), 1, 0,
            MINOR_BYTES, bulk
        );
        EDMA_SetTransferConfig(dma, txCh, &xferCfg, NULL);
        EDMA_EnableChannelInterrupts(dma, txCh, kEDMA_MajorInterruptEnable);

        LPSPI_EnableDMA(spi, kLPSPI_TxDmaEnable);
        EDMA_EnableChannelRequest(dma, txCh);
        /* Returns immediately — ISR handles completion */
    }
    else
    {
        /* No bulk data — handle everything synchronously (unlikely for pixels) */
        volatile uint8_t *tdr = (volatile uint8_t *)((uint32_t)&spi->TDR + 3U);
        for (uint32_t i = 0; i < tail; i++)
            *tdr = data[i];
        while (spi->FSR & LPSPI_FSR_TXCOUNT_MASK) {}
        spi->CR &= ~LPSPI_CR_MEN_MASK;
        while (spi->SR & LPSPI_SR_MBF_MASK) {}
        spi->CR |= LPSPI_CR_MEN_MASK;
        LCD_CS_HIGH();
        s_tailLen = 0;
        if (s_pendingDisp)
        {
            lv_disp_flush_ready(s_pendingDisp);
            s_pendingDisp = NULL;
        }
        s_dmaActive = false;
    }
}

static void LCD_FlushDisplay(lv_display_t *disp_drv, const lv_area_t *area, uint8_t *color_p)
{
    /* Wait for any previous async DMA to finish */
    while (s_dmaActive) {}

    lv_coord_t x1 = area->x1;
    lv_coord_t y1 = area->y1;
    lv_coord_t x2 = area->x2;
    lv_coord_t y2 = area->y2;

    uint8_t data[4];
    const uint8_t *pdata = (const uint8_t *)color_p;
    uint32_t send_size   = (x2 - x1 + 1) * (y2 - y1 + 1) * LCD_FB_BYTE_PER_PIXEL;

    /* Flush D-cache to ensure DMA reads the latest pixel data from SDRAM */
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

    LCD_CS_LOW();
    LCD_DC_HIGH();

    /* Start async DMA — ISR will call lv_disp_flush_ready() */
    s_pendingDisp = disp_drv;
    s_dmaActive   = true;
    LCD_SPI_DMA_StartAsync(pdata, send_size);
}

void lv_port_disp_init(void)
{
    lv_display_t * disp_drv; /*Descriptor of a display driver*/

    /*-------------------------
     * Initialize your display
     * -----------------------*/
    LCD_Init();

    disp_drv = lv_display_create(LCD_WIDTH, LCD_HEIGHT);

    lv_display_set_buffers(
    		disp_drv,
    		s_frameBuffer[0],
			s_frameBuffer[1],
			LCD_VIRTUAL_BUF_SIZE * LCD_FB_BYTE_PER_PIXEL,
			LV_DISPLAY_RENDER_MODE_PARTIAL);
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
