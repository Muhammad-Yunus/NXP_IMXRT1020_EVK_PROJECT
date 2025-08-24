/*
 * app.h
 *
 *  Created on: Jul 20, 2025
 *      Author: yunus
 */

#ifndef APP_H_
#define APP_H_


#define AT_SDRAM_SECTION_ALIGN(var, alignbytes) \
    __attribute__((section("BOARD_SDRAM"))) var __attribute__((aligned(alignbytes)))

#define LPSPI_TRANSFER_BAUDRATE 			20000000U /*! Transfer baudrate - 20MHz */

#define BOARD_EEPROM_LPSPI_BASEADDR 		(LPSPI1)
#define BOARD_LPSPI_IRQN            		LPSPI1_IRQn
#define BOARD_LPSPI_IRQHandler      		LPSPI1_IRQHandler

#define BOARD_LPSPI_DMA_MUX_BASE          	(DMAMUX)
#define BOARD_LPSPI_DMA_RX_REQUEST_SOURCE 	kDmaRequestMuxLPSPI1Rx
#define BOARD_LPSPI_DMA_TX_REQUEST_SOURCE 	kDmaRequestMuxLPSPI1Tx
#define BOARD_LPSPI_DMA_BASE              	(DMA0)
#define BOARD_LPSPI_DMA_RX_CHANNEL        	0U
#define BOARD_LPSPI_DMA_TX_CHANNEL        	1U

#define BOARD_LPSPI_PCS_FOR_INIT     		(kLPSPI_Pcs0)
#define BOARD_LPSPI_PCS_FOR_TRANSFER 		(kLPSPI_MasterPcs0)

/* Select USB1 PLL PFD0 (392.72 MHz) as lpspi clock source */
#define BOARD_LPSPI_CLOCK_SOURCE_SELECT 	(1U)
/* Clock divider for master lpspi clock source */
#define BOARD_LPSPI_CLOCK_SOURCE_DIVIDER 	(7U)

#define BOARD_LPSPI_CLK_FREQ (CLOCK_GetFreq(kCLOCK_Usb1PllPfd0Clk) / (BOARD_LPSPI_CLOCK_SOURCE_DIVIDER + 1U))



#endif /* APP_H_ */
