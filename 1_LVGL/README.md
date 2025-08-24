
# Setup NXP IMX RT1020 LVGL Demo Project 
- LCD : Seeed Studio 2.8inch TFT Touch Shield v2.0 (ILI9341)
1. Create New Project,
	- Add `lpspi` & `lpspi_edma` driver
	- Add `adc` driver
	- Add `cache` driver
2. Open **Project** > **MCU Xpresso Config Tool** > **Pin** 
	- Setup `LPSPI1` Pin 
		- LCD_CS : `[97] GPIO_AD_B0_11`
		- LCD_CLK : `[98] GPIO_AD_B0_10`
		- LCD_MISO : `[95] GPIO_AD_B0_13`
		- LCD_MOSI : `[96] GPIO_AD_B0_12`
	- Setup `GPIO1` Pin 
		- LCD_DC : `[94] GPIO_AD_B0_14`
	- Setup `ADC1` pin 
		- LCD_TOUCH_YM_ADC_A0 : `[80] GPIO_AD_B1_10`
		- LCD_TOUCH_XM_ADC_A1 : `[79] GPIO_AD_B1_11`
		- LCD_TOUCH_YP_ADC_A2 : `[78] GPIO_AD_B1_12`
		- LCD_TOUCH_XP_ADC_A3 : `[76] GPIO_AD_B1_13`
3. Create new header file `board/app.h`
    ```
    #define LPSPI_TRANSFER_BAUDRATE 			10000000U /*! Transfer baudrate - 10MHz*/

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
    ```
4. Open `board/board.h`, then edit line 108,
	- set macros `BOARD_LCD_DC_GPIO_PIN` to `14U`. 
5. Clone **LVGL** project : https://github.com/lvgl/lvgl 
    - use version **9.2.1** or **8.3.10**
	- copy `lvgl/lv_conf_template.h`
	- copy `lvgl/lv_version.h`
	- copy `lvgl/lvgl.h`
	- copy `lvgl/src/`
	- copy `lvgl/demos/` (optional)
6. Source LVGL,
	- Select **Project** > **New** > **Add Source Folder** > choose **lvgl directory**
	- Select **Project** > **Properties** > **C/C++ Build** > **Settings** > **MCU C Compiler** > **Includes** > Add New **Include Path** for LVGL directory
7. Add LCD Driver,
	- copy `drivers/fsl_ili9341.h` & `drivers/fsl_ili9341.c`
	- copy `drivers/fsl_4pin_adc_touch.h` & `drivers/fsl_4pin_adc_touch.c`
8. Add lvgl support ,
	- copy `source/lv_conf.h`
	- copy `source/lcd_support.h`
	- copy `source/lcd_support.c`
9. Modify main code, 
	- Add 
        ```	
        #include "lvgl.h"
        #include "lcd_support.h"
        #include "demos/lv_demos.h"

        void SysTick_Handler(void)
        {
            lv_tick_inc(1);
        }
        ```<br><br>
	- Replace,
        ```	
        PRINTF("Hello World\r\n");

        /* Force the counter to be placed into memory. */
        volatile static int i = 0 ;
        /* Enter an infinite loop, just incrementing a counter. */
        while(1) {
            i++ ;
            /* 'Dummy' NOP to allow source level single stepping of
                tight while() loop */
            __asm volatile ("nop");
        }
        return 0 ;
        ```
	- with,
        ```
            SysTick_Config(SystemCoreClock / 1000);

            lv_init();
            lv_port_disp_init();
            lv_port_indev_init();

        #if LV_USE_LOG
            lv_log_register_print_cb(print_cb);
        #endif

            LV_LOG("LVGL demo started\r\n");

            // LVGL Demo,
            /* enable the flag also in lv_conf.h */
            //lv_demo_widgets();
            //lv_demo_stress();
            lv_demo_music();

            for (;;)
            {
                lv_timer_handler();
            }
        ```
10. Additional project setting :
	- Select **Project** > **Properties** > **C/C++ General** > **Paths and Symbols** > Add **XIP_BOOT_HEADER_DCD_ENABLE** with value **1**,
	- Select **Project** > **Properties** > **C/C++ General** > **Paths and Symbols** > Add **SKIP_SYSCLK_INIT** don't set the value.
	- Select **Project** > **Properties** > **C/C++ Build** > **MCU Setting** 
		- Remove **RAM4** NCACHE_REGION
		- Modify **RAM3** SRAM_OC into NCACHE_REGION
	- Select **Project** > **Properties** > **C/C++ Build** > **Settings** > **MCU Linker** > **Managed Linker Script**
		- Modify `Stack` -> `SRAM_DTC` -> `0x200`
		- Modify `Heap` -> `SRAM_DTC` -> `0x800`
		- Add `*(CodeQuickAccess)` -> `SRAM_ITC` -> `.data`
		- Add `*(DataQuickAccess)` -> `SRAM_DTC` -> `.data`
		- Add `*(NonCacheable.init)` -> `NCACHE_REGION` -> `.data`
		- Add `*(NonCacheable)` -> `NCACHE_REGION` -> `.bss`
		- Add `*(BOARD_SDRAM)` -> `BOARD_SDRAM` -> `.bss` 
	- Select **Project** > **Properties** > **C/C++ Build** > **Settings** > **MCU C Compiler** > **Miscellaneous** > **Library Header** > **NewlibNano (Auto)**
	- Select **Project** > **Properties** > **C/C++ Build** > **Settings** > **MCU Assembler** > **Architeture & Headers** > **Library Header** > **NewlibNano (Auto)**
	- Select **Project** > **Properties** > **C/C++ Build** > **Settings** > **MCU Linker** > **Managed Linker Script** > **Library** > **NewlibNano (nohost)**
	- Select **Project** > **Properties** > **C/C++ Build** > **Settings** > **MCU C Compiler** > **Optimization** > **Optimization Level** > **Optimize most (-O3)**