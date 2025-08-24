/*
 * Copyright 2016-2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    test_sdram_lpspi_edma.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "fsl_debug_console.h"
#include "lvgl.h"
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "lcd_support.h"
#include "demos/lv_demos.h"

void SysTick_Handler(void)
{
    lv_tick_inc(1);
}
/*
 * @brief   Application entry point.
 */
int main(void) {
	/* Init board hardware. */
	BOARD_ConfigMPU();
	BOARD_InitBootPins();
	BOARD_InitBootClocks();
	BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
	/* Init FSL debug console. */
	BOARD_InitDebugConsole();
#endif
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
		//SDK_DelayAtLeastUs(3000, CLOCK_GetFreq(kCLOCK_CoreSysClk)); // 3 ms delay
	}
}
