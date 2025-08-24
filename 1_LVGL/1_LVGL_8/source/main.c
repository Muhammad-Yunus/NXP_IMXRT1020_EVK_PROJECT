/*
 * Copyright 2016-2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    text_lvgl_guider_control_led.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "lvgl.h"
#include "lcd_support.h"
#include "gui_guider.h"
#include "events_init.h"
#include "fsl_gpio.h"
#include "fsl_iomuxc.h"
#include "demos/lv_demos.h"

#define LED_GPIO GPIO1
#define LED_GPIO_PORT 1U
#define LED_GPIO_PIN 5U

lv_ui guider_ui;

void SysTick_Handler(void)
{
    lv_tick_inc(1);
}

void init_led_gpio(void);
void toggle_led(int state);

void lvgl_test_screen(void);
static void anim_color_cb(void * obj, int32_t v);
void create_rect_with_anim(void);

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

    init_led_gpio();

	lv_init();
	lv_port_disp_init();
	lv_port_indev_init();

#if LV_USE_LOG
	lv_log_register_print_cb(print_cb);
#endif

	// ------ GUI GUIDER ------
    //setup_ui(&guider_ui); // this creates your screens
    //events_init(&guider_ui); // setup events and bindings

	// ------ LVGL 8.3.10 ------
	//LV_LOG("LVGL demo started\r\n");
	/* enable the flag also in lv_conf.h */
	//lv_demo_widgets();
	//lv_demo_stress();
	lv_demo_music();

	// ------ LVGL TEST ------
	//create_rect_with_anim();

	for (;;)
	{
		lv_timer_handler();
		//SDK_DelayAtLeastUs(2000, CLOCK_GetFreq(kCLOCK_CoreSysClk)); // 2 ms delay
	}
}


/****************************LVGL TEST****************************/
void lvgl_test_screen(void)
{
    /* Create a button on the active screen */
    lv_obj_t * btn = lv_btn_create(lv_scr_act());
    lv_obj_align(btn, LV_ALIGN_CENTER, 0, 0);  // Center of screen
    lv_obj_set_size(btn, 100, 50);             // 100x50 px

    /* Add a label on the button */
    lv_obj_t * label = lv_label_create(btn);
    lv_label_set_text(label, "Click me");
    lv_obj_center(label);
}

static void anim_color_cb(void * obj, int32_t v)
{
    // `v` will go from 0 (red) to 255 (blue)
    lv_color_t c = lv_color_mix(lv_palette_main(LV_PALETTE_BLUE),
                                lv_palette_main(LV_PALETTE_RED),
                                v); // v = 0 -> RED, v=255 -> BLUE
    lv_obj_set_style_bg_color((lv_obj_t *)obj, c, LV_PART_MAIN);
}

void create_rect_with_anim(void)
{
    // Create rectangle
    lv_obj_t * rect = lv_obj_create(lv_scr_act());
    lv_obj_set_size(rect, 100, 100);
    lv_obj_center(rect);

    // Remove default style if needed (optional)
    lv_obj_set_style_radius(rect, 0, LV_PART_MAIN);

    // Set initial color
    lv_obj_set_style_bg_color(rect, lv_palette_main(LV_PALETTE_RED), LV_PART_MAIN);

    // Create animation
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, rect);
    lv_anim_set_exec_cb(&a, anim_color_cb);
    lv_anim_set_time(&a, 500); // 500 ms
    lv_anim_set_values(&a, 0, 255); // From RED to BLUE
    lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
    lv_anim_set_playback_time(&a, 500); // Animate back to red

    lv_anim_start(&a);
}



/************************GPIO CONTROL : LED************************/
void init_led_gpio(void)
{
    gpio_pin_config_t led_config = {
        kGPIO_DigitalOutput,
        1, // initial logic level (1 = ON)
    };

    // Enable clock if not already done (usually handled by SDK)
    CLOCK_EnableClock(kCLOCK_Iomuxc);

    // Set pin mux if needed (depends on board and SDK)
    IOMUXC_SetPinMux(
        IOMUXC_GPIO_AD_B0_05_GPIO1_IO05,  // Example for GPIO1_IO05 on RT1020 EVK
        0U);

    GPIO_PinInit(LED_GPIO, LED_GPIO_PIN, &led_config);
}

void toggle_led(int state)
{
    GPIO_PinWrite(GPIO1, 5U, state); // example for RT1020
}
