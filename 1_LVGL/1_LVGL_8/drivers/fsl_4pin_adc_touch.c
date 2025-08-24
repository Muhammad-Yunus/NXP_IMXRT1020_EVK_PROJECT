#include "fsl_gpio.h"
#include "fsl_adc.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "fsl_iomuxc.h"
#include "fsl_4pin_adc_touch.h"
#include <stdlib.h>

#define THRESHOLD 			250
#define YM_ADC_A0 			26U //GPIO_AD_B1_10 (A0)
#define XM_ADC_A1 			27U //GPIO_AD_B1_11 (A1)
#define YP_ADC_A2 			28U //GPIO_AD_B1_12 (A2)
#define XP_ADC_A3 			29U //GPIO_AD_B1_13 (A3)
#define YM_ADC_A0_IOMUX 	IOMUXC_GPIO_AD_B1_10_GPIO1_IO26
#define XM_ADC_A1_IOMUX 	IOMUXC_GPIO_AD_B1_11_GPIO1_IO27
#define YP_ADC_A2_IOMUX 	IOMUXC_GPIO_AD_B1_12_GPIO1_IO28
#define XP_ADC_A3_IOMUX 	IOMUXC_GPIO_AD_B1_13_GPIO1_IO29
#define YM_ADC_A0_CH		BOARD_INITPINS_LCD_TOUCH_YM_ADC_A0_CHANNEL
#define XM_ADC_A1_CH		BOARD_INITPINS_LCD_TOUCH_XM_ADC_A1_CHANNEL
#define YP_ADC_A2_CH 		BOARD_INITPINS_LCD_TOUCH_YP_ADC_A2_CHANNEL
#define XP_ADC_A3_CH 		BOARD_INITPINS_LCD_TOUCH_XP_ADC_A3_CHANNEL
#define ADC_BASE 			ADC1
#define ADC_CH_GROUP 		0U
#define HIGH 				1U
#define LOW 				0U
#define ADC_MODE 			0U
#define GPIO_MODE			1U
#define AVERAGETIME 		10U
#define NUMSAMPLES			4U
#define COMP				20U
#define INVALID_XY			9999u

#define GET_MUX_REG_ADDR(_mux, _mode, _input, _daisy, _cfg)  (_mux)
#define pinMode(muxMacro, mode)  \
    do { \
        if ((mode) == 1) IOMUXC_SetPinMux(muxMacro, mode); \
        else IOMUXC_SetPinToAnalog(GET_MUX_REG_ADDR(muxMacro), mode); \
    } while (0)

void IOMUXC_SetPinToAnalog(uint32_t muxReg, uint32_t mode)
{
    *(volatile uint32_t *)muxReg = mode;
}

void TouchScreen_Init(void)
{
    adc_config_t adcConfig;
    ADC_GetDefaultConfig(&adcConfig);
    ADC_Init(ADC_BASE, &adcConfig);
#if !(defined(FSL_FEATURE_ADC_SUPPORT_HARDWARE_TRIGGER_REMOVE) && FSL_FEATURE_ADC_SUPPORT_HARDWARE_TRIGGER_REMOVE)
    ADC_EnableHardwareTrigger(ADC_BASE, false);
#endif
    /* Do auto hardware calibration. */
    if (kStatus_Success != ADC_DoAutoCalibration(ADC_BASE))
    {
    	PRINTF("ADC_DoAutoCalibration() Failed.\r\n");
    }
}

void digitalWrite(GPIO_Type *base, uint32_t pin, bool level)
{
    gpio_pin_config_t config = {kGPIO_DigitalOutput, level};
    GPIO_PinInit(base, pin, &config);
}
void digitalRead(GPIO_Type *base, uint32_t pin)
{
    gpio_pin_config_t config = {kGPIO_DigitalInput};
    GPIO_PinInit(base, pin, &config);
}

uint16_t analogRead(uint32_t channel)
{
    adc_channel_config_t config;
    config.channelNumber = channel;
    config.enableInterruptOnConversionCompleted = false;

    ADC_SetChannelConfig(ADC_BASE, ADC_CH_GROUP, &config);
    while (0U == ADC_GetChannelStatusFlags(ADC_BASE, ADC_CH_GROUP))
    {
    }
    return ADC_GetChannelConversionValue(ADC_BASE, ADC_CH_GROUP);
}
uint16_t analogReadAverage(uint32_t channel)
{
    uint32_t sum = 0;
    for (int i = 0; i < AVERAGETIME; i++) {
    	sum += analogRead(channel);
    }
    return (uint16_t)(sum / AVERAGETIME);
}

uint16_t Touch_ReadX()
{
    // === Prepare for X-read ===
    // YM = GND (LOW), YP = VCC (HIGH)
	pinMode(YP_ADC_A2_IOMUX, GPIO_MODE);
    digitalWrite(GPIO1, YP_ADC_A2, HIGH);

    pinMode(YM_ADC_A0_IOMUX, GPIO_MODE);
    digitalWrite(GPIO1, YM_ADC_A0, LOW);

    pinMode(XM_ADC_A1_IOMUX, GPIO_MODE);
    digitalRead(GPIO1, XM_ADC_A1);

    pinMode(XP_ADC_A3_IOMUX, ADC_MODE);

//    uint16_t samples[NUMSAMPLES];
//    for (int i = 0; i < NUMSAMPLES; i++) {
//    	samples[i] = analogReadAverage(XP_ADC_A3_CH);
//    }
//    int icomp = samples[0] > samples[1] ? samples[0] - samples[1] : samples[1] - samples[0];
//    return icomp > COMP ? INVALID_XY : samples[NUMSAMPLES - 1];
    return analogReadAverage(XP_ADC_A3_CH);
}

uint16_t Touch_ReadY()
{
    // === Prepare for Y-read ===
    // XM = VCC (HIGH), XP = GND (LOW)
	pinMode(XP_ADC_A3_IOMUX, GPIO_MODE);
    digitalWrite(GPIO1, XP_ADC_A3, LOW);

    pinMode(XM_ADC_A1_IOMUX, GPIO_MODE);
    digitalWrite(GPIO1, XM_ADC_A1, HIGH);

    pinMode(YM_ADC_A0_IOMUX, GPIO_MODE);
    digitalRead(GPIO1, YM_ADC_A0);

    pinMode(YP_ADC_A2_IOMUX, ADC_MODE);

//    uint16_t samples[NUMSAMPLES];
//    for (int i = 0; i < NUMSAMPLES; i++) {
//    	samples[i] = analogReadAverage(YP_ADC_A2_CH);
//    }
//    int icomp = samples[0] > samples[1] ? samples[0] - samples[1] : samples[1] - samples[0];
//    return icomp > COMP ? INVALID_XY : samples[NUMSAMPLES - 1];
    return analogReadAverage(YP_ADC_A2_CH);
}

uint16_t Touch_ReadZ()
{
    // === Prepare for Z-read ===
    // YM = VCC (HIGH), XP = GND (LOW)
    pinMode(YM_ADC_A0_IOMUX, GPIO_MODE);
    digitalWrite(GPIO1, YM_ADC_A0, HIGH);

	pinMode(XP_ADC_A3_IOMUX, GPIO_MODE);
    digitalWrite(GPIO1, XP_ADC_A3, LOW);

    pinMode(XM_ADC_A1_IOMUX, ADC_MODE);
    pinMode(XP_ADC_A3_IOMUX, ADC_MODE);
	return analogRead(XM_ADC_A1_CH) + analogRead(XP_ADC_A3_CH);
}
touch_point_t TouchScreen_GetPoint(void)
{
    touch_point_t p = {0, 0, 0};
    p.y = Touch_ReadY();
    p.x = Touch_ReadX();
    p.z = Touch_ReadZ();
    if (p.z <= THRESHOLD || p.y == INVALID_XY || p.x == INVALID_XY) {
    	p.x = 0;
    	p.y = 0;
        p.z = 0;
    }
    return p;
}
