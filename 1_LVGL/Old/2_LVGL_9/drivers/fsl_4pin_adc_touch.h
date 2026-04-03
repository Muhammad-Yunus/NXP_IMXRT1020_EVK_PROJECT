#ifndef _FSL_4PIN_ADC_TOUCH_H_
#define _FSL_4PIN_ADC_TOUCH_H_


// Touch point structure
typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t z;
} touch_point_t;

// Function prototypes
void TouchScreen_Init(void);
touch_point_t TouchScreen_GetPoint(void);


#endif // _FSL_4PIN_ADC_TOUCH_H_
