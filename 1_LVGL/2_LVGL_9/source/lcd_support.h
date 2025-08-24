/*
 * lcd_support.h
 *
 *  Created on: Jul 20, 2025
 *      Author: yunus
 */

#ifndef LCD_SUPPORT_H_
#define LCD_SUPPORT_H_


#define LCD_WIDTH             	320
#define LCD_HEIGHT            	240
#define LCD_TOUCH_MINY 			400
#define LCD_TOUCH_MAXY 			3700
#define LCD_TOUCH_MINX 			300
#define LCD_TOUCH_MAXX 			3400
#define LCD_FB_BYTE_PER_PIXEL 	2
/* The virtual buffer for DBI panel, it should be ~1/10 screen size. */
#define LCD_VIRTUAL_BUF_SIZE (LCD_WIDTH * LCD_HEIGHT / 3)

#ifdef __cplusplus
extern "C" {
#endif

void lv_port_disp_init(void);
void lv_port_indev_init(void);

#if defined(__cplusplus)
}
#endif


#endif /* LCD_SUPPORT_H_ */
