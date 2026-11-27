#ifndef __OLED_H_
#define __OLED_H_

#include "u8g2.h"

#define OLED_ADDRESS    0x78
#define OLED_CMD        0x00
#define OLED_DATA       0x40
#define OLED_WRITE_CMD	0x00
#define OLED_WRITE_DATA	0x40
/**
 * @brief U8G2库初始化
 * 
 */
void u8g2_init(void);

/**
 * @brief 写入字符串到OLED
 * 
 * @param  x-OLED的x轴坐标
 * @param  y-OLED的y轴坐标
 * @param  str-字符串
 */
void draw_str(uint16_t x, uint16_t y, const char *str);

/**
 * @brief 拼接字符传和变量显示到OLED
 * 
 * @param  num-变量
 */
void u8g2_display_var(float num);
#endif
