#ifndef __TOUCH_H__
#define __TOUCH_H__

#include "stdint.h"

#define TOUCH_PORT	GPIOB
#define TOUCH_PIN	GPIO_PIN_13

/**
 * @brief 获取触摸状态
 * 
 * @return 1-触摸未按下，0-触摸按下
 */
uint8_t get_touch_state(void);

#endif
