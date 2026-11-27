#ifndef __AVOID_OBSTACLES_H__
#define __AVOID_OBSTACLES_H__

#include "stdint.h"

#define AVOID_OBSTACLE1_PORT	GPIOA
#define AVOID_OBSTACLE1_PIN		GPIO_PIN_4

#define AVOID_OBSTACLE2_PORT	GPIOB
#define AVOID_OBSTACLE2_PIN		GPIO_PIN_13


/**
 * @brief 获取1号红外对管传感器当前状态
 * 
 * @return 0-识别到障碍     1-没有识别到障碍 
 */
uint8_t get_avoid_obstacle1_state(void);

/**
 * @brief 获取2号红外对管传感器当前状态
 * 
 * @return 0-识别到障碍     1-没有识别到障碍 
 */

uint8_t get_avoid_obstacle2_state(void);

#endif
