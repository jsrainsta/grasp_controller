#include "avoid_obstacles.h"
#include "stm32f1xx_hal.h"

uint8_t get_avoid_obstacle1_state()
{
	return HAL_GPIO_ReadPin(AVOID_OBSTACLE1_PORT, AVOID_OBSTACLE1_PIN);
}

uint8_t get_avoid_obstacle2_state()
{
	return HAL_GPIO_ReadPin(AVOID_OBSTACLE2_PORT, AVOID_OBSTACLE2_PIN);
}
