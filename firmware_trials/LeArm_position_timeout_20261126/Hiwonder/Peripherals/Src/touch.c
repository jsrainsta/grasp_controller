#include "touch.h"
#include "stm32f1xx_hal.h"

uint8_t get_touch_state()
{
	return HAL_GPIO_ReadPin(TOUCH_PORT, TOUCH_PIN);
}
