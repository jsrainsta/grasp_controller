#include "tim.h"
#include "pwm_servos.h"
#include "global.h"

PWMServoHandleTypeDef pwm_servos[USE_SERVO_NUM];

static void pwm_servo_object_init(PWMServoHandleTypeDef* handle, uint8_t id)
{
	memset(handle, 0, sizeof(PWMServoHandleTypeDef));
	handle->id = id;
	switch(id)
	{
		case 1:
			handle->current_duty = PWM_SERVO1_RESET_DUTY;
			handle->write_duty = PWM_SERVO1_RESET_DUTY;
			break;
		
		case 2:
			handle->current_duty = PWM_SERVO2_RESET_DUTY;
			handle->write_duty = PWM_SERVO2_RESET_DUTY;
			break;
		
		case 3:
			handle->current_duty = PWM_SERVO3_RESET_DUTY;
			handle->write_duty = PWM_SERVO3_RESET_DUTY;
			break;
		
		case 4:
			handle->current_duty = PWM_SERVO4_RESET_DUTY;
			handle->write_duty = PWM_SERVO4_RESET_DUTY;
			break;
		
		case 5:
			handle->current_duty = PWM_SERVO5_RESET_DUTY;
			handle->write_duty = PWM_SERVO5_RESET_DUTY;
			break;
		
		case 6:
			handle->current_duty = PWM_SERVO6_RESET_DUTY;
			handle->write_duty = PWM_SERVO6_RESET_DUTY;
			break;
	}
}

static void pwm_servo1_write_pin(uint8_t new_state)
{
	HAL_GPIO_WritePin(PWM_SERVO1_GPIO_Port, PWM_SERVO1_Pin, (GPIO_PinState)new_state);
}

static void pwm_servo2_write_pin(uint8_t new_state)
{
	HAL_GPIO_WritePin(PWM_SERVO2_GPIO_Port, PWM_SERVO2_Pin, (GPIO_PinState)new_state);
}

static void pwm_servo3_write_pin(uint8_t new_state)
{
	HAL_GPIO_WritePin(PWM_SERVO3_GPIO_Port, PWM_SERVO3_Pin, (GPIO_PinState)new_state);
}

static void pwm_servo4_write_pin(uint8_t new_state)
{
	HAL_GPIO_WritePin(PWM_SERVO4_GPIO_Port, PWM_SERVO4_Pin, (GPIO_PinState)new_state);
}

static void pwm_servo5_write_pin(uint8_t new_state)
{
	HAL_GPIO_WritePin(PWM_SERVO5_GPIO_Port, PWM_SERVO5_Pin, (GPIO_PinState)new_state);
}

static void pwm_servo6_write_pin(uint8_t new_state)
{
	HAL_GPIO_WritePin(PWM_SERVO6_GPIO_Port, PWM_SERVO6_Pin, (GPIO_PinState)new_state);
}

void pwm_servos_init()
{
	for(uint8_t i = 0; i < USE_SERVO_NUM; i++)
	{
		pwm_servo_object_init(&pwm_servos[i], USE_SERVO_NUM - i);
	}

	pwm_servos[0].write_pin = pwm_servo6_write_pin;
	pwm_servos[1].write_pin = pwm_servo5_write_pin;
	pwm_servos[2].write_pin = pwm_servo4_write_pin;
	pwm_servos[3].write_pin = pwm_servo3_write_pin;
	pwm_servos[4].write_pin = pwm_servo2_write_pin;
	pwm_servos[5].write_pin = pwm_servo1_write_pin;
	
	
	HAL_TIM_Base_Start_IT(&htim3);
	__HAL_TIM_ENABLE_IT(&htim3, TIM_IT_CC1);
	__HAL_TIM_ENABLE_IT(&htim3, TIM_IT_CC2);
}

void pwm_servos_handler()
{
	for(uint8_t i = 0; i < USE_SERVO_NUM; i++)
	{
		pwm_servo_handler(&pwm_servos[i]);
	}
}
