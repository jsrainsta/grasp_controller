#include "pwm_servos.h"
#include <string.h>

void pwm_servo_duty_set(PWMServoHandleTypeDef* handle, int target_duty, uint32_t time)
{
	target_duty = target_duty > MAX_DUTY ? MAX_DUTY : (target_duty < MIN_DUTY ? MIN_DUTY : target_duty);
	time = time < MIN_RUNNING_TIME ? MIN_RUNNING_TIME : (time > MAX_RUNNING_TIME ? MAX_RUNNING_TIME : time);

	handle->time = time;
	handle->duty_changed_state = true;	
	handle->target_duty = target_duty;
	handle->last_target_duty = handle->target_duty;
}

void pwm_servo_angle_set(PWMServoHandleTypeDef* handle, float target_angle, uint32_t time)
{
	target_angle = target_angle > MAX_ANGLE ? MAX_ANGLE : (target_angle < MIN_ANGLE ? MIN_ANGLE : target_angle);
	time = time < MIN_RUNNING_TIME ? MIN_RUNNING_TIME : (time > MAX_RUNNING_TIME ? MAX_RUNNING_TIME : time);
	
	handle->time = time;
	handle->duty_changed_state = true;	
	handle->target_angle = target_angle;
	handle->target_duty = 500 + (int)(target_angle * 2000 / MAX_ANGLE);
}

void pwm_servo_offset_set(PWMServoHandleTypeDef* handle, int offset)
{
	offset = offset < MIN_OFFSET_DUTY ? MIN_OFFSET_DUTY : (offset > MAX_OFFSET_DUTY ? MAX_OFFSET_DUTY : offset);
	handle->offset = offset;
}


void pwm_servo_handler(PWMServoHandleTypeDef* handle)
{
	if(handle->duty_changed_state)
	{
		handle->duty_changed_state = false;
		handle->inc_times = handle->time / MIN_RUNNING_TIME;
		handle->duty_inc = handle->target_duty > handle->current_duty ? 		\
						   (float)(-(handle->target_duty - handle->current_duty)) :	\
						   (float)(handle->current_duty - handle->target_duty);
		handle->duty_inc /= (float)handle->inc_times;
		handle->running_state = true;
	}

	if (handle->running_state)
	{
		 --handle->inc_times;
		if (handle->inc_times == 0)
		{
			handle->current_duty = handle->target_duty;
			handle->running_state = false;
		}
		else
		{
			handle->current_duty = handle->target_duty + (int)(handle->duty_inc * handle->inc_times);
		}
	}
	handle->write_duty = handle->current_duty + handle->offset;
}

