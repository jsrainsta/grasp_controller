#ifndef __PWM_SERVOS_H__
#define __PWM_SERVOS_H__

/**
 * @file pwm_servo.h
 * @author Min
 * @brief PWM舵机驱动
 * @version 1.0
 * @date 2024-12-13
 *
 * @copyright Copyright (c) 2024 Hiwonder
 *
 */

#include "stdint.h"
#include "stdbool.h"

#define USE_SERVO_NUM				   6

#define MAX_DUTY					2500
#define MIDDLE_DUTY					1500
#define MIN_DUTY					 500
#define MAX_ANGLE				  180.0f
#define MIDDLE_ANGLE			   90.0f
#define MIN_ANGLE					0.0f
#define MAX_RUNNING_TIME		   30000
#define MIN_RUNNING_TIME			  20
#define MAX_OFFSET_DUTY				 100
#define MIN_OFFSET_DUTY				-100



typedef struct {
	uint8_t			id;
	int				offset;
	int 			target_duty;
	int 			last_target_duty;
	int 			current_duty;
	int 			write_duty;				/* 实际写入的脉宽 */
	float			target_angle;

	/* 以下变量用于速度控制 */
	bool 			running_state;
	bool 			duty_changed_state;				
	int 			inc_times;				/* 当前位置到达目标位置需要递增的次数 */
	float 			duty_inc;					/* 每次递增时间的脉宽增量 */
	uint32_t 		time;						/* 运行时间 单位：ms */
		
    void (*write_pin)(uint8_t new_state); 
}PWMServoHandleTypeDef;

extern PWMServoHandleTypeDef pwm_servos[USE_SERVO_NUM];

/**
 * @brief 舵机对象初始化
 */
void pwm_servos_init(void);

/**
 * @brief 舵机位置控制(直接控制脉宽的方式实现)
 * 
 * @param  handle		需要控制的舵机对象句柄指针
 * @param  target_duty	目标脉宽
 * @param  time			运行时间
 */
void pwm_servo_duty_set(PWMServoHandleTypeDef* handle, int target_duty, uint32_t time);

/**
 * @brief 舵机位置控制(控制角度的方式实现)
 * 
 * @param  handle		需要控制的舵机对象句柄指针
 * @param  target_angle	目标角度
 * @param  time			运行时间
 */
void pwm_servo_angle_set(PWMServoHandleTypeDef* handle, float target_angle, uint32_t time);

/**
 * @brief 舵机偏差设置
 * 
 * @param  handle		需要控制的舵机对象句柄指针
 * @param  offset		设置的偏差值
 */
void pwm_servo_offset_set(PWMServoHandleTypeDef* handle, int offset); 

/**
 * @brief 舵机输出脉宽控制
 * 
 * @param  handle	需要控制的舵机对象句柄指针
 * @attention 		需要间隔20ms调用一次
 */
void pwm_servo_handler(PWMServoHandleTypeDef* handle);
void pwm_servos_handler(void);

extern PWMServoHandleTypeDef pwm_servos[USE_SERVO_NUM];
#endif
