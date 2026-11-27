// <<< Use Configuration Wizard in Context Menu >>>
#ifndef __GLOBAL_H__
#define __GLOBAL_H__

#include "main.h"
#include <string.h>
#include "led.h"
#include "pwm_servos.h"

/* 宏函数 获得A的低八位 */
#define GET_LOW_BYTE(A) ((uint8_t)(A))
/* 宏函数 获得A的高八位 */
#define GET_HIGH_BYTE(A) ((uint8_t)((A) >> 8))
/* 宏函数 将高地八位合成为十六位 */
#define BYTE_TO_HW(A, B) ((((uint16_t)(A)) << 8) | (uint8_t)(B))

/* 单位：ms */
#define LED_HANDLER_PERIOD       	LED_TIMER_PERIOD
#define PWM_SERVOS_HANDLER_PERIOD	MIN_RUNNING_TIME
#define BUTTON_HANDLER_PERIOD    	20
#define BUZZER_HANDLER_PERIOD       25
#define SOFTWARE_VERSION			1

/* 对应坐标(15,0,2) */
#define PWM_SERVO1_RESET_DUTY 			   			  770
#define PWM_SERVO2_RESET_DUTY 			  			 1500
#define PWM_SERVO3_RESET_DUTY 			   			  640
#define PWM_SERVO4_RESET_DUTY 			   			  511
#define PWM_SERVO5_RESET_DUTY 			  			 1255
#define PWM_SERVO6_RESET_DUTY 			  			 1500
#define PWM_SERVO_PS2_MAX_DUTY						 2500
#define PWM_SERVO_PS2_MIN_DUTY						  500

#define SERIAL_SERVO1_RESET_DUTY		   			  226
#define SERIAL_SERVO2_RESET_DUTY		   			  500 
#define SERIAL_SERVO3_RESET_DUTY		   			  177 
#define SERIAL_SERVO4_RESET_DUTY		   			  129 
#define SERIAL_SERVO5_RESET_DUTY		   			  408 
#define SERIAL_SERVO6_RESET_DUTY		   			  500 
#define SERIAL_SERVO_PS2_MAX_DUTY					  875
#define SERIAL_SERVO_PS2_MIN_DUTY					  125

// <o>Chassis Select
//  <i>Select chassis type
//  <0=> None
//  <1=> Mecanum Chassis
//  <2=> Differential Chassis
#define CHASSIS_SELECT				0
#if (CHASSIS_SELECT == 1)
	#define MECANUM_CHASSIS
#elif (CHASSIS_SELECT == 2)
	#define DIFFERENTIAL_CHASSIS
#endif

extern uint8_t arm_select;
extern LEDHandleTypeDef led;
extern LEDHandleTypeDef led1;

#endif
// <<< end of configuration section >>>
