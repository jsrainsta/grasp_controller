#ifndef __MOTOR_MODULE_H_
#define __MOTOR_MODULE_H_

#include "stdint.h"
#include "stdbool.h"

#define MOTOR_MODULE_ADDR			0x34
#define ADC_BAT_REG					0x00
#define MOTOR_TYPE_REG				0x14
#define MOTOR_ENCODER_POLARITY_REG 	0x15
#define MOTOR_SET_SPEED_REG			0x33
#define MOTOR_SET_PWM_REG			0x1F
#define MOTOR_GET_ENCODER_REG		0x3C

#define MOTOR_TYPE_WITHOUT_ENCODER			0
#define MOTOR_TYPE_TT						1
#define MOTOR_TYPE_N20						2
#define MOTOR_TYPE_JGB37_520_12V_110RPM		3

typedef struct MotorModuleHandle MotorModuleHandleTypeDef;
struct MotorModuleHandle
{
	uint8_t motor_type;
	int8_t set_pwm[4];
	int8_t set_speed[4];
	uint8_t transmit_status;
	uint8_t receive_status;

	uint16_t dev_addr;
	uint16_t module_voltage;	
	
	uint8_t (*write_data)(MotorModuleHandleTypeDef* self, uint8_t* pdata, uint16_t size);
	uint8_t (*read_data)(MotorModuleHandleTypeDef* self, uint8_t* pdata, uint16_t size);
};

/**
 * @brief 电机驱动模块初始化
 * 
 * @param  motor_type-电机类型
 * 		   MOTOR_TYPE_WITHOUT_ENCODER
 * 		   MOTOR_TYPE_TT
 * 		   MOTOR_TYPE_N20
 * 		   MOTOR_TYPE_JGB37_520_12V_110RPM
 */
void motor_init(uint8_t motor_type);

/**
 * @brief 获取电机驱动模块电压值
 * 
 * @param  bat-电压值 单位：mv
 * @return true 
 * @return false 
 */
bool get_motor_module_voltage(uint16_t* bat);

/**
 * @brief 获取电机编码器计数值
 * 
 * @param  4个电机编码器计数值保存指针
 * @return true 
 * @return false 
 */
bool get_motor_encoder(int32_t* encoder_val);

/**
 * @brief 设置电机开环还是闭环运行
 * 
 * @param  type
 * @return true 
 * @return false 
 */
bool set_motor_type(uint8_t type);

/**
 * @brief 设置电机极性
 * 
 * @param  polarity 取值为0或1
 * @return true 
 * @return false 
 */
bool set_motor_polarity(uint8_t polarity);

/**
 * @brief 设置4个电机的PWM值
 * 
 * @param  pwm范围[-100,100] 数组大小为4
 * @return true 
 * @return false 
 */
bool set_motor_pwm(int8_t* pwm);

/**
 * @brief 设置4个电机速度
 * 
 * @param  speed范围[-100,100]
 * @return true 
 * @return false 
 * @attention 使用该函数前必须先将电机设置为闭环模式
 */
bool set_motor_speed(int8_t* speed);

#endif



