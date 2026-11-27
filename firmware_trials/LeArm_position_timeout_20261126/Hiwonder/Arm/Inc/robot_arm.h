#ifndef __ROBOT_ARM_H__
#define __ROBOT_ARM_H__
/**
 * @file robot_arm.h
 * @author Min
 * @brief 机械臂控制实现
 * @version 1.0
 * @date 2025-02-22
 *
 * @copyright Copyright (c) 2025 Hiwonder
 *
 */
#include "stdint.h"
#include "stdbool.h"
#include "serial_servo.h"
#include "pwm_servos.h"

#define MAX_SERVOS_NUM					  	 			6

#define DEFAULT_X						 			15.0f
#define DEFAULT_Y						  			 0.0f
#define DEFAULT_Z						  			 2.0f

#define MAX_X										20.0f
#define MIN_X							 			10.0f
#define MAX_Y							 			10.0f
#define MIN_Y									   -10.0f
#define MAX_Z							 			25.0f
#define MIN_Z							  			 0.0f

#define DEFAULT_CLAW_OPEN_ANGLE			 			90.0f
#define DEFAULT_CLAW_ROTATION_ANGLE		 			90.0f

#define MIN_OPEN_ANGLE					  			 0.0f
#define MAX_OPEN_ANGLE					 			90.0f
#define MIN_ROTATION_ANGLE			 			   -90.0f
#define MAX_ROTATION_ANGLE				 			90.0f

#define SERIAL_ANGLE_FACTOR			   4.166666666666667f

/* 数据存放在Flash中的起始基地址 */
#define LOGO_BASE_ADDRESS          		   0UL  /* 该基地址用于存放标识LOGO */
#define SERVOS_OFFSET_BASE_ADDRESS 	 	  4096UL  /* 该基地址用于存放每个舵机的偏差 */
#define ACTION_FRAME_SUM_BASE_ADDRESS 	  8192UL  /* 该基地址用于存放每个动作组有多少动作 */
#define ACTION_GROUP_BASE_ADDRESS 		 12288UL	/* 该基地址用于存放下载的动作组文件 */

#define ACTION_FRAME_SIZE					21  /* 一个动作帧占21个字节 */ 
#define ACTION_GROUP_SIZE			 	  8192  /* 1个动作组留8KB内存空间 */
#define ACTION_GROUP_MAX_NUM               255  /* 默认最多存放255个动作组 */

/* 获得A的低八位 */
#define GET_LOW_BYTE(A) ((uint8_t)(A))
/* 获得A的高八位 */
#define GET_HIGH_BYTE(A) ((uint8_t)((A) >> 8))
/* 将高低八位合成为十六位 */
#define MERGE_HL(A, B) ((((uint16_t)(A)) << 8) | (uint8_t)(B))

typedef enum
{
	ACTION_FRAME_START = 0,
	ACTION_FRAME_RUNNING,
	ACTION_FRAME_IDLE
}ActionFrameStatusTypeDef;

typedef enum
{
	ACTION_GROUP_START = 0,
	ACTION_GROUP_RUNNING,
	ACTION_GROUP_END_PERIOD,
	ACTION_GROUP_IDLE
}ActionGroupStatusTypeDef;

typedef struct
{
	uint8_t 				 index;				/* 当前动作帧编号 */
	uint32_t 				 time;				/* 当前动作帧的运行时间 */
	uint8_t					 status;			/* 当前运行标志 */
}ActionFrameHandleTypeDef;

typedef struct
{
	/* 用于保存已有的动作组中各自的总动作帧数量 */
	uint8_t _sum[ACTION_GROUP_MAX_NUM];
	
	uint8_t 				 index;				/* 当前动作组的编号 */
	uint8_t					 sum;				/* 当前动作组的动作帧总数 */
	uint8_t					 running_times;		/* 当前动作组的运行次数 */
	uint8_t					 status;			/* 当前运行标志 */
	uint32_t				 time;				/* 当前动作组的运行时间 */

	ActionFrameHandleTypeDef frame;

}ActionGroupHandleTypeDef;

typedef struct
{
	uint8_t 				 cmd;
	int8_t		 servo_offset[6];
	ActionGroupHandleTypeDef action_group;

}RobotArmHandleTypeDef;

/**
 * @brief 机械臂初始化
 */
bool robot_arm_init(void);

/**
 * @brief 机械臂复位
 * 
 * @param  time 	运行时间
 */
void robot_arm_reset(uint32_t time);

/**
 * @brief 机械臂坐标控制接口
 * 
 * @param  target_x 	目标x轴坐标
 * @param  target_y		目标y轴坐标
 * @param  target_z		目标z轴坐标
 * @param  pitch		目标俯仰角
 * @param  min_pitch	最小俯仰角
 * @param  max_pitch	最大俯仰角
 * @param  time			运行时间
 * @return true			有解
 * 		   false		无解
 */
uint8_t robot_arm_coordinate_set(float target_x,
								 float target_y,
								 float target_z,
								 float pitch,
								 float min_pitch,
								 float max_pitch,
								 uint32_t time);

/**
 * @brief 机械臂关节偏差读取
 * 
 * @param  value 	所读取的机械臂关节偏差值保存地址
 */
void robot_arm_offset_read(int8_t* value);
								 
/**
 * @brief 机械臂关节偏差设置
 * 
 * @param  id 		关节id号
 * @param  value	偏差值
 */
void robot_arm_offset_set(uint8_t id, int8_t value);
								 
/**
 * @brief 机械臂关节偏差保存
 */
void robot_arm_offset_save(void);
							
/**
 * @brief 机械爪控制
 * 
 * @param  open_angle 		张开角度，范围为[0,90]
 * @param  open_angle_time	运行时间
 */			 
void robot_arm_claw_set(float open_angle, uint32_t open_angle_time);

/**
 * @brief 机械臂腕关节控制
 * 
 * @param  rotation_angle 			旋转角度，范围为[-90,90]
 * @param  rotation_angle_time		运行时间
 */
void robot_arm_roll_set(float rotation_angle, uint32_t rotation_angle_time);

/**
 * @brief 机械臂单关节控制
 * 
 * @param  id 			关节id号
 * @param  target_duty	目标脉宽，默认PWM舵机控制范围为[500,2500]
 *									   总线舵机控制范围为[0,1000]
 * @param  time			运行时间
 */
void robot_arm_knot_run(uint8_t id, int target_duty, uint32_t time);

/**
 * @brief 机械臂关节停止
 * 
 * @param  id 	关节id号
 */
void robot_arm_knot_stop(uint8_t id);

/**
 * @brief 机械臂关节运行标志
 * 
 * @param  id 				关节id号
 * @param  target_duty		目标脉宽
 * @return true  -运行完成
 *		   false -运行未完成
 */
bool robot_arm_knot_is_finish(uint8_t id, int target_duty);

/**
* @brief 获取机械臂关节当前脉宽
 * 
 * @param  id 			关节id号
 * @return 脉宽值
 */
int robot_arm_get_knot_current_duty(uint8_t id);			 
							
/**
 * @brief 动作组复位
 * @attention 每次运行完一次动作组都要调用此函数
 */	
void action_group_reset(void);

/**
 * @brief 动作组停止运行
 */	
void action_group_stop(void);

/**
 * @brief 擦除全部动作组
 */	
void action_group_erase(void);

/**
 * @brief 动作组运行
 * 
 * @param  action_group_index 	动作组编号
 * @param  repeat_times			重复运行次数
 */
bool action_group_run(uint8_t action_group_index, uint8_t repeat_times);
								 
/**
 * @brief 动作组数据写入接口
 * 
 * @param  self
 * @param  action_group_index 	动作组编号
 * @param  frame_num			该动作组的动作帧总数
 * @param  frame_index			写入的动作帧是第几帧，取值范围[0,255]
 * @param  pdata				帧数据指针
 * @param  size					帧数据长度
 */
void action_group_save(uint8_t action_group_number, 
					   uint8_t frame_num,
					   uint8_t frame_index,
					   uint8_t* pdata,
					   uint16_t size);

/**
 * @brief 重映射函数
 */
float map(float x, float in_min, float in_max, float out_min, float out_max);
#endif
