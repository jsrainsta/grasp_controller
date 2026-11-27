#ifndef __ULTRASOUND_H_
#define __ULTRASOUND_H_

#include "stdint.h"
#include "stdbool.h"

#define ULTRASOUND_ADDRESS  			0x77

#define ULTRASOUND_DISTANCE_REG     	0x00
#define RGB_WORK_MODE_REG       		0x02
#define RGB_WORK_SIMPLE_MODE_REG	   	   0
#define RGB_WORK_BREATHING_MODE_REG    	   1
#define RGB_CONSTANT_INDEX_REG			   3
#define RGB_BREATHING_INDEX_REG			   9

typedef struct UltrasoundHandle UltrasoundHandleTypeDef;
struct UltrasoundHandle
{
	uint8_t left_rgb[3];
	uint8_t right_rgb[3];
	uint8_t rgb_mode;
	uint16_t distance;
	
	uint8_t transmit_status;
	uint8_t receive_status;
	uint8_t dev_addr;

	uint8_t (*write_data)(UltrasoundHandleTypeDef* self, uint8_t* pdata, uint16_t size);
	uint8_t (*read_data)(UltrasoundHandleTypeDef* self, uint8_t* pdata, uint16_t size);
};

/**
 * @brief 超声波测距传感器初始化
 */
void ultrasound_init(void);
	
/**
 * @brief 
 * 
 * @param  self				超声波对象指针
 * @return  distance
 */
uint16_t get_ultrasound_distance(void);

/**
 * @brief RGB灯颜色设置
 * 
 * @param  self				超声波对象指针
 * @param  mode				0为固定颜色模式, 1为呼吸灯模式
 * @param  left 			指向保存颜色参数的数组地址
 * @param  right			指向保存颜色参数的数组地址
 */
void set_ultrasound_color(uint8_t mode, uint8_t* left_rgb, uint8_t* right_rgb);

#endif
