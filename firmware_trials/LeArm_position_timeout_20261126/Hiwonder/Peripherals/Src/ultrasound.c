#include "ultrasound.h"
#include "i2c.h"
#include "string.h"

UltrasoundHandleTypeDef ultrasound;

static uint8_t write_data(UltrasoundHandleTypeDef* self, uint8_t* pdata, uint16_t size)
{
	self->transmit_status = (uint8_t)HAL_I2C_Master_Transmit(&hi2c1, self->dev_addr << 1, pdata, size, 0xfff);
	return self->transmit_status;
}


static uint8_t read_data(UltrasoundHandleTypeDef* self, uint8_t* pdata, uint16_t size)
{	
	/* 使用HAL_I2C_Master_Receive_DMA这个函数需要把i2c的事件中断勾上 */
	self->receive_status = (uint8_t)HAL_I2C_Master_Receive_DMA(&hi2c1, self->dev_addr << 1, pdata, size);
	return self->receive_status;	
}


static bool write_to_device(UltrasoundHandleTypeDef* self, uint8_t reg, uint8_t* pdata, uint16_t size)
{
	uint8_t trans_data[size + 1];
	
	trans_data[0] = reg;
	
	for (uint16_t i = 0; i < size; i++)
	{
		trans_data[1 + i] = pdata[i];
	}
	
	if(write_data(self, trans_data, sizeof(trans_data)) != 0)
	{
		return false;
	}
	
	return true;
}

static bool receive_from_device(UltrasoundHandleTypeDef* self, uint8_t reg, uint8_t* pdata, uint16_t size)
{
	uint8_t set_reg= reg;	
	
	if(write_data(self, &set_reg, 1) != 0)
	{
		return false;
	}
	
	if(read_data(self, pdata, size) != 0)
	{
		return false;
	}
	
	return true;
}

void ultrasound_init()
{
	memset(&ultrasound, 0, sizeof(UltrasoundHandleTypeDef));
	ultrasound.dev_addr = ULTRASOUND_ADDRESS;
	ultrasound.write_data = write_data;
	ultrasound.read_data = read_data;
}


uint16_t get_ultrasound_distance()
{
	static uint8_t distance[2];
	uint32_t sum = 0;
	const static uint8_t filter_num = 3;
	static uint16_t filter_buf[filter_num + 1];
	
	receive_from_device(&ultrasound, ULTRASOUND_DISTANCE_REG, distance, sizeof(distance));
	while(HAL_I2C_STATE_READY != HAL_I2C_GetState(&hi2c1));
	filter_buf[filter_num] = (uint16_t)distance[1] << 8 | distance[0];
	/* 递推平均滤波*/
	for (uint8_t i = 0; i < filter_num; i++)
	{
		filter_buf[i] = filter_buf[i + 1];
		sum += filter_buf[i];
	}
	
	ultrasound.distance = (uint16_t)sum / filter_num;
	return ultrasound.distance;
}

void set_ultrasound_color(uint8_t mode, uint8_t* left_rgb, uint8_t* right_rgb)
{
	uint8_t set_reg;
	uint8_t rgb[6];
	
	ultrasound.rgb_mode = mode;
	
	for (uint8_t i = 0; i < 3; i++)
	{
		rgb[i] = left_rgb[i];
		rgb[i + 3] = right_rgb[i];
		ultrasound.left_rgb[i] = rgb[i];
		ultrasound.right_rgb[i] = rgb[i + 3];
	}
	
	switch(mode)
	{
		case 0:
			set_reg = RGB_WORK_SIMPLE_MODE_REG;
			write_to_device(&ultrasound, RGB_WORK_MODE_REG, &set_reg, sizeof(set_reg));
			write_to_device(&ultrasound, RGB_CONSTANT_INDEX_REG, rgb, sizeof(rgb));
			break;
		
		case 1:
			set_reg = RGB_WORK_BREATHING_MODE_REG;
			write_to_device(&ultrasound, RGB_WORK_MODE_REG, &set_reg, sizeof(set_reg));
			write_to_device(&ultrasound, RGB_BREATHING_INDEX_REG, rgb, sizeof(rgb));
			break;
		
		default:
			break;
	}
}
