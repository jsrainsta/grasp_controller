#include "wonder_mv.h"
#include "string.h"
#include "global.h"
#include "i2c.h"

/*
 * [0]:id
 * [1]:x(low 8bit) 
 * [2]:x(high 8bit) 
 * [3]:y(low 8bit) 
 * [4]:y(high 8bit) 
 * [5]:w(low 8bit) 
 * [6]:w(high 8bit) 
 * [7]:h(low 8bit) 
 * [8]:h(high 8bit)
 */
 
WonderMVHandleTypeDef wonder_mv;
	
static uint8_t write_data(WonderMVHandleTypeDef* self, uint8_t* pdata, uint16_t size)
{
	self->transmit_status = (uint8_t)HAL_I2C_Master_Transmit(&hi2c1, self->dev_addr << 1, pdata, size, 0xfff);
	return self->transmit_status;
}

static uint8_t read_data(WonderMVHandleTypeDef* self, uint8_t* pdata, uint16_t size)
{	
	/* 使用HAL_I2C_Master_Receive_DMA这个函数需要把i2c的事件中断勾上 */
	self->receive_status = (uint8_t)HAL_I2C_Master_Receive_DMA(&hi2c1, self->dev_addr << 1, pdata, size);
	return self->receive_status;	
}


static bool write_to_device(WonderMVHandleTypeDef* self, uint8_t reg, uint8_t* pdata, uint16_t size)
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

static bool receive_from_device(WonderMVHandleTypeDef* self, uint8_t reg, uint8_t* pdata, uint16_t size)
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

void wonder_mv_init()
{
	memset(&wonder_mv, 0, sizeof(WonderMVHandleTypeDef));
	wonder_mv.dev_addr = WONDERMV_ADDR;
	wonder_mv.read_data = read_data;
	wonder_mv.write_data = write_data;
}

bool wonder_mv_color_recognition(RecognitionHanleTypeDef* color)
{
	if(receive_from_device(&wonder_mv, COLOR_REG, wonder_mv.results, sizeof(wonder_mv.results)))
	{
		while(HAL_I2C_STATE_READY != HAL_I2C_GetState(&hi2c1));
		color->id = wonder_mv.results[0];
		color->position.x = BYTE_TO_HW(wonder_mv.results[2], wonder_mv.results[1]);
		color->position.y = BYTE_TO_HW(wonder_mv.results[4], wonder_mv.results[3]);
		color->position.w = BYTE_TO_HW(wonder_mv.results[6], wonder_mv.results[5]);
		color->position.h = BYTE_TO_HW(wonder_mv.results[8], wonder_mv.results[7]);
		return true;
	}

	return false;
}


bool wonder_mv_face_detection(RecognitionHanleTypeDef* face)
{
	if(receive_from_device(&wonder_mv, FACE_REG, wonder_mv.results, sizeof(wonder_mv.results)))
	{
		while(HAL_I2C_STATE_READY != HAL_I2C_GetState(&hi2c1));
		face->id = wonder_mv.results[0];
		face->position.x = BYTE_TO_HW(wonder_mv.results[2], wonder_mv.results[1]);
		face->position.y = BYTE_TO_HW(wonder_mv.results[4], wonder_mv.results[3]);
		face->position.w = BYTE_TO_HW(wonder_mv.results[6], wonder_mv.results[5]);
		face->position.h = BYTE_TO_HW(wonder_mv.results[8], wonder_mv.results[7]);
		return true;
	}

	return false;
}

bool wonder_mv_tag_detection(RecognitionHanleTypeDef* tag)
{
	if(receive_from_device(&wonder_mv, TAG_REG, wonder_mv.results, sizeof(wonder_mv.results)))
	{
		while(HAL_I2C_STATE_READY != HAL_I2C_GetState(&hi2c1));
		tag->id = wonder_mv.results[0];
		tag->position.x = BYTE_TO_HW(wonder_mv.results[2], wonder_mv.results[1]);
		tag->position.y = BYTE_TO_HW(wonder_mv.results[4], wonder_mv.results[3]);
		tag->position.w = BYTE_TO_HW(wonder_mv.results[6], wonder_mv.results[5]);
		tag->position.h = BYTE_TO_HW(wonder_mv.results[8], wonder_mv.results[7]);
		return true;
	}
	return false;
}

bool wonder_mv_object_detection(RecognitionHanleTypeDef* obj)
{
	if(receive_from_device(&wonder_mv, OBJECT_REG, wonder_mv.results, sizeof(wonder_mv.results)))
	{
		while(HAL_I2C_STATE_READY != HAL_I2C_GetState(&hi2c1));
		obj->id = wonder_mv.results[0];
		obj->position.x = BYTE_TO_HW(wonder_mv.results[2], wonder_mv.results[1]);
		obj->position.y = BYTE_TO_HW(wonder_mv.results[4], wonder_mv.results[3]);
		obj->position.w = BYTE_TO_HW(wonder_mv.results[6], wonder_mv.results[5]);
		obj->position.h = BYTE_TO_HW(wonder_mv.results[8], wonder_mv.results[7]);
		return true;
	}
	return false;
}
