#include "wonder_echo.h"
#include "i2c.h"
#include "string.h"

WonderEchoHandleTypeDef wonder_echo;

static uint8_t write_data(WonderEchoHandleTypeDef* self, uint8_t* pdata, uint16_t size)
{
	self->transmit_status = (uint8_t)HAL_I2C_Master_Transmit(&hi2c1, self->dev_addr << 1, pdata, size, 0xfff);
	return self->transmit_status;
}


static uint8_t read_data(WonderEchoHandleTypeDef* self, uint8_t* pdata, uint16_t size)
{	
	/* 使用HAL_I2C_Master_Receive_DMA这个函数需要把i2c的事件中断勾上 */
	self->receive_status = (uint8_t)HAL_I2C_Master_Receive_DMA(&hi2c1, self->dev_addr << 1, pdata, size);
	return self->receive_status;	
}


static bool write_to_device(WonderEchoHandleTypeDef* self, uint8_t reg, uint8_t* pdata, uint16_t size)
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

static bool receive_from_device(WonderEchoHandleTypeDef* self, uint8_t reg, uint8_t* pdata, uint16_t size)
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

void wonder_echo_init()
{
	memset(&wonder_echo, 0, sizeof(wonder_echo));
	wonder_echo.write_data = write_data;
	wonder_echo.read_data = read_data;
	wonder_echo.dev_addr = WONDER_ECHO_ADDRESS;
}

int echo_recognition(void)
{
	static uint8_t result = 0;
	
	if(receive_from_device(&wonder_echo, ECHO_RESULT_REG, &result, sizeof(result)))
	{
		while(HAL_I2C_STATE_READY != HAL_I2C_GetState(&hi2c1));
		return result;
	}
	else
	{
		return -1;
	}
}

bool echo_speak(uint8_t reg, uint8_t speak_id)
{
	uint8_t trans_data[2] = {0};
	trans_data[0] = reg;
	trans_data[1] = speak_id;
	
	if(write_to_device(&wonder_echo, ECHO_SPEAK_REG, trans_data, sizeof(trans_data)))
	{
		return true;
	}
	
	return false;
}
