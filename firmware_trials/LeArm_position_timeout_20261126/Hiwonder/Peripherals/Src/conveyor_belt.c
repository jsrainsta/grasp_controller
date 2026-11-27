#include "conveyor_belt.h"
#include "string.h"
#include "i2c.h"

ConveyorBeltHandleTypeDef conveyor_belt;

static uint8_t write_data(ConveyorBeltHandleTypeDef* self, uint8_t* pdata, uint16_t size)
{
	self->transmit_status = (uint8_t)HAL_I2C_Master_Transmit(&hi2c1, self->dev_addr << 1, pdata, size, 0xfff);
	return self->transmit_status;
}

static uint8_t read_data(ConveyorBeltHandleTypeDef* self, uint8_t* pdata, uint16_t size)
{	
	/* 使用HAL_I2C_Master_Receive_DMA这个函数需要把i2c的事件中断勾上 */
	self->receive_status = (uint8_t)HAL_I2C_Master_Receive_DMA(&hi2c1, self->dev_addr << 1, pdata, size);
	return self->receive_status;	
}


static bool write_to_device(ConveyorBeltHandleTypeDef* self, uint8_t reg, uint8_t* pdata, uint16_t size)
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

static bool receive_from_device(ConveyorBeltHandleTypeDef* self, uint8_t reg, uint8_t* pdata, uint16_t size)
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

void conveyor_belt_init()
{
	memset(&conveyor_belt, 0, sizeof(ConveyorBeltHandleTypeDef));
	conveyor_belt.dev_addr = CONVEYORBELT_ADDR;
	conveyor_belt.read_data = read_data;
	conveyor_belt.write_data = write_data;
	
}

bool set_conveyor_belt_speed(int speed)
{
	if(write_to_device(&conveyor_belt, 0x00, (uint8_t*)speed, 1))
	{
		return true;
	}
	return false;
}