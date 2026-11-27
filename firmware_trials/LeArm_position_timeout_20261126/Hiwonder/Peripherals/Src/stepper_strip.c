#include "stepper_strip.h"

#include "string.h"
#include "i2c.h"

StepperStripHandleTypeDef stepper_strip;

static void delay_ms(uint32_t ms)
{
	HAL_Delay(ms);
}

static uint8_t write_data(StepperStripHandleTypeDef* self, uint8_t* pdata, uint16_t size)
{
	self->transmit_status = (uint8_t)HAL_I2C_Master_Transmit(&hi2c1, self->dev_addr << 1, pdata, size, 0xfff);
	return self->transmit_status;
}

static uint8_t read_data(StepperStripHandleTypeDef* self, uint8_t* pdata, uint16_t size)
{	
	self->receive_status = (uint8_t)HAL_I2C_Master_Receive(&hi2c1,  self->dev_addr << 1, pdata, size, 0xfff);
	return self->receive_status;	
}

static bool write_to_device(StepperStripHandleTypeDef* self, uint8_t reg, uint8_t* pdata, uint16_t size)
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

static bool receive_from_device(StepperStripHandleTypeDef* self, uint8_t reg, uint8_t* pdata, uint16_t size)
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

static int32_t _abs(int32_t val)
{
	int32_t abs_val;
	abs_val = val >= 0 ? val : (-val);
	return abs_val;
}

void stepper_strip_init()
{
	delay_ms(1000);
	memset(&stepper_strip, 0, sizeof(StepperStripHandleTypeDef));
	stepper_strip.dev_addr = DEVICE_ADDR;
	stepper_strip.read_data = read_data;
	stepper_strip.write_data = write_data;
	stepper_strip.remain_steps = MAX_DIV_8_STEPS;
	if(read_motor_is_reposition() == 0)
	{
		set_motor_reposition();
		while(1)
		{
			delay_ms(1000);
			if(read_motor_is_reposition() == 1)
			{
				set_motor_subdivision(SUBDIVISION_8);
				break;
			}
		}
	}
}

int8_t read_motor_is_reposition()
{
	if(receive_from_device(&stepper_strip, 
						   MOTOR_AUTO_REPOSITION_REG, 
						   &stepper_strip.is_reset, 
						   sizeof(stepper_strip.is_reset)) == false)
	{
		return -1;
	}
	return stepper_strip.is_reset;
}

bool set_motor_reposition()
{
	uint8_t set_value = 1;
	
	return write_to_device(&stepper_strip, MOTOR_AUTO_REPOSITION_REG, &set_value, sizeof(set_value));
}

bool set_motor_subdivision(uint8_t num)
{
	uint8_t set_subdivision = num;
	return write_to_device(&stepper_strip, MOTOR_STEPS_DRIVER_MODE_REG, &set_subdivision, sizeof(set_subdivision));
}

bool set_motor_move(int32_t step)
{
	stepper_strip.steps = step;
	
//	if(step >= 0)
//	{
//		stepper_strip.steps = step < stepper_strip.remain_steps ? step : stepper_strip.remain_steps;
//	}
//	else
//	{
//		stepper_strip.steps = step > (stepper_strip.remain_steps - MAX_DIV_8_STEPS) ? step : (stepper_strip.remain_steps - MAX_DIV_8_STEPS);
//	}
//	
//	stepper_strip.remain_steps -= stepper_strip.steps;
	
	return write_to_device(&stepper_strip, MOTOR_STEPS_REG, (uint8_t*)&stepper_strip.steps, sizeof(stepper_strip.steps));
}
