#include "motor_module.h"
#include "string.h"
#include "i2c.h"
#include "global.h"

MotorModuleHandleTypeDef motor_module;

static uint8_t write_data(MotorModuleHandleTypeDef* self, uint8_t* pdata, uint16_t size)
{
	self->transmit_status = (uint8_t)HAL_I2C_Master_Transmit(&hi2c1, self->dev_addr << 1, pdata, size, 0xfff);
	return self->transmit_status;
}

static uint8_t read_data(MotorModuleHandleTypeDef* self, uint8_t* pdata, uint16_t size)
{	
	/* 使用HAL_I2C_Master_Receive_DMA这个函数需要把i2c的事件中断勾上 */
	self->receive_status = (uint8_t)HAL_I2C_Master_Receive_DMA(&hi2c1, self->dev_addr << 1, pdata, size);
	return self->receive_status;	
}


static bool write_to_device(MotorModuleHandleTypeDef* self, uint8_t reg, uint8_t* pdata, uint16_t size)
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

static bool receive_from_device(MotorModuleHandleTypeDef* self, uint8_t reg, uint8_t* pdata, uint16_t size)
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

void motor_init(uint8_t motor_type)
{
	memset(&motor_module, 0, sizeof(MotorModuleHandleTypeDef));
	motor_module.dev_addr = MOTOR_MODULE_ADDR;
	motor_module.read_data = read_data;
	motor_module.write_data = write_data;
	set_motor_type(MOTOR_TYPE_JGB37_520_12V_110RPM);
	set_motor_polarity(0);
}

bool get_motor_module_voltage(uint16_t* bat)
{
	uint8_t val[2] = {0};
	if(receive_from_device(&motor_module, ADC_BAT_REG, val, sizeof(val)))
	{
		while(HAL_I2C_STATE_READY != HAL_I2C_GetState(&hi2c1));
		*bat = BYTE_TO_HW(val[1], val[0]);
		return true;
	}
	return false;
}

bool get_motor_encoder(int32_t* encoder_val)
{
	int32_t val[4] = {0};
	if(receive_from_device(&motor_module, MOTOR_GET_ENCODER_REG, (uint8_t*)val, sizeof(val)))
	{
		while(HAL_I2C_STATE_READY != HAL_I2C_GetState(&hi2c1));
		for (uint8_t i = 0; i < 4; i++)
		{
			encoder_val[i] = val[i];
		}
		return true;
	}	
	return false;
}

bool set_motor_type(uint8_t type)
{
	if(write_to_device(&motor_module, MOTOR_TYPE_REG, &type, 1))
	{
		return true;
	}
	return false;
}

bool set_motor_polarity(uint8_t polarity)
{
	if(write_to_device(&motor_module, MOTOR_ENCODER_POLARITY_REG, &polarity, 1))
	{
		return true;
	}
	return false;
}

bool set_motor_pwm(int8_t* pwm)
{
	motor_module.set_pwm[0] = pwm[0];
	motor_module.set_pwm[1] = pwm[1];
	motor_module.set_pwm[2] = pwm[2];
	motor_module.set_pwm[3] = pwm[3];
	
	if(write_to_device(&motor_module, MOTOR_SET_PWM_REG, (uint8_t*)pwm, 4))
	{
		return true;
	}
	return false;
}

bool set_motor_speed(int8_t* speed)
{
	motor_module.set_speed[0] = speed[0];
	motor_module.set_speed[1] = speed[1];
	motor_module.set_speed[2] = speed[2];
	motor_module.set_speed[3] = speed[3];
	
	if(write_to_device(&motor_module, MOTOR_SET_SPEED_REG, (uint8_t*)speed, 4))
	{
		return true;
	}
	return false;
}
