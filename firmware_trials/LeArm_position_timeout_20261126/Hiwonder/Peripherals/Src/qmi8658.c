#include "qmi8658.h"
#include "i2c.h"

#define BYTE_TO_HW(A, B) ((((uint16_t)(A)) << 8) | (uint8_t)(B))

QMI8658HandleTypeDef qmi8658;

static uint8_t write_data(QMI8658HandleTypeDef* self, uint8_t* pdata, uint16_t size)
{
	self->transmit_status = (uint8_t)HAL_I2C_Master_Transmit(&hi2c1, self->dev_addr << 1, pdata, size, 0xfff);
	return self->transmit_status;
}

static uint8_t read_data(QMI8658HandleTypeDef* self, uint8_t* pdata, uint16_t size)
{	
	/* 使用HAL_I2C_Master_Receive_DMA这个函数需要把i2c的事件中断勾上 */
	self->receive_status = (uint8_t)HAL_I2C_Master_Receive_DMA(&hi2c1, self->dev_addr << 1, pdata, size);
	return self->receive_status;	
}


static bool write_to_device(QMI8658HandleTypeDef* self, uint8_t reg, uint8_t* pdata, uint16_t size)
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

static bool receive_from_device(QMI8658HandleTypeDef* self, uint8_t reg, uint8_t* pdata, uint16_t size)
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

void qmi8658_init(void)
{
	memset(&qmi8658, 0, sizeof(QMI8658HandleTypeDef));
	qmi8658.dev_addr = QMI8658_ADDR;
	qmi8658.read_data = read_data;
	qmi8658.write_data = write_data;
}

void qmi8658_get_accelerometer(float* acc)
{
	for(uint8_t i = 0; i < 6; i++)
	{
		receive_from_device(&qmi8658, ACCEL_XOUT_H + i, &qmi8658.rec_data[i], 1);
		while(HAL_I2C_STATE_READY != HAL_I2C_GetState(&hi2c1));
	}
	acc[0] = (float)((int16_t)BYTE_TO_HW(qmi8658.rec_data[0], qmi8658.rec_data[1])) / 100.0f;
	acc[1] = (float)((int16_t)BYTE_TO_HW(qmi8658.rec_data[2], qmi8658.rec_data[3])) / 100.0f;
	acc[2] = (float)((int16_t)BYTE_TO_HW(qmi8658.rec_data[4], qmi8658.rec_data[5])) / 100.0f;
}

void qmi8658_get_gyro(float* gyro)
{
	for(uint8_t i = 0; i < 6; i++)
	{
		receive_from_device(&qmi8658, GYRO_XOUT_H + i, &qmi8658.rec_data[i], 1);
		while(HAL_I2C_STATE_READY != HAL_I2C_GetState(&hi2c1));
	}
	gyro[0] = (float)((int16_t)BYTE_TO_HW(qmi8658.rec_data[0], qmi8658.rec_data[1])) / 100.0f;
	gyro[1] = (float)((int16_t)BYTE_TO_HW(qmi8658.rec_data[2], qmi8658.rec_data[3])) / 100.0f;
	gyro[2] = (float)((int16_t)BYTE_TO_HW(qmi8658.rec_data[4], qmi8658.rec_data[5])) / 100.0f;	
}

void qmi8658_get_euler(float* euler)
{
	for(uint8_t i = 0; i < 6; i++)
	{
		receive_from_device(&qmi8658, EULER_XOUT_H + i, &qmi8658.rec_data[i], 1);
		while(HAL_I2C_STATE_READY != HAL_I2C_GetState(&hi2c1));
	}
	euler[0] = (float)((int16_t)BYTE_TO_HW(qmi8658.rec_data[0], qmi8658.rec_data[1])) / 100.0f;
	euler[1] = (float)((int16_t)BYTE_TO_HW(qmi8658.rec_data[2], qmi8658.rec_data[3])) / 100.0f;
	euler[2] = (float)((int16_t)BYTE_TO_HW(qmi8658.rec_data[4], qmi8658.rec_data[5])) / 100.0f;
}
