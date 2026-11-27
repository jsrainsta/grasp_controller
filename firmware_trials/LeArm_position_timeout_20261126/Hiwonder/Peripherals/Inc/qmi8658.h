#ifndef __QMI8658_H__
#define __QMI8658_H__

#include "stdint.h"
#include "stdbool.h"
#include "string.h"

#define QMI8658_ADDR	0x68

#define ACCEL_XOUT_H	0x3B
#define ACCEL_XOUT_L	0x3C
#define ACCEL_YOUT_H	0x3D
#define ACCEL_YOUT_L	0x3E
#define ACCEL_ZOUT_H	0x3F
#define ACCEL_ZOUT_L	0x40

#define TEMP_OUT_H		0x41
#define TEMP_OUT_L		0x42

#define GYRO_XOUT_H		0x43
#define GYRO_XOUT_L		0x44
#define GYRO_YOUT_H		0x45
#define GYRO_YOUT_L		0x46
#define GYRO_ZOUT_H		0x47
#define GYRO_ZOUT_L		0x48

#define EULER_XOUT_H	0x49
#define EULER_XOUT_L	0x4A
#define EULER_YOUT_H	0x4B
#define EULER_YOUT_L	0x4C
#define EULER_ZOUT_H	0x4D
#define EULER_ZOUT_L	0x4E

typedef struct
{
	float x;
	float y;
	float z;
}ParametersObejctTypedDef;

typedef struct QMI8658Handle QMI8658HandleTypeDef;
struct QMI8658Handle
{
	uint16_t dev_addr;
	
	uint8_t rec_data[6];
	uint8_t transmit_status;
	uint8_t receive_status;
	
	ParametersObejctTypedDef acc;
	ParametersObejctTypedDef gyro;	
	
	uint8_t (*write_data)(QMI8658HandleTypeDef* self, uint8_t* pdata, uint16_t size);
	uint8_t (*read_data)(QMI8658HandleTypeDef* self, uint8_t* pdata, uint16_t size);
};

void qmi8658_init(void);
float qmi8658_get_temperature(void);
void qmi8658_get_accelerometer(float* acc);
void qmi8658_get_gyro(float* gyro);
void qmi8658_get_euler(float* euler);

#endif
