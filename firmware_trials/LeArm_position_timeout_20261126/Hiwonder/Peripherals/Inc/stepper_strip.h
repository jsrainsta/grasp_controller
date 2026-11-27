#ifndef __STEPPER_STRIP_H_
#define __STEPPER_STRIP_H_

#include "stdint.h"
#include "stdbool.h"

#define DEVICE_ADDR						0x35
#define MOTOR_STEPS_DRIVER_MODE_REG		0x15
#define MOTOR_AUTO_REPOSITION_REG		0x16
#define MOTOR_STEPS_REG					0x18
#define MOTOR_STEPS_TIME_REG			0x1C

#define SUBDIVISION_NONE				0x00
#define SUBDIVISION_2					0x01
#define SUBDIVISION_4					0x02
#define SUBDIVISION_8					0x03
#define SUBDIVISION_16					0x07

/* 8细分度下的最大总步数 */
#define MAX_DIV_8_STEPS					10400

typedef struct StepperStripHandle StepperStripHandleTypeDef;
struct StepperStripHandle
{
	uint8_t is_reset;
	uint8_t transmit_status;
	uint8_t receive_status;
	
	uint16_t dev_addr;

	int32_t steps;
	int32_t remain_steps;
	
	uint8_t (*write_data)(StepperStripHandleTypeDef* self, uint8_t* pdata, uint16_t size);
	uint8_t (*read_data)(StepperStripHandleTypeDef* self, uint8_t* pdata, uint16_t size);
};

void stepper_strip_init(void);
int8_t read_motor_is_reposition(void);
bool set_motor_reposition(void);
bool set_motor_subdivision(uint8_t num);
bool set_motor_move(int32_t step);

#endif
