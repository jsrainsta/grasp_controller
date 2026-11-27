#ifndef __CONVEYOR_BELT_H__
#define __CONVEYOR_BELT_H__

#include "stdint.h"
#include "stdbool.h"

#define CONVEYORBELT_ADDR	0x37

typedef struct ConveyorBeltHandle ConveyorBeltHandleTypeDef;
struct ConveyorBeltHandle
{
	uint8_t transmit_status;
	uint8_t receive_status;
	
	uint16_t dev_addr;

	uint8_t (*write_data)(ConveyorBeltHandleTypeDef* self, uint8_t* pdata, uint16_t size);
	uint8_t (*read_data)(ConveyorBeltHandleTypeDef* self, uint8_t* pdata, uint16_t size);
};

void conveyor_belt_init(void);
bool set_conveyor_belt_speed(int speed);
	
#endif
