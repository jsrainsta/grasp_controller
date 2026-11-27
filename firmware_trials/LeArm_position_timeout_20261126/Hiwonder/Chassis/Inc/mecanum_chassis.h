#ifndef __MECANUM_CHASSIS_H_
#define __MECANUM_CHASSIS_H_

#include "stdint.h"
#include "stdbool.h"

void mecanum_chassis_init(void);
void mecanum_chassis_run(uint16_t angle, uint8_t speed, int8_t rot, bool drift);

#endif
