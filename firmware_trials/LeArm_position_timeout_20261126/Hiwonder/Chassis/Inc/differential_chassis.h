#ifndef __MECANUM_CHASSIS_H_
#define __MECANUM_CHASSIS_H_

#include "stdint.h"
#include "stdbool.h"

void differential_chassis_init(void);
void differentials_chassis_run(float speed, float rot);

#endif
