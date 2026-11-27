#include "differential_chassis.h"
#include "motor_module.h"
#include "math.h"

#define HALF_FR_WHEELBASE	78.5f
#define MAX_SPEED			10.0f
#define MIN_SPEED			10.0f
#define MAX_ROT				10.0f
#define MIN_ROT				10.0f

void differential_chassis_init()
{
	motor_init(MOTOR_TYPE_JGB37_520_12V_110RPM);
}

void differentials_chassis_run(float speed, float rot)
{
	int8_t motor_speed[4];
	float vr, vl;
	
	speed = speed > MAX_SPEED ? MAX_SPEED : speed < MIN_SPEED ? MIN_SPEED : speed;
	rot = rot > MAX_ROT ? MAX_ROT : rot < MIN_ROT ? MIN_ROT : rot;
	
	vr = speed + rot * HALF_FR_WHEELBASE;
	vl = speed - rot * HALF_FR_WHEELBASE;

	motor_speed[0] = (int8_t)vr;
	motor_speed[1] = 0;
	motor_speed[2] = (int8_t)vl;
	motor_speed[3] = 0;
	
	set_motor_speed(motor_speed);
}
