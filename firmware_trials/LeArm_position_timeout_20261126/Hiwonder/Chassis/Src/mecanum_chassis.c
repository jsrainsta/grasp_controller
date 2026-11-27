#include "mecanum_chassis.h"
#include "motor_module.h"
#include "math.h"

#define PI 3.1415926f

static float invSqrt(float number)
{
    volatile long i;
    volatile float x, y;
    volatile const float f = 1.5F;

    x = number * 0.5F;
    y = number;
    i = * (( long * ) &y);
    i = 0x5f375a86 - ( i >> 1 );
    y = * (( float * ) &i);
    y = y * ( f - ( x * y * y ) );
    return y;
}

void mecanum_chassis_init()
{
	motor_init(MOTOR_TYPE_JGB37_520_12V_110RPM);
}

	int8_t motor_speed[4];
void mecanum_chassis_run(uint16_t angle, uint8_t speed, int8_t rot, bool drift)
{

	float speed_factor = 1;
	double rad;

	angle += 90;
	rad = angle * PI / 180;
	speed_factor = (rot == 0) ? 1 : 0.5;
	
	speed *= invSqrt(2);
	
	if (drift)
	{
//		motor_speed[0] = (speed * sin(rad) - speed * cos(rad)) * speed_factor;
//		motor_speed[2] = -(speed * sin(rad) + speed * cos(rad)) * speed_factor;
//		motor_speed[3] = (speed * sin(rad) - speed * cos(rad)) * speed_factor - rot * speed_factor * 2;
//		motor_speed[1] = -(speed * sin(rad) + speed * cos(rad)) * speed_factor + rot * speed_factor * 2;
		
		motor_speed[0] = (speed * sin(rad) - speed * cos(rad)) * speed_factor + rot * speed_factor * 2;		/* 俯视 右前轮 - M1口*/
		motor_speed[1] = -(speed * sin(rad) + speed * cos(rad)) * speed_factor - rot * speed_factor * 2;	/* 俯视 右后轮 - M2口*/
		motor_speed[2] = -(speed * sin(rad) + speed * cos(rad)) * speed_factor;								/* 俯视 左前轮 - M3口*/
		motor_speed[3] = (speed * sin(rad) - speed * cos(rad)) * speed_factor;								/* 俯视 左后轮 - M4口*/
	}
	else
	{
		
		motor_speed[0] = (speed * sin(rad) - speed * cos(rad)) * speed_factor + rot * speed_factor;			/* 俯视 右前轮 - M1口*/
		motor_speed[1] = -(speed * sin(rad) + speed * cos(rad)) * speed_factor - rot * speed_factor;		/* 俯视 右后轮 - M2口*/
		motor_speed[2] = -(speed * sin(rad) + speed * cos(rad)) * speed_factor + rot * speed_factor;		/* 俯视 左前轮 - M3口*/
		motor_speed[3] = (speed * sin(rad) - speed * cos(rad)) * speed_factor - rot * speed_factor;			/* 俯视 左后轮 - M4口*/
	}
	
	set_motor_speed(motor_speed);
}
