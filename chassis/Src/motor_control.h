#ifndef _MOTOR_CONTROL_H
#define _MOTOR_CONTROL_H

#include "cmsis_os.h"
#include "pid.h"

typedef struct
{
	uint8_t id;
	int16_t count;
	int16_t count_temp;
	int16_t rpm;
}encoder_measure_t;

typedef struct
{
	encoder_measure_t	encoder_measure;
	pid_type_def	motor_velocity_pid;
	float rate;
	float velocity;
	
	float rate_set;
	float velocity_set;
	float give_pwm;
	
	float angle;
	float len;
}motor_measure_t;

typedef struct
{
	motor_measure_t motor_measure[3]; 
}motor_control_t;

void motor_task(void const * argument);
void motor_cmd_vel(uint8_t id, float velocity);
const motor_control_t *get_motor_measure_point(void);

#endif
