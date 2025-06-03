#ifndef _CHASSIS_TASK_H
#define _CHASSIS_TASK_H

#include "cmsis_os.h"
#include "dw_task.h"
#include "INS_task.h"
#include "pid.h"
#include "motor_control.h"

typedef struct
{
	const dw1000_para_t *chassis_UWB_para;
	const INS_para_t *chassis_INS_para;
	const motor_control_t *chassis_motor_para;
	
	
	
	pid_type_def	chassis_pos_pid[2];
	pid_type_def	chassis_angle_pid;
	
	uint8_t chassis_state;
	pos_t	chassis_pos_set;
	pos_t	chassis_pos;
	line_t	chassis_line_set;
	line_t	chassis_line;
	
	float motor_vel_set[3];

}chassis_para_t;


void chassis_task(void const * argument);

#endif
