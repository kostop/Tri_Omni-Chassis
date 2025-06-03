#include "chassis_task.h"
#include <math.h>

chassis_para_t chassis_para;

#define CHASSIS_TASK_TIME_OUT	5

#define CHASSIS_POS_KP				0.5f
#define CHASSIS_POS_KI				0.0f
#define CHASSIS_POS_KD				0.0f
#define CHASSIS_POS_MAX_OUT			50.0f
#define CHASSIS_POS_MAX_IOUT		0.0f

#define CHASSIS_ANGLE_KP			4.0f		//450.0f	编码器
#define CHASSIS_ANGLE_KI			0.0f
#define CHASSIS_ANGLE_KD			0.0f
#define CHASSIS_ANGLE_MAX_OUT		10.0f
#define CHASSIS_ANGLE_MAX_IOUT		50.0f


#define CHASSIS_SPEED_MAX			0.9f
#define CHASSIS_YAW_SPEED			4.0f//0.34906585039886591538473815369772f		//Z轴线速度	rad/s

#define CHASSIS_RADIUS				0.15f
#define WHEEL_RADIUS				0.12f

#define CAR_NO_MOVE			0
#define CAR_CALIBRATE		1
#define CAR_NORMAL_MOVE		2
#define CAR_POSITION_MOVE	3

static void chassis_init(chassis_para_t *chassis_init_point);
static void chassis_feedback(chassis_para_t *chassis_feedback_point);
static void chassis_set_control(chassis_para_t *chassis_set_control_point);
static void chasiss_control_loop(chassis_para_t *chassis_control_loop_point);

/*
	数据限幅函数
*/
float limit_Data(float val, float hight, float low)
{
	if(val<low) val = low;
	else val = val;
	if(val>hight) val = hight;
	else val = val;
	return val;
}


void chassis_task(void const * argument)
{
	vTaskDelay(2000);
	chassis_init(&chassis_para);
	while(1)
	{
		chassis_feedback(&chassis_para);
		chassis_set_control(&chassis_para);
		chasiss_control_loop(&chassis_para);
		
		for(int i=0; i<3; i++)
			motor_cmd_vel(i, chassis_para.motor_vel_set[i]);
		vTaskDelay(CHASSIS_TASK_TIME_OUT);
	}
}


static void chassis_init(chassis_para_t *chassis_init_point)
{
	chassis_init_point->chassis_UWB_para = get_UWB_point();
	chassis_init_point->chassis_INS_para = get_INS_point();
	chassis_init_point->chassis_motor_para = get_motor_measure_point();

	float chassis_pos_pid[] = {CHASSIS_POS_KP, CHASSIS_POS_KI, CHASSIS_POS_KD};
	float chassis_angle_pid[] = {CHASSIS_ANGLE_KP, CHASSIS_ANGLE_KI, CHASSIS_ANGLE_KD};

	PID_init(&chassis_init_point->chassis_pos_pid[0], PID_POSITION, chassis_pos_pid, CHASSIS_POS_MAX_OUT, CHASSIS_POS_MAX_IOUT);
	PID_init(&chassis_init_point->chassis_pos_pid[1], PID_POSITION, chassis_pos_pid, CHASSIS_POS_MAX_OUT, CHASSIS_POS_MAX_IOUT);
	PID_init(&chassis_init_point->chassis_angle_pid, PID_POSITION, chassis_angle_pid, CHASSIS_ANGLE_MAX_OUT, CHASSIS_ANGLE_MAX_IOUT);
}

static void chassis_feedback(chassis_para_t *chassis_feedback_point)
{
	chassis_feedback_point->chassis_pos.yaw = chassis_feedback_point->chassis_INS_para->INS_attitude.yaw;
	chassis_feedback_point->chassis_state = chassis_feedback_point->chassis_UWB_para->state;
	chassis_feedback_point->chassis_pos.x = chassis_feedback_point->chassis_UWB_para->car_pos.x;
	chassis_feedback_point->chassis_pos.y = chassis_feedback_point->chassis_UWB_para->car_pos.y;
}

static void chassis_set_control(chassis_para_t *chassis_set_control_point)
{
	switch(chassis_set_control_point->chassis_state)
	{
		case CAR_NO_MOVE:
			memset(&chassis_set_control_point->chassis_pos_set, 0, sizeof(chassis_set_control_point->chassis_pos_set));
			memset(&chassis_set_control_point->chassis_line_set, 0, sizeof(chassis_set_control_point->chassis_line_set));
			break;
		case CAR_CALIBRATE:
			chassis_set_control_point->chassis_pos_set.x = 0;
			chassis_set_control_point->chassis_pos_set.y = 0;
			chassis_set_control_point->chassis_pos_set.yaw = chassis_set_control_point->chassis_UWB_para->car_set_pos.yaw;
			break;
		case CAR_NORMAL_MOVE:
			chassis_set_control_point->chassis_line_set.vx = chassis_set_control_point->chassis_UWB_para->car_set_line.vx*2.0f;
			chassis_set_control_point->chassis_line_set.vy = chassis_set_control_point->chassis_UWB_para->car_set_line.vy*2.0f;
			chassis_set_control_point->chassis_line_set.vyaw = chassis_set_control_point->chassis_UWB_para->car_set_line.vyaw*3.0f;
			break;
	
		case CAR_POSITION_MOVE:
			chassis_set_control_point->chassis_pos_set.x = chassis_set_control_point->chassis_UWB_para->car_set_pos.x;
			chassis_set_control_point->chassis_pos_set.y = chassis_set_control_point->chassis_UWB_para->car_set_pos.y;
			chassis_set_control_point->chassis_pos_set.yaw = chassis_set_control_point->chassis_UWB_para->car_set_pos.yaw;
			break;
	}
	
}



//位置环计算
void chassis_position_move_task(chassis_para_t *chassis_para_position_count)
{
	float vx_map_set = PID_calc(&chassis_para_position_count->chassis_pos_pid[0], chassis_para_position_count->chassis_pos.x, chassis_para_position_count->chassis_pos_set.x);
	float vy_map_set = PID_calc(&chassis_para_position_count->chassis_pos_pid[1], chassis_para_position_count->chassis_pos.y, chassis_para_position_count->chassis_pos_set.y);
	
	
	float cos_angle = cos(chassis_para_position_count->chassis_pos.yaw);
	float sin_angle = sin(chassis_para_position_count->chassis_pos.yaw);
	
	float before_limit_vx = cos_angle * vx_map_set + sin_angle * vy_map_set;
	float before_limit_vy = -sin_angle * vx_map_set + cos_angle * vy_map_set;
	float before_limit_wz = PID_calc(&chassis_para_position_count->chassis_angle_pid, chassis_para_position_count->chassis_pos.yaw, chassis_para_position_count->chassis_pos_set.yaw);
	
	chassis_para_position_count->chassis_line_set.vx = limit_Data(before_limit_vx, CHASSIS_SPEED_MAX, -CHASSIS_SPEED_MAX);
	chassis_para_position_count->chassis_line_set.vy = limit_Data(before_limit_vy, CHASSIS_SPEED_MAX, -CHASSIS_SPEED_MAX);
//	chassis_para_position_count->chassis_line_set.vx = before_limit_vx;
//	chassis_para_position_count->chassis_line_set.vy = before_limit_vy;
	chassis_para_position_count->chassis_line_set.vyaw = limit_Data(before_limit_wz, CHASSIS_YAW_SPEED, -CHASSIS_YAW_SPEED);
}


float chassis_move_backward_decomposition(const motor_measure_t motor_measure[3])
{
	float len_total;
	float result;
	len_total = motor_measure[0].len + motor_measure[1].len + motor_measure[2].len;
	result = len_total/3.0f/CHASSIS_RADIUS;
	return result;
}

//运动正解算
void chassis_move_forward_decomposition(float motor_rate_set[3], line_t chassis_line_set)
{
   motor_rate_set[0] = (chassis_line_set.vx*0.866f - chassis_line_set.vy*0.5f - chassis_line_set.vyaw*CHASSIS_RADIUS);
   motor_rate_set[1] = (chassis_line_set.vy - chassis_line_set.vyaw*CHASSIS_RADIUS);
   motor_rate_set[2] = (-chassis_line_set.vx*0.866f - chassis_line_set.vy*0.5f - chassis_line_set.vyaw*CHASSIS_RADIUS);
   
//   motor_cmd_vel(3, -0.5f);


}

static void chasiss_control_loop(chassis_para_t *chassis_control_loop_point)
{
//	chassis_control_loop_point->chassis_pos.yaw = -chassis_move_backward_decomposition(chassis_control_loop_point->chassis_motor_para->motor_measure);

	if(chassis_control_loop_point->chassis_state == CAR_POSITION_MOVE)
		chassis_position_move_task(chassis_control_loop_point);
	
	chassis_move_forward_decomposition(chassis_control_loop_point->motor_vel_set, chassis_control_loop_point->chassis_line_set);
}
