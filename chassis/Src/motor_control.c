#include "motor_control.h"
#include "tim.h"
#include <stdlib.h>
#include "user_lib.h"
#include "vofa.h"

motor_control_t motor_control;


#define MOTOR_REDUCTION_RITIO		30		//1:30
#define MOTOR_ENCODER_LINES			40		//ÏßÊý
#define WHEEL_RADIUS				0.06f	//m

#define MOTOR_VELOCITY_KP			1.0f
#define MOTOR_VELOCITY_KI			0.00f
#define MOTOR_VELOCITY_KD			1.2f
#define MOTOR_VELOCITY_MAX_OUT		25.0f
#define MOTOR_VELOCITY_MAX_IOUT		20.0f

static void motor_init(motor_control_t *motor_init_point);
static void motor_feedback(motor_control_t *motor_feedback_point);
static void motor_control_loop(motor_control_t *motor_control_loop_point);
static void motor_controller(uint8_t id, int16_t pwm);

void motor_task(void const * argument)
{
	motor_init(&motor_control);
	while(1)
	{
		motor_feedback(&motor_control);
		motor_control_loop(&motor_control);
		
		for(int i=0; i<3; i++)
			motor_controller(motor_control.motor_measure[i].encoder_measure.id, motor_control.motor_measure[i].give_pwm);
			
		vTaskDelay(5);
	}
}

static void motor_init(motor_control_t *motor_init_point)
{
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim5,TIM_CHANNEL_ALL);
	
	float motor_velocity_pid[] = {MOTOR_VELOCITY_KP, MOTOR_VELOCITY_KI, MOTOR_VELOCITY_KD};
	for(int i=0; i<3; i++)
	{
		motor_init_point->motor_measure[i].encoder_measure.id = i;
		PID_init(&motor_init_point->motor_measure[i].motor_velocity_pid, PID_POSITION, motor_velocity_pid, MOTOR_VELOCITY_MAX_OUT, MOTOR_VELOCITY_MAX_IOUT);
	}	
}


static void motor_feedback(motor_control_t *motor_feedback_point)
{
	for(int i=0; i<3; i++)
		motor_feedback_point->motor_measure[i].velocity = (float)motor_feedback_point->motor_measure[i].encoder_measure.rpm/60.0f*2.0f*PI*WHEEL_RADIUS;
}

static void motor_control_loop(motor_control_t *motor_control_loop_point)
{
	for(int i=0; i<3; i++)
	{
		PID_calc(&motor_control_loop_point->motor_measure[i].motor_velocity_pid, motor_control_loop_point->motor_measure[i].velocity, motor_control_loop_point->motor_measure[i].velocity_set);
		motor_control_loop_point->motor_measure[i].give_pwm = motor_control_loop_point->motor_measure[i].motor_velocity_pid.out*7200.0f;
//		LimitMax(motor_control_loop_point->motor_measure[i].give_pwm, 7200.0f);
	}
}

static void motor_controller(uint8_t id, int16_t pwm)
{
	switch(id)
	{
		case 0:
			if(pwm<0)
			{
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_SET);
			}
			else if(pwm>0)
			{
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_RESET);
			}
			else
			{
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_RESET);
			}
			__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, abs(pwm));
			break;
		case 1:
			if(pwm<0)
			{
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_SET);
			}
			else if(pwm>0)
			{
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_RESET);
			}
			else
			{
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_RESET);
			}
			__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, abs(pwm));
			break;
		case 2:
			if(pwm<0)
			{
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, GPIO_PIN_SET);
			}
			else if(pwm>0)
			{
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, GPIO_PIN_RESET);
			}
			else
			{
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, GPIO_PIN_RESET);
			}
			__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, abs(pwm));
			break;
	}
}



void read_encoder(encoder_measure_t *encoder_measure_point, uint16_t time_out)
{
	switch(encoder_measure_point->id)
	{
		case 0:
			encoder_measure_point->count_temp = __HAL_TIM_GetCounter(&htim3);
			__HAL_TIM_SetCounter(&htim3, 0);
			break;
		case 1:
			encoder_measure_point->count_temp = __HAL_TIM_GetCounter(&htim4);
			__HAL_TIM_SetCounter(&htim4, 0);
			break;
		case 2:
			encoder_measure_point->count_temp = __HAL_TIM_GetCounter(&htim5);
			__HAL_TIM_SetCounter(&htim5, 0);
			break;
	}
	encoder_measure_point->count += encoder_measure_point->count_temp;
	encoder_measure_point->rpm = (60000.0f * encoder_measure_point->count_temp) / (MOTOR_ENCODER_LINES * MOTOR_REDUCTION_RITIO * (float)time_out);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
   if(htim==&htim6)
    {
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
		for(int i=0; i<3; i++)
		{
			read_encoder(&motor_control.motor_measure[i].encoder_measure, htim6.Init.Prescaler/10);
			motor_control.motor_measure[i].angle = 2.0f*PI*(float)motor_control.motor_measure[i].encoder_measure.count /(MOTOR_ENCODER_LINES * MOTOR_REDUCTION_RITIO);
			motor_control.motor_measure[i].len = motor_control.motor_measure[i].angle*WHEEL_RADIUS;
		}
	}
}

const motor_control_t *get_motor_measure_point(void)
{
	return &motor_control;
}

void motor_cmd_vel(uint8_t id, float velocity)
{
	motor_control.motor_measure[id].velocity_set = velocity;
}
