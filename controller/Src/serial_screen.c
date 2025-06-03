#include "serial_screen.h"
#include "usart.h"

static uint8_t frame_buffer[1];
static uint8_t receive_state;

static line_t line_set;
static pos_t pos_set;

#define CAR_NO_MOVE			0		//底盘断电
#define CAR_CALIBRATE		1		//底盘校准
#define CAR_NORMAL_MOVE		2		//底盘速度模式
#define CAR_POSITION_MOVE	3		//底盘位置模式

void serial_screen_init(void)
{
	HAL_UART_Receive_IT(&huart1, frame_buffer, 1);
	receive_state = 0;
}

static uint8_t car_state = 0;
void serial_screen_call_back(void)
{
	if(frame_buffer[0]==0xf5&&car_state!=0x03)
	{
		receive_state = 0;
	}
	switch(receive_state)
	{
		case 0:
			if(frame_buffer[0]==0x5f)
				receive_state = 1;
			else
				receive_state = 0;
			break;
		case 1:
			car_state = frame_buffer[0];
			receive_state = 2;
			break;
		case 2:
			switch(car_state)
			{
				case CAR_NO_MOVE:
					car_set_no_move(0);
					break;
				case CAR_CALIBRATE:
					car_start_calibrate(0);
					break;
				case CAR_NORMAL_MOVE:
					switch(frame_buffer[0])
					{
						case 0x1a:
							line_set.vx = 0.5f;
							break;
						case 0x1b:
							line_set.vx = -0.5f;
							break;
						case 0x1c:
							line_set.vyaw = 0.5f;
							break;
						case 0x1d:
							line_set.vyaw = -0.5f;
							break;
						case 0xfa:
							line_set.vx = 0.0f;
							break;
						case 0xfb:
							line_set.vx = 0.0f;
							break;
						case 0xfc:
							line_set.vyaw = 0.0f;
							break;
						case 0xfd:
							line_set.vyaw = 0.0f;
							break;
					}
					car_set_line(0, line_set);
					break;
				case CAR_POSITION_MOVE:
					pos_set.x = 1.5f;
					pos_set.y = 3.0f;
					pos_set.yaw = 0.0f;
					car_set_pos(0, pos_set);
					break;
			}
			receive_state = 0;
			break;
	}
	
	HAL_UART_Receive_IT(&huart1, frame_buffer, 1);
}
