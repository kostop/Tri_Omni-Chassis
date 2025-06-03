#include "dw_task.h"
//#include "spi.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "ins_task.h"
#include "serial_screen.h"

dw1000_para_t	dw1000_para;
dw1000_rx_data_t	dw1000_rx_data;

#define DW_TASK_TIME_OUT	2
#define RX_DATA_LEN		18

#define CAR_NO_MOVE			0		//���̶ϵ�
#define CAR_CALIBRATE		1		//����У׼
#define CAR_NORMAL_MOVE		2		//�����ٶ�ģʽ
#define CAR_POSITION_MOVE	3		//����λ��ģʽ

static uint8_t rx_state=0;
static uint8_t rx_buffer[1];
static uint8_t rx_data[RX_DATA_LEN];
static uint8_t rx_index=0;

static uint8_t tx_data[17];

static void DW1000_UART_Init(void);
static void dw1000_send_data(dw1000_para_t *dw1000_para_point);

void dw_task(void const * argument)
{
	DW1000_UART_Init();
	dw1000_para.dw1000_INS_point = get_INS_point();
	serial_screen_init();
	while(1)
	{
		dw1000_send_data(&dw1000_para);
		vTaskDelay(DW_TASK_TIME_OUT);
	}
}

void car_set_no_move(uint8_t id)
{
	dw1000_para.car_para[id].state = CAR_NO_MOVE;
}
void car_set_pos(uint8_t id, pos_t car_pos)
{
	dw1000_para.car_para[id].state = CAR_POSITION_MOVE;
	dw1000_para.car_para[id].tar_pos = car_pos;
}

void car_set_line(uint8_t id, line_t car_line)
{
	dw1000_para.car_para[id].state = CAR_NORMAL_MOVE;
	dw1000_para.car_para[id].tar_line = car_line;
}

void car_start_calibrate(uint8_t id)
{
	dw1000_para.car_para[id].state = CAR_CALIBRATE;
}

void car_stop_calibrate(uint8_t id)
{
	dw1000_para.car_para[id].state = CAR_NORMAL_MOVE;
	dw1000_para.car_para[id].tar_line.vx = 0;
	dw1000_para.car_para[id].tar_line.vy = 0;
	dw1000_para.car_para[id].tar_line.vyaw = 0;
}

static void DW1000_UART_Init(void)
{
	HAL_UART_Receive_IT(&huart2, rx_buffer, 1);
}



/*
	ң���� C8T6-->>UWB C8T6
	���͸�UWB C8T6�����ݰ�
	�����Ѿ�д����
*/
static void dw1000_send_data(dw1000_para_t *dw1000_para_point)
{
	dw1000_para_point->dw1000_tx.header1 = 0xe5;
	dw1000_para_point->dw1000_tx.header2 = 0x12;
	dw1000_para_point->dw1000_tx.ender = 0x5e;
	for(int i=0; i<TAG_MAX_NUM; i++)
	{
		if(i==0)
			dw1000_para_point->dw1000_tx.id = 0x0f;
		else
			dw1000_para_point->dw1000_tx.id = i;
		dw1000_para_point->dw1000_tx.state = dw1000_para_point->car_para[i].state;
		switch(dw1000_para_point->car_para[i].state)
		{
			case CAR_NO_MOVE:
				dw1000_para_point->dw1000_tx.x = 0;
				dw1000_para_point->dw1000_tx.y = 0;
				dw1000_para_point->dw1000_tx.yaw = 0;
				break;
			case CAR_POSITION_MOVE:
				dw1000_para_point->dw1000_tx.x = dw1000_para_point->car_para[i].tar_pos.x;
				dw1000_para_point->dw1000_tx.y = dw1000_para_point->car_para[i].tar_pos.y;
				dw1000_para_point->dw1000_tx.yaw = dw1000_para_point->car_para[i].tar_pos.yaw;
				break;
			case CAR_NORMAL_MOVE:
				dw1000_para_point->dw1000_tx.x = dw1000_para_point->car_para[i].tar_line.vx;
				dw1000_para_point->dw1000_tx.y = dw1000_para_point->car_para[i].tar_line.vy;
				dw1000_para_point->dw1000_tx.yaw = dw1000_para_point->car_para[i].tar_line.vyaw;
				break;
			case CAR_CALIBRATE:
				dw1000_para_point->dw1000_tx.x = 0;
				dw1000_para_point->dw1000_tx.y = 0;
				dw1000_para_point->dw1000_tx.yaw = dw1000_para_point->dw1000_INS_point->yaw_start;
				break;
		}
		
		memcpy(&tx_data, &dw1000_para_point->dw1000_tx, sizeof(tx_data));
		HAL_UART_Transmit(&huart2, tx_data, sizeof(tx_data), 10);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//UWB C8T6-->>ң���� C8T6
	if(huart->Instance == USART2)
	{
		switch(rx_state)
		{
			case 0:
				if(rx_buffer[0]==0x6d)
					rx_state = 1;
				else
					rx_state = 0;
				break;
			case 1:
				if(rx_buffer[0]==0x72)
					rx_state = 2;
				else
					rx_state = 0;
				break;
			case 2:
				if(rx_buffer[0]==0x02)
					rx_state = 3;
				else
					rx_state = 0;
				break;
			case 3:
				if(rx_buffer[0]==0x01)
					rx_state = 4;
				break;
			case 4:
				if(rx_buffer[0]==0x10)
					rx_state = 5;
				else
					rx_state = 3;
				break;
		}
		if(rx_state!=0)
			rx_data[rx_index++] = rx_buffer[0];
		else
			rx_index = 0;
		if(rx_state == 5)
		{
			memcpy(&dw1000_rx_data, rx_data, RX_DATA_LEN);
			memset(&rx_data, 0, RX_DATA_LEN);
			rx_state = 0;
			rx_index = 0;
		}
		else if(rx_index>RX_DATA_LEN)
		{
			rx_state = 5;
			rx_index = 0;
		}
		HAL_UART_Receive_IT(&huart2, rx_buffer, 1);
	}
	//������-->>ң���� C8T6
	else if(huart->Instance == USART1)
	{
		serial_screen_call_back();
	}
	//������-->>ң���� C8T6
	else if(huart->Instance == USART3)
	{
		INS_Communication_Callback();
	}
}



const dw1000_para_t *get_pos_point(void)
{
	return &dw1000_para;
}

