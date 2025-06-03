#include "dw_task.h"
//#include "spi.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "ins_task.h"

#define CAR_NO_MOVE			0
#define CAR_CALIBRATE		1
#define CAR_NORMAL_MOVE		2
#define CAR_POSITION_MOVE	3

dw1000_para_t	dw1000_para;
//dw1000_rx_data_t	dw1000_rx_data;

#define DW_TASK_TIME_OUT	1.0f
#define RX_DATA_LEN		11
static uint8_t rx_state=0;
static uint8_t rx_buffer[1];
static uint8_t rx_data[RX_DATA_LEN];
static uint8_t rx_index=0;
uint8_t ender_resever=0;

static void DW1000_UART_Init(void);
static void tag_pos_count(dw1000_para_t *dw_para_point);

float parseSignedFloat(uint8_t highByte, uint8_t lowByte, float scale) {
    int16_t value = (highByte << 8) | lowByte; // ���Ϊ16λ�з�������
    return (float)value / scale; // Ӧ����������
}

// ��������ת��Ϊ�з���16λ���������Ϊ�����ֽ�
void floatToSignedBytes(float value, float scale, uint8_t* highByte, uint8_t* lowByte) {
    int16_t fixedPoint = (int16_t)(value * scale); // ת��Ϊ������
    *highByte = (fixedPoint >> 8) & 0xFF; // ���ֽ�
    *lowByte = fixedPoint & 0xFF; // ���ֽ�
}

void dw_task(void const * argument)
{
	DW1000_UART_Init();
	for(int i=0; i<2; i++)
		first_order_filter_init(&dw1000_para.pos_filter[i], (DW_TASK_TIME_OUT/1000.0f), 0.1f);
	while(1)
	{
//		tag_pos_count(&dw1000_para);
		vTaskDelay(DW_TASK_TIME_OUT);
	}
}

//static float y_before_filter, x_before_filter;
//static float cos_data;
//static void tag_pos_count(dw1000_para_t *dw_para_point)
//{
//	float a, b, c, d, e;
//	a = dw_para_point->dis.anthor_tag_dis;
//	b = dw_para_point->dis.anthor_dis;
//	c = dw_para_point->dis.tag_dis[0][2];
//	d = dw_para_point->dis.tag_dis[0][1];
//	e = dw_para_point->dis.tag_dis[0][0];
//	
//	if(a==0&&b==0&&c==0&&d==0&&e==0)
//		return;
//	cos_data = (pow(a,2)+pow(e,2)-pow(d,2))/(2.0f*a*e);
//	x_before_filter = e*sqrt(1.0f-pow(cos_data,2));
//	y_before_filter = e*cos_data;
//	
////	first_order_filter_cali(&dw_para_point->pos_filter[0], x_before_filter);
////	first_order_filter_cali(&dw_para_point->pos_filter[1], y_before_filter);
//	dw_para_point->pos.tag_pos[0].x = x_before_filter;
//	dw_para_point->pos.tag_pos[0].y = y_before_filter;
//}

static void DW1000_UART_Init(void)
{
	HAL_UART_Receive_IT(&huart2, rx_buffer, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2)
    {
        // �洢��ǰ���յ����ֽ�
        uint8_t current_byte = rx_buffer[0];
        
        switch(rx_state)
        {
            case 0: // �ȴ�֡ͷ 0x6d
                if(current_byte == 0x6d)
                {
                    rx_state = 1;
                    rx_index = 0; // ��������
                    rx_data[rx_index++] = current_byte; // ����֡ͷ
                }
                else
                {
                    rx_state = 0;
                    rx_index = 0;
                }
                break;
                
            case 1: // �ȴ��ڶ���֡ͷ 0x72
                if(current_byte == 0x72)
                {
                    rx_state = 2;
                    rx_data[rx_index++] = current_byte;
                }
                else
                {
                    rx_state = 0; // ֡ͷ��ƥ�䣬����
                    rx_index = 0;
                }
                break;
                
            case 2: // �ȴ���������
                rx_data[rx_index++] = current_byte;
                if(current_byte == 0x01)
                    rx_state = 3;
                else
                    rx_state = 2; // �����ȴ���Ч����
                break;
                
            case 3: // �ȴ��������
                rx_data[rx_index++] = current_byte;
                if(current_byte == 0x10)
                    rx_state = 5;
                else
                    rx_state = 2; // ����Ԥ�ڲ������ص���������״̬
                break;
                
            case 5: // ������������
                
            default:
                rx_state = 0;
                rx_index = 0;
                break;
        }
        if(rx_state==5)
		{
			// ����Ƿ��������������
			if(rx_index == RX_DATA_LEN)
			{
				// ���ݴ���
				dw1000_para.state = rx_data[2];
				if(rx_data[2] == CAR_NORMAL_MOVE) // �ٶ�����
				{
					dw1000_para.car_set_line.vx = parseSignedFloat(rx_data[3], rx_data[4], 100.0f);
					dw1000_para.car_set_line.vy = parseSignedFloat(rx_data[5], rx_data[6], 100.0f);
					dw1000_para.car_set_line.vyaw = parseSignedFloat(rx_data[7], rx_data[8], 100.0f);
				}
				else if(rx_data[2] == CAR_POSITION_MOVE) // λ������
				{
					dw1000_para.car_set_pos.x = parseSignedFloat(rx_data[3], rx_data[4], 100.0f);
					dw1000_para.car_set_pos.y = parseSignedFloat(rx_data[5], rx_data[6], 100.0f);
					dw1000_para.car_set_pos.yaw = parseSignedFloat(rx_data[7], rx_data[8], 100.0f);
				}
				else if(rx_data[2] == CAR_POSITION_MOVE+1)
				{
					dw1000_para.car_pos.x = parseSignedFloat(rx_data[3], rx_data[4], 100.0f);
					dw1000_para.car_pos.y = parseSignedFloat(rx_data[5], rx_data[6], 100.0f);
					dw1000_para.car_pos.yaw = parseSignedFloat(rx_data[7], rx_data[8], 100.0f);
				}
				
				rx_state = 0; // ����״̬��
				rx_index = 0;
			}
		}
                
        // �������������ж�
        HAL_UART_Receive_IT(&huart2, rx_buffer, 1);
    }
    else if(huart->Instance == USART3)
    {
        INS_Communication_Callback();
    }
}



const dw1000_para_t *get_UWB_point(void)
{
	return &dw1000_para;
}

