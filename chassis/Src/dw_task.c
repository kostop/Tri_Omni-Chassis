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
    int16_t value = (highByte << 8) | lowByte; // 组合为16位有符号整数
    return (float)value / scale; // 应用缩放因子
}

// 将浮点数转换为有符号16位整数并拆分为两个字节
void floatToSignedBytes(float value, float scale, uint8_t* highByte, uint8_t* lowByte) {
    int16_t fixedPoint = (int16_t)(value * scale); // 转换为定点数
    *highByte = (fixedPoint >> 8) & 0xFF; // 高字节
    *lowByte = fixedPoint & 0xFF; // 低字节
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
        // 存储当前接收到的字节
        uint8_t current_byte = rx_buffer[0];
        
        switch(rx_state)
        {
            case 0: // 等待帧头 0x6d
                if(current_byte == 0x6d)
                {
                    rx_state = 1;
                    rx_index = 0; // 重置索引
                    rx_data[rx_index++] = current_byte; // 保存帧头
                }
                else
                {
                    rx_state = 0;
                    rx_index = 0;
                }
                break;
                
            case 1: // 等待第二个帧头 0x72
                if(current_byte == 0x72)
                {
                    rx_state = 2;
                    rx_data[rx_index++] = current_byte;
                }
                else
                {
                    rx_state = 0; // 帧头不匹配，重置
                    rx_index = 0;
                }
                break;
                
            case 2: // 等待命令类型
                rx_data[rx_index++] = current_byte;
                if(current_byte == 0x01)
                    rx_state = 3;
                else
                    rx_state = 2; // 继续等待有效命令
                break;
                
            case 3: // 等待命令参数
                rx_data[rx_index++] = current_byte;
                if(current_byte == 0x10)
                    rx_state = 5;
                else
                    rx_state = 2; // 不是预期参数，回到命令类型状态
                break;
                
            case 5: // 接收数据内容
                
            default:
                rx_state = 0;
                rx_index = 0;
                break;
        }
        if(rx_state==5)
		{
			// 检查是否接收完所有数据
			if(rx_index == RX_DATA_LEN)
			{
				// 数据处理
				dw1000_para.state = rx_data[2];
				if(rx_data[2] == CAR_NORMAL_MOVE) // 速度命令
				{
					dw1000_para.car_set_line.vx = parseSignedFloat(rx_data[3], rx_data[4], 100.0f);
					dw1000_para.car_set_line.vy = parseSignedFloat(rx_data[5], rx_data[6], 100.0f);
					dw1000_para.car_set_line.vyaw = parseSignedFloat(rx_data[7], rx_data[8], 100.0f);
				}
				else if(rx_data[2] == CAR_POSITION_MOVE) // 位置命令
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
				
				rx_state = 0; // 重置状态机
				rx_index = 0;
			}
		}
                
        // 重新启动接收中断
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

