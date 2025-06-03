#ifndef _DW_MAIN_H
#define _DW_MAIN_H

#include "cmsis_os.h"
#include "main.h"
#include "first_order_filter.h"

#define TAG_MAX_NUM		7

//typedef __packed struct
//{
//	uint8_t header1;
//	uint8_t header2;
//	uint8_t state;
//	float x;
//	float y;
//	float z;
//	uint8_t ender1;
//	uint8_t ender2;
//}dw1000_rx_data_t;

typedef struct
{
	float x;
	float y;
	float yaw;
}pos_t;

typedef struct
{
	float vx;
	float vy;
	float vyaw;
}line_t;

/*
				^x
		c		|
	oD----------|C2
	|	|		|
	  d|  |e  	|b
		|	| 	|
<-------o-------o
y		B1	a	A0
*/

typedef struct
{
	pos_t anthor_pos[2];			//AB
	pos_t anthor_tag_pos;			//C
	pos_t tag_pos[TAG_MAX_NUM];		//D
}pos_para_t;

typedef struct
{
	float anthor_dis;						//b
	float anthor_tag_dis;					//a
	float tag_dis[TAG_MAX_NUM][3];			//edc
}dis_para_t;

typedef struct
{
	first_order_filter_type_t	pos_filter[2];
//	dis_para_t	dis;
	pos_t	car_pos;
	uint8_t state;
	pos_t car_set_pos;
	line_t car_set_line;
}dw1000_para_t;

void dw_task(void const * argument);
const dw1000_para_t *get_UWB_point(void);

#endif
