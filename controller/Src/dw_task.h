#ifndef _DW_MAIN_H
#define _DW_MAIN_H

#include "cmsis_os.h"
#include "main.h"
#include "first_order_filter.h"
#include "INS_task.h"

#define TAG_MAX_NUM		3


typedef __packed struct
{
	uint8_t header1;
	uint8_t header2;
	uint8_t header3;
	uint8_t tag_id;
	uint16_t frame_num;
	uint16_t distance1;
	uint16_t distance2;
	uint16_t distance3;
	uint16_t distance4;
	uint16_t reserve;
	uint8_t ender1;
	uint8_t ender2;
}dw1000_rx_data_t;

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
	oD------oC2	|
	|	|	|	|
	  d|  |e  |b|
		|	|  ||
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

typedef __packed struct
{
	uint8_t header1;
	uint8_t header2;
	uint8_t id;
	uint8_t state;
	float x;
	float y;
	float yaw;
	uint8_t ender;
}dw1000_tx_t;

typedef struct
{
	uint8_t 	state;
	pos_t		pos;
	pos_t		tar_pos;
	line_t		line;
	line_t		tar_line;
}car_para_t;

typedef struct
{
	const INS_para_t *dw1000_INS_point;
	dw1000_tx_t dw1000_tx;
	car_para_t	car_para[TAG_MAX_NUM];
}dw1000_para_t;


void dw_task(void const * argument);
const dw1000_para_t *get_pos_point(void);

void car_set_no_move(uint8_t id);
void car_set_pos(uint8_t id, pos_t car_pos);
void car_set_line(uint8_t id, line_t car_line);
void car_start_calibrate(uint8_t id);
void car_stop_calibrate(uint8_t id);



#endif
