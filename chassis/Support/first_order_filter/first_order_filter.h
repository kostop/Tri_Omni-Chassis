#ifndef _FIRST_ORDER_FILTER_H
#define	_FIRST_ORDER_FILTER_H

#include "main.h"

typedef __packed struct
{
    float input;        //��������
    float out;          //�˲����������
    float num[1];       //�˲�����
    float frame_period; //�˲���ʱ���� ��λ s
} first_order_filter_type_t;

void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, float frame_period, const float num);
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, float input);

#endif
