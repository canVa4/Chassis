#ifndef __vega_action_H
#define __vega_action_H
#ifdef __cplusplus
 extern "C" {
#endif
#include "stm32f4xx_hal.h"
#include "main.h"
#include "usart.h"
#include "stdlib.h"
#include "string.h"
#include "stdarg.h"
#include "usart.h"

void vega_action_setPos(float pos_x, float pos_y);//������������ֵ
void vega_action_setAngle(float angle);//�������ýǶ�ֵ
void vega_action_setAll(float pos_x, float pos_y, float angle);
void vega_action_reset();//������ͽǶȶ�����
void vega_action_init();
	










#ifdef __cplusplus
}
#endif
#endif /*__ vega_action_H */