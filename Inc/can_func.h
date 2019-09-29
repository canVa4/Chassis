#ifndef __can_func_H
#define __can_func_H
#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f4xx_hal.h"
#include "main.h"
#include "can.h"
#include "robomaster.h"
#include "chassis.h"
#include "ccd.h"
    
void can_func_init();
void can_motor_rcv(CanRxMsgTypeDef* pRxMsg);
void can_ccd_handle(CanRxMsgTypeDef* pRxMsg);
void can_send_slide_pos(int pos,int flag);
void can_show_screen();

extern float ccd_kp,ccd_kd;
extern int min_ccd;
extern int max_ccd;

    
    
    
#ifdef __cplusplus
}
#endif
#endif /*__ can_func_H */