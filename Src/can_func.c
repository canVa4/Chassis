#include "can_func.h"
float ccd_kp = 12,ccd_kd = 50;
int min_ccd = 40;
int max_ccd = 60;


void can_func_init()
{
    can_add_callback(recive_id.handle_button_id,chassis_handle);
    can_add_callback(recive_id.handle_rocker_id,chassis_rocker);
    can_add_callback(recive_id.ccd_id,can_ccd_handle);
    can_add_callback(0x201, can_robomaster_rcv);
    can_add_callback(0x202, can_robomaster_rcv);
    can_add_callback(0x203, can_robomaster_rcv);
    can_add_callback(0x204, can_robomaster_rcv);
    //can_add_callback(410, can_motor_rcv);
    //can_add_callback(411, can_motor_rcv);
    //can_add_callback(412, can_motor_rcv);
}

void can_motor_rcv(CanRxMsgTypeDef* pRxMsg)
{
    int id = pRxMsg->StdId - 410;
    if(id > 2 || id < 0) return;
    int i;
    for(i=0;i<8;i++)
    {
        can_RX_data.ui8[i]=pRxMsg->Data[i];
    }
    //chassis.now_speed[id] = can_RX_data.in[0];
}

float ccd_pid(uint16_t p,float kp,float kd)
{
    static int16_t last;
    if(ChassisSignal.m_CtrlFlag._sensor_flag != 3)
    {
        last = 0;
        return 0;
    }
    if(p>=min_ccd&&p<=max_ccd)
        return 0;
    
    int16_t error = 0;
    if(p>max_ccd)
      error = p - max_ccd;
    else if(p<min_ccd)
      error = p - min_ccd;
    
    float pwm = kp * error + kd * (error - last);
    
    last = error;
    return pwm;
}

uint16_t ccd_pos = 0;
void can_ccd_handle(CanRxMsgTypeDef* pRxMsg)
{
     for(int i=0;i<8;i++)
    {
        can_RX_data.ui8[i]=pRxMsg->Data[i];
    }
    ccd_pos = can_RX_data.ui16[0];
    chassis.ccd_pos = (int)ccd_pid(ccd_pos,ccd_kp,ccd_kd);
    //uprintf(CMD_USART,"ccd_pos = %d\r\n",can_RX_data.ui16[0]);
}

void can_send_slide_pos(int pos,int flag)//0åŠ¨
{
    can_TX_data.in[0] = pos;
    can_TX_data.in[1] = flag;
    can_send_msg(460, can_TX_data.ui8, 8);
    can_send_msg(460, can_TX_data.ui8, 8);
    can_send_msg(460, can_TX_data.ui8, 8);
}

void can_show_screen()
{
    can_TX_data.i16[0] = (int16_t)(chassis.pos_x * 1000);
    can_TX_data.i16[1] = (int16_t)(chassis.pos_y * 1000);
    can_TX_data.i16[2] = (int16_t)(chassis.angle * 180 / PI);
    can_TX_data.i16[3] = (int16_t)(ccd_pos* 1000);
    can_send_msg(send_id.show_screen_id, can_TX_data.ui8, 8);
    
}