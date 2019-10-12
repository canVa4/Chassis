#include "robomaster.h"
RoboMaster robomaster[4];
PID_Struct Robomaster_Speed_PID[4];// = {1.4,0.9,0.6,0,0,5000,0,0.005};
PID_Struct Robomaster_Position_PID[4];// = {0.13,0,0.082,0,0,5000,0,0.005};
int robomaster_flag = 0;
int ver_slide_error = 0;

float PID_Release(PID_Struct *PID,float target,float now)
{    
  float err;
  float err_dt;
  float result;
  
  err=target-now; 
  err_dt=err-PID->last_err; 
  
  err_dt*=0.384f;
  err_dt+=PID->last_d*0.615f;   //低通滤波 
  
  PID->last_err=err;
  
  PID->i+=err*PID->I_TIME;
  
  Limit(PID->i,PID->i_max);
  PID->last_d=err_dt;
  
  result = err * PID->KP  +   err_dt * PID->KD   +   PID->i * PID->KI;
  return result;
}

void can_robomaster_rcv(CanRxMsgTypeDef* pRxMsg)
{
    //pRxMsg->StdId
    //这里id还没改
    int id = pRxMsg->StdId - 0x201;
    if(id > 4 || id < 0) return;
    static int first_flag[4] = {1,1,1,1};
    if(first_flag[id] == 1)
    {
        robomaster[id].angle = (uint16_t)(pRxMsg->Data[0]<<8 | pRxMsg->Data[1]);
        robomaster[id].offset_angle = robomaster[id].angle;
        first_flag[id] = 0;
        robomaster[id].round_cnt = 0;
        return ;
    }
    robomaster[id].last_angle = robomaster[id].angle;
    robomaster[id].angle = (uint16_t)(pRxMsg->Data[0]<<8 | pRxMsg->Data[1]);
    robomaster[id].speed_rpm = (int16_t)(pRxMsg->Data[2]<<8 | pRxMsg->Data[3]);
    robomaster[id].real_current = (pRxMsg->Data[4]<<8 | pRxMsg->Data[5])*5.f/16384.f;
    
  if(robomaster[id].angle - robomaster[id].last_angle > 4096)
		robomaster[id].round_cnt --;
	else if (robomaster[id].angle - robomaster[id].last_angle < -4096)
		robomaster[id].round_cnt ++;
	robomaster[id].total_angle = robomaster[id].round_cnt * 8192 + robomaster[id].angle - robomaster[id].offset_angle;
}

void RoboconMaster_Control()
{
    float speed_out[4];
    for(int i = 0; i < 4; i++)
    {
        speed_out[i] = robomaster_pid_control(i);
    }
    robomaster_set_current((int16_t)speed_out[0],(int16_t)speed_out[1],(int16_t)speed_out[2],(int16_t)speed_out[3]);
}

float robomaster_pid_control(int id)
{
    float speed_out=0;
    float position_out=0;
    position_out = PID_Release(&Robomaster_Position_PID[id],
                               (float)robomaster[id].target_position + ver_slide_error,(float)robomaster[id].total_angle);
    if(robomaster[id].type == _M2006)
    {
        //if(position_out > 19000) position_out = 19000;
        //if(position_out < -19000) position_out = -19000;
        if(position_out > 4000) position_out = 4000;
        if(position_out < -10000) position_out = -10000;
    }
    else if(robomaster[id].type == _M3508)
    {
       if(position_out > 9100) position_out = 9100;
       if(position_out < -9100) position_out = -9100; 
        //if(position_out > 2000) position_out = 2000;
        //if(position_out < -2000) position_out = -2000; 
    }
    else 
    {
        position_out = 0;
    }
    speed_out=PID_Release(&Robomaster_Speed_PID[id],position_out,(float)robomaster[id].speed_rpm);
   //speed_out=PID_Release(&Robomaster_Speed_PID[id],(float)robomaster[id].target_speed,(float)robomaster[id].speed_rpm);
    return speed_out;
}

void robomaster_set_current(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
	uint8_t Data[8];
	Data[0] = (iq1 >> 8);
	Data[1] = iq1;
	Data[2] = (iq2 >> 8);
	Data[3] = iq2;
	Data[4] = iq3 >> 8;
	Data[5] = iq3;
	Data[6] = iq4 >> 8;
	Data[7] = iq4;
    /*for(int i = 0; i < 8; i++)
        uprintf("%d ",(int)Data[i]);
    uprintf("\r\n");*/
    can_send_msg(0x200,Data, 8);
}

void reset_PID(PID_Struct * s){
  s->i=0;
  s->last_err=0;
  s->last_d=0;
}

void M2006_init(int id)
{
    reset_PID(&Robomaster_Speed_PID[id]);
          reset_PID(&Robomaster_Position_PID[id]);
          Robomaster_Speed_PID[id].KP = 1.4;
          Robomaster_Speed_PID[id].KI = 0.9;
          Robomaster_Speed_PID[id].KD = 0.6;
          Robomaster_Speed_PID[id].i_max = 5000;
          Robomaster_Speed_PID[id].I_TIME = 0.005;
          
          Robomaster_Position_PID[id].KP = 0.13;
          Robomaster_Position_PID[id].KI = 0;
          Robomaster_Position_PID[id].KD = 0.082;
          Robomaster_Position_PID[id].i_max = 5000;
          Robomaster_Position_PID[id].I_TIME = 0.005;
}

void M3508_init(int id)
{
    reset_PID(&Robomaster_Speed_PID[id]);
          reset_PID(&Robomaster_Position_PID[id]);
          Robomaster_Speed_PID[id].KP = 1.6;
          Robomaster_Speed_PID[id].KI = 0.52;
          Robomaster_Speed_PID[id].KD = 0.6;
          Robomaster_Speed_PID[id].i_max = 5000;
          Robomaster_Speed_PID[id].I_TIME = 0.005;
          
          Robomaster_Position_PID[id].KP = 0.1;
          Robomaster_Position_PID[id].KI = 0;
          Robomaster_Position_PID[id].KD = 0.8;
          Robomaster_Position_PID[id].i_max = 5000;
          Robomaster_Position_PID[id].I_TIME = 0.005;
}

void PID_init()
{
    robomaster[0].type = _M2006;
    robomaster[1].type = _M2006;
    robomaster[2].type = _M2006;
    robomaster[3].type = _M2006;
    for(int i = 0; i < 4; i++)
    {
        switch(robomaster[i].type)
        {
        case _M2006:
            M2006_init(i);
            break;
        case _M3508:
            M3508_init(i);
            break;
        }
    }
}