/*******************************************************************************
Copyright:      2018/12/18
File name:      track.c
Description:    底盘轨迹规划
Author:         王云轩
Version：       1.0
Data:           2018/12/18 22:36
History:        无 
Bug:            NUM_POINTS>5000左右会溢出
*******************************************************************************/
#include "track.h"
#include "stmflash.h"
float modify_x = 0;
float modify_y = 0;
float modify_angle = 0;
int num_line = 140;
int track_max_speed = 1100;
float track_test_offset = 0;

uint8_t track_section_flag = 0;
float track_turn_throw = PI * 45 / 180;//投掷的旋转角
float track_turn_throw_2 = PI * 45 / 180;
float track_turn_throw_3 = PI * 45 / 180;

float track_turn_throw_blue = PI * 45 / 180;//投掷的旋转角
float track_turn_throw_2_blue = PI * 45 / 180;
float track_turn_throw_3_blue = PI * 45 / 180;

void track_goline(int n, float x1, float y1, float x2, float y2)
//参数：分段数 起始点x 起始点y 终止点x 终止点y // n >= 1
{
    param_a = 2 * chassis_speed_max;
    if(n <= 0) n = 1;
    float deltaX = x2 - x1;
    float deltaY = y2 - y1;
    for(int i = 1; i <= n; i++)
    {
    	chassis_xpos[chassis_posnum] = x1 + deltaX * i / n;
    	chassis_ypos[chassis_posnum] = y1 + deltaY * i / n;
        chassis_speed_dir[chassis_posnum] = atan2(y2-y1,x2-x1);
        //chassis_speed[chassis_posnum] = chassis_speed_max;
        chassis_posnum++;
        
    }
    
}

void track_release()
{
    memset(chassis_xpos,0,sizeof(float)*NUM_POINTS);
    memset(chassis_ypos,0,sizeof(float)*NUM_POINTS);
    memset(chassis_speed,0,sizeof(float)*NUM_POINTS);
    chassis_posnum = 0;
    chassis_poscnt = 0;
    chassis.g_ispeed = 0;
    
}

void flag_release(uint8_t Routeflag,uint8_t Sensorflag)
{
    ChassisSignal.m_CtrlFlag._routeflag = Routeflag;
    ChassisSignal.m_CtrlFlag._sensor_flag = Sensorflag;
}

int chassis_winding_and_deliver()
{
    static int state = 0;
    switch(state)
    {
      case 0:
        ChassisSignal.m_DriveMode = _FIXED_TRACK_MODE;
        ChassisSignal.m_SpeedMode = _SPEED_SHEET;
        
           if(chassis_arena == LEFT_ARENA)
           {
            ChassisSignal.m_CtrlFlag._routeflag = 1;
            chassis_init_pos(ORIGIN_X,ORIGIN_Y);
           }
            else if(chassis_arena == RIGHT_AREAN)
            {
              ChassisSignal.m_CtrlFlag._routeflag = 4;
              chassis_init_pos(ORIGIN_X3,ORIGIN_Y3);
            }
        
        
        
        chassis_posnum = NUM_POINTS;//第一段全程曲线点数
        state =3;
        break;
      case 3:
        if(ChassisSignal.m_FinishFlag._position_flag == 1)
        {
            ChassisSignal.m_FinishFlag._position_flag = 0;
            ChassisSignal.m_CtrlFlag._routeflag = 0;
            ChassisSignal.m_CtrlFlag._sensor_flag = 0;
            //uprintf(CMD_USART,"交接！\r\n");
            state = 0;
            return 1;
        }
        break;
    }
    return 0;
}

int chassis_winding_and_deliver_1()
{
    static int state = 0;
    switch(state)
    {
      case 0:
        ChassisSignal.m_DriveMode = _FIXED_TRACK_MODE;
        ChassisSignal.m_SpeedMode = _SPEED_SHEET;
        
           if(chassis_arena == LEFT_ARENA)
           {
            ChassisSignal.m_CtrlFlag._routeflag = 2;
            chassis_init_pos(ORIGIN_X1,ORIGIN_Y1);
           }
            else if(chassis_arena == RIGHT_AREAN)
            {
              ChassisSignal.m_CtrlFlag._routeflag = 5;
              chassis_init_pos(ORIGIN_X4,ORIGIN_Y4);
            }
        
        
        chassis_posnum = NUM_POINTS1;//第一段全程曲线点数
        //vega_action_reset();
        chassis_update();
        
        //uprintf(CMD_USART,"x = %f y = %f ang = %f\r\n",chassis.pos_x,chassis.pos_y,chassis.angle);
        state =3;
        break;
      case 3:
        if(ChassisSignal.m_FinishFlag._position_flag == 1)
        {
            ChassisSignal.m_FinishFlag._position_flag = 0;
            ChassisSignal.m_CtrlFlag._routeflag = 0;
            ChassisSignal.m_CtrlFlag._sensor_flag = 0;
            //uprintf(CMD_USART,"交接！\r\n");
            state = 0;
            return 1;
        }
        break;
    }
    return 0;
}

int chassis_winding_and_deliver_2()
{
    static int state = 0;
    switch(state)
    {
      case 0:
        ChassisSignal.m_DriveMode = _FIXED_TRACK_MODE;
        ChassisSignal.m_SpeedMode = _SPEED_SHEET;
        
           if(chassis_arena == LEFT_ARENA)
           {
            ChassisSignal.m_CtrlFlag._routeflag = 3;
            chassis_init_pos(ORIGIN_X2,ORIGIN_Y2);
           }
            else if(chassis_arena == RIGHT_AREAN)
            {
              ChassisSignal.m_CtrlFlag._routeflag = 6;
              chassis_init_pos(ORIGIN_X5,ORIGIN_Y5);
            }
        
        chassis_posnum = NUM_POINTS2;//第一段全程曲线点数
        //vega_action_reset();
        chassis_update();
        //uprintf(CMD_USART,"x = %f y = %f ang = %f\r\n",chassis.pos_x,chassis.pos_y,chassis.angle);
        state =3;
        break;
      case 3:
        if(ChassisSignal.m_FinishFlag._position_flag == 1)
        {
            ChassisSignal.m_FinishFlag._position_flag = 0;
            ChassisSignal.m_CtrlFlag._routeflag = 0;
            ChassisSignal.m_CtrlFlag._sensor_flag = 0;
            //uprintf(CMD_USART,"交接！\r\n");
            state = 0;
            return 1;
        }
        break;
    }
    return 0;
}
/*
* 修改：5/22 4：21---->交接完后，向前夹兽骨再退后再水平向右走到投掷区前，向后不贴边是因为后轮两个贴边都检测不到了
*/
int chassis_calbration_and_craw()
{
    return 0;
}

/*
* 修改：无
*/
int chassis_throw_thrice_first()
{
    return 0;
}


/*
* 修改：增加upper_flag 3多一个变形
* 待测。。。。
*/
int chassis_throw_thrice_third()
{
    return 0;  
}    

int chassis_throw_thrice_second()
{
    return 0;  
}    