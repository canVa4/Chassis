#include "vavle.h"
#include "robomaster.h"
int upper_flag = 0;
int throw_finish_flag = 0;
int after_throw_continue_track_flag = 0;
int deliver_blue = 5030;
int deliver_red = 13300;
int deliver_mid = 9190;


void upper_back_to_place()//没安微动不要用这个初始化//阻塞函数，建议放在初始化
{
    can_send_slide_pos(370,3);
    /*while(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_5) != 1);   
    can_send_slide_pos(370,2);//清零*/
        
    while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) != 1)
    {
        robomaster_set_current(500,0,0,0);
    }
    robomaster_set_current(0,0,0,0);
    ver_slide_error = robomaster[0].total_angle;
}


void upper_init()
{
        E13_LOW;
        E14_LOW;
        E15_LOW;
        B12_LOW;
        B15_LOW;
        upper_back_to_place();
}

void deliver_rotate()
{
}

void deliver_back()
{
    send_u16(deliver_mid,send_id.giver_id);
}

void craw_close()
{
    E15_HIGH;
}

void craw_open()
{
    E15_LOW;
}

void upper_throw()
{
    E14_HIGH;
}
    
void upper_back()
{
    E14_LOW;
}

void bracket_out()
{
    E13_HIGH;
}

void bracket_in()
{
    E13_LOW;
}

void lift_craw()
{
    B12_HIGH;
}
    
void putback_craw()
{
    B12_LOW;
}

/*
* 修改：5/22 4：06---->夹取后直接抬起来，延时抬第二次投掷容易撞，去掉了upper_flag = 2
* 修改：5/22 4：06---->upper_flag 9 10之间加上在投掷完成计数，衔接走第二次投掷 第三次投掷
* 修改：5/22 4：09---->upper_flag 10单独拿出来，为了第二第三次夹兽骨边移动边复位投掷装置
*/
void upper_thread()
{
    
    static int delay_time = 0;
    static int clap_cnt = 0;
    switch(upper_flag)
    {
      case 0:
        break;
      case 1:
        craw_close();
        robomaster[0].target_position = -700000;
        throw_finish_flag = 0;
        upper_flag = 0;
        break;
      case 3:
        delay_time = 100;
        can_send_slide_pos(-200,0);
        upper_flag = 4;
        can_send_slide_pos(-200,0);
        break;
      case 4:
        delay_time--;
        if(delay_time <= 0)
        {
            delay_time = 0;
            robomaster[0].target_position = -1500000;
            upper_flag = 5;
        }
        break;
      case 5:
        if(ChassisSignal.m_CtrlFlag._press_flag._front_slide_up_press == _PRESSED)
        {
            robomaster[0].target_position = robomaster[0].total_angle - ver_slide_error;
            bracket_out();
            delay_time = 100;
            upper_flag = 500;
        }    
        break;
      case 500:
        delay_time--;
        if(delay_time <= 0)
        {
            delay_time = 0;
            lift_craw();
            upper_flag = 0;
        }
        break;
      case 6:
        can_send_slide_pos(-370,0);
        upper_flag = 0;
        can_send_slide_pos(-370,0);
        break;
    /*
      case 7:
        if(ChassisSignal.m_CtrlFlag._press_flag._up_slide_press == _PRESSED)
        {
            can_send_slide_pos(-370,1);
            upper_flag = 0; 
            can_send_slide_pos(-370,1);
        }
        break;
*/
      case 8:
        craw_open();
        delay_time = 100;
        upper_flag = 9;
        break;
      case 9:
        delay_time--;
        if(delay_time <= 0)
        {
            delay_time = 100;            
            upper_throw();
            upper_flag = 1000;
        }
        break;
      case 1000:
         delay_time--;
        if(delay_time <= 0)
        {
            static int throw_cnt = 0;
            throw_cnt++;
            delay_time = 0;
            if(after_throw_continue_track_flag == 1)
            {
                upper_flag = 0;
                Chassis_State = 3 + throw_cnt;
            }
            else
            {
                upper_flag = 10;
            }
        }
        break;
      case 10:
        delay_time--;
        if(delay_time <= 0)
        {
            delay_time = 200;
            throw_finish_flag = 1;
            upper_back();
            bracket_in();
            putback_craw();
            upper_flag = 11;
        }
        break;
      case 11:
        delay_time--;//if delay time arrived, stretch out the arm
        if(delay_time <= 0)
        {
            delay_time = 0; 
            robomaster[0].target_position = 0;
            can_send_slide_pos(20,0);
            upper_flag = 12;
            can_send_slide_pos(20,0);
        }
        break;
      /*case 12:
        if(ChassisSignal.m_CtrlFlag._press_flag._front_slide_down_press == _PRESSED && both_flag1 != 1 )
        {
            robomaster[0].target_position = robomaster[0].total_angle - ver_slide_error;
            both_flag1 = 1;
            robomaster[0].target_position = robomaster[0].total_angle - ver_slide_error;
        }    
        if(ChassisSignal.m_CtrlFlag._press_flag._down_slide_press == _PRESSED && both_flag2 != 1)
        {
            can_send_slide_pos(0,1);
            both_flag2 = 1;
        }
        if(both_flag1 == 1 && both_flag2 == 1)
        {
            both_flag1 = 0;
            both_flag2 = 0;
            upper_flag = 0;
        }
        break;*/
      case 12:
        if(ChassisSignal.m_CtrlFlag._press_flag._front_slide_down_press == _PRESSED)
        {
            robomaster[0].target_position = robomaster[0].total_angle - ver_slide_error;
            upper_flag = 0;
        }
        break;
      case 100://拍巴掌
        delay_time--;//if delay time arrived, stretch out the arm
        if(delay_time <= 0)
        {
            delay_time = 10;
            E15_LOW;
            clap_cnt++;
            upper_flag = 101;
        }
        break;
      case 101:
        delay_time--;//if delay time arrived, stretch out the arm
        if(delay_time <= 0)
        {
            delay_time = 10;
            E15_HIGH;
            upper_flag = 100;
            if(clap_cnt >= 5)
            {
                upper_flag = 9;
                craw_open();
                delay_time = 0;
                clap_cnt = 0;
            }
        }
        break;
        
        
      case 200://投掷一次后收回，结构不还原
        upper_throw();
        delay_time = 60;
        upper_flag = 201;
        break;
      case 201:
        delay_time--;
        if(delay_time <= 0)
        {     
            upper_back();
            upper_flag = 0;
        }
        break;
        
    case 300:
        delay_time = 200;
        upper_flag = 301;
      case 301://扶一下兽骨
          delay_time--;
          if(delay_time <= 0)
          {
          delay_time = 200;
          craw_open();
          upper_flag = 302;
          }
        break;
      case 302:
          delay_time--;
          if(delay_time <= 0)
          {
              craw_close();
              delay_time = 0;
              upper_flag = 0;
          }
          break;
      case 305:
        delay_time = 250;
        deliver_rotate();
        upper_flag = 306;
        break;
      case 306:
        delay_time--;
          if(delay_time <= 0)
          {
              delay_time = 0;
              upper_flag = 0;
              B15_HIGH;
          }
          
  
    }
}
