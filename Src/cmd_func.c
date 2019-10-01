/*******************************************************************************
Copyright:      2018/12/18
File name:      cmd_func.c
Description:    存放串口命令函数，用户自己添加，要求设成外部可以调用
Author:         徐铭远
Version：       1.0
Data:           2018/12/18 22:36
History:        无
Bug:            无
*******************************************************************************/
#include "cmd_func.h"


void cmd_hello_func(int argc,char *argv[])
{
  uprintf(CMD_USART,"hello world");
  ChassisSignal.m_CtrlFlag._handle_flag = 0;
  //modify_x = ORIGIN_X - (chassis.pos_x - modify_x);//修改全场定位初始值
  //modify_y = ORIGIN_Y - (chassis.pos_y - modify_y);
  chassis.g_fturn = 0;//防止方向环突变 
  chassis_update();
  //chassis_modify_pos(chassis_xpos,chassis_ypos,ORIGIN_X,ORIGIN_Y);
  chassis_poscnt = 0;
  Chassis_State = atoi(argv[1]);
}

void cmd_go_straight_func(int argc,char *argv[])//go_straight 1000 30
{
  //chassis_gostraight( atoi(argv[1]) , atof(argv[2]) ,atof(argv[3]));
  chassis.g_ispeed = atoi(argv[1]);
  chassis.g_fangle = (atof(argv[2])/180)*PI;
  //g_fturn = atof(argv[3]);
}

void cmd_reset_vega(int argc, char *argv[])
{
  vega_action_reset();
  //uprintf(ACTION_USART,"ACTR");
  //vega_action_setAngle(0);
  uprintf(CMD_USART,"复位：x = %f   y = %f   angle = %f\r\n",chassis.g_vega_pos_x,chassis.g_vega_pos_y,chassis.g_vega_angle);
}


void cmd_angle_pid(int argc, char *argv[])
{
  chassis_turn_angle_KP = atof(argv[1]);
  chassis_turn_angle_KD = atof(argv[2]);
  
}

void cmd_print_pos(int argc, char *argv[])
{
  uprintf(CMD_USART,"x = %f, y = %f,  angle = %f\r\n",chassis.pos_x,chassis.pos_y,chassis.angle);
  //uprintf(CMD_USART,"复位：x = %f   y = %f   angle = %f\r\n",chassis.g_vega_pos_x,chassis.g_vega_pos_y,chassis.g_vega_angle);
}

void cmd_debug_func(int argc, char *argv[])
{
  int input = atoi(argv[1]);
  switch(input)
  {
  case 1:
    Chassis_State = 1;
    break;
  case 3:
    Chassis_State = 3;
    action_init_posx = 3.87;
    action_init_posy = -8.44;
    //action_init_angle = -PI * 3/4;
    chassis.g_fturn = -PI/4;
    break;
  }
  
}

void send_wang_can(int id,int num,char a)
{
  char Data[6] = {0};
  Data[0] = a;
  if(num < 1) Data[1] = '1';
  else Data[1] = '0';
  num = abs(num);
  Data[2] = num / 1000 + '0';
  num %= 1000;
  Data[3] = num / 100 + '0';
  num %= 100;
  Data[4] = num / 10 + '0';
  num %= 10;
  Data[5] = num / 1 + '0';
  can_send_msg(id, (uint8_t *)Data, 6);
}

void cmd_param_func(int argc, char *argv[])
{
  TIM1->CCR1 = atoi(argv[1]);
  TIM1->CCR2 = atoi(argv[2]);
  uprintf(CMD_USART,"1 = %d 2 = %d\r\n",atoi(argv[1]),atoi(argv[2]));
  //chassis_speed_max = atof(argv[3]);
  //    ccd_kp = atof(argv[1]);
  //    ccd_kd = atof(argv[2]);
  //    min_ccd = atoi(argv[3]);
  //    max_ccd = atoi(argv[4]);
  //upper_flag = atoi(argv[1]);
  
}

void cmd_robomaster_position(int argc, char *argv[])//第1234个参数为robomaster电调的id，看对应电调闪绿灯频率可知id
{
  for(int i = 0; i < 4; i++)
  {
    robomaster[i].target_position = atoi(argv[i + 1]);
    uprintf(CMD_USART,"target_position%d  = %d\r\n",i,atoi(argv[i + 1]));
  }
}

void cmd_r_sp_pid(int argc, char *argv[])
{
  //Robomaster_Speed_PID[0].KP = atof(argv[1]);
  //Robomaster_Speed_PID[0].KI = atof(argv[2]);
  //Robomaster_Speed_PID[0].KD = atof(argv[3]);
  
  //Robomaster_Position_PID[0].KP = atof(argv[1]);
  //Robomaster_Position_PID[0].KI = atof(argv[2]);
  //Robomaster_Position_PID[0].KD = atof(argv[3]);
}

void cmd_r_speed(int argc, char *argv[])
{
  //robomaster[0].target_position = atoi(argv[1]);
}

void cmd_valve_open(int argc, char *argv[])
{
  int i = atoi(argv[1]);
  medical_take_off = i;
}

void cmd_upper_func(int argc, char *argv[])
{
  int i = atoi(argv[1]);
  if((i >= 1 && i <= 3) || i == 6)
  {
    upper_flag = i;
  }
}

void cmd_chassis_pid(int argc, char *argv[])
{
  position_y_dir_pid.KP = atof(argv[1]);
  position_y_dir_pid.KD = atof(argv[2]);
  speed_x_dir_pid.KP = atof(argv[3]);
  speed_x_dir_pid.KI = atof(argv[4]);
  chassis.g_ispeed = atoi(argv[5]);
  //test_target_speed = atof(argv[5]);
  //uprintf(CMD_USART,"chassis_kp_kd = %f %f\r\n",chassis_PID.KP,chassis_PID.KD);
}

void cmd_slide_pos(int argc,char *argv[])
{
  if(atoi(argv[2]) == 0)
  {
    uprintf(CMD_USART,"move %d\r\n",atoi(argv[1]));
  }else
  {
    uprintf(CMD_USART,"stop %d\r\n",atoi(argv[2]));
  }
  can_send_slide_pos(atoi(argv[1]),atoi(argv[2]));
  
}

void cmd_open_box1(int argc,char *argv[])
{
  open_box = 1;
}

void cmd_chassis_gostraight(int argc,char *argv[]){
  uprintf(CMD_USART,"speed = %d angle = %f self_angle = %f\r\n", atoi(argv[1]), atoi(argv[2]) , atof(argv[3]));
  chassis_gostraight(atoi(argv[1]) , atof(argv[2]) , atof(argv[3])  , 0 );
}

void cmd_print_angle(int argc,char *argv[]){
  uprintf(CMD_USART,"angle = %f\r\n",chassis.angle);
}

void cmd_test_angle(int argc,char *argv[]){
  float target_angle = atof(argv[1]);
  uprintf(CMD_USART,"angle = %f\r\n",target_angle);
  while( fabs(chassis.angle - target_angle) >= 0.002 ){
  chassis_gostraight(0,0, atof(argv[1]) , atoi(argv[2]) );  
  }
}

void cmd_start_point_tracer(int argc,char *argv[]){
  float angle = point_tracer_angle_return( atof(argv[1]) , atof(argv[2]) );
  uprintf(CMD_USART , "point_tracer angle : %f \r\n", angle);
}

float point_x,point_y;
void stop_flag(int argc,char *argv[]){
  point_tracer_flag = atoi(argv[1]);
  first_time_controler = 1;
  if(point_tracer_flag == 0){
    chassis_gostraight(0,0,chassis.angle,0);
  }
  //point_x = atof(argv[2]);
  //point_y = atof(argv[3]);
  uprintf(CMD_USART , " stop_flag : %d \r\n" , point_tracer_flag);
  //uprintf(CMD_USART , "point_tracer : tracing %f,%f now \r\n", atof(argv[2]) , atof(argv[3]) );

}

int start_speed = 0 , final_speed = 0 , max_speed = 10;

void cmd_point_tracer(int argc,char *argv[]){
  uprintf(CMD_USART , "point_tracer : tracing %f,%f now!\r\n", atof(argv[1]) , atof(argv[2]) );
  start_point_x = atof(argv[1]);
  start_point_y = atof(argv[2]);
  point_x = atof(argv[3]);
  point_y = atof(argv[4]);
  start_speed = atoi(argv[5]);
  final_speed = atoi(argv[6]);
  max_speed = atoi(argv[7]);
  uprintf(CMD_USART,"start_speed : %d, final_speed : %d, max_speed : %d\r\n",start_speed, final_speed,max_speed);
}

void cmd_vega_init(int argc,char *argv[]){
  chassis_init();
  uprintf(CMD_USART , "vega : init\r\n");
}

void cmd_point_collection_tracer(int argc,char *argv[]){
  ENBALE_POINT_COLLECTION_TRACER = atoi(argv[1]);
  if(ENBALE_POINT_COLLECTION_TRACER == 0){
    chassis_gostraight(0,0,chassis.angle,0);
  }
}

void cmd_set_boost_slow(int argc,char *argv[]){
  Boost_Period = atof( argv[1] );
  Slow_Period = atof(argv[2]);
  uprintf(CMD_USART,"set_boost_slow : boost_period = %f , slow = %f\r\n",Boost_Period, Slow_Period);
}

float line_control_p_x,line_control_p_y,start_point_x,start_point_y;
void cmd_line_control_test(int argc,char *argv[]){
  line_control_flag_first = 1;
  total_line_control_flag = atoi(argv[1]);
  start_point_x = atof(argv[2]);
  start_point_y = atof(argv[3]);
  line_control_p_x = atof(argv[4]);
  line_control_p_y = atof(argv[5]);
}

void cmd_line_control_PID(int argc,char *argv[]){
  line_control_PID.KP = atof(argv[1]);
  line_control_PID.KI = atof(argv[2]);
  line_control_PID.KD = atof(argv[3]);
  uprintf(CMD_USART,"KP: %f , KI: %f , KD: %f\r\n" ,line_control_PID.KP , line_control_PID.KI , line_control_PID.KD);
}