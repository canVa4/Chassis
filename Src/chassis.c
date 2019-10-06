#include "chassis.h"
#include "stmflash.h"

/***********更换红蓝场*********************/
int chassis_arena = LEFT_ARENA;
/***********here*********************/
Chassis chassis;
CHASSIS_SIGNAL ChassisSignal;
CHASSIS_HANDLE ChassisHandle;
//三轮与全场定位模块安装偏角
float catch_bone_modify_x = 0;
float catch_bone_modify_y = 0;
float ERR_angle_m3 = -PI/3 , ERR_angle_m1 = -PI/3 + 1*2*PI/3 , ERR_angle_m0 = -PI/3 + 2*2*PI/3 ;
int chassis_poscnt = 0;//点计数
//曲线开始标志位
float param_a = 4000, param_b = 3.5;  //a越大，加速上限越大， b越大，加速越慢 
//曲线中某一个点距离开头和结尾的距离数组
float chassis_dis_to_bgn[NUM_POINTS];
float chassis_dis_to_end[NUM_POINTS];

//底盘自转pid参数
float chassis_turn_angle_KP = -1000;
float chassis_turn_angle_KD = 0;

int open_box = 1;
int move_speed = 0;


PID_Struct chassis_PID = {0,3,0,0,0,5000,0,0.005};
PID_Struct speed_x_dir_pid = {0,0,0,0,0,5000,0,0.005};
//PID_Struct position_y_dir_pid = {2000,90000,0,0,0,5000,0,0.005};
PID_Struct position_y_dir_pid = {3000,175000,0,0,0,5000,0,0.005};
//{1200,0,0,0,0,5000,0,0.005};/
float test_target_speed = 0;

int scan_wave = 0;

float scope_path = 0.06;

float slow_turn_angle = PI/6;

float chassis_speed_max = 500;//手柄最大速度
int chassis_posnum = 1;

int rocker_x = 0;
int rocker_y = 0;
int rocker_z = 0;

float motor0_error = 0;
float motor1_error = 0;
float motor2_error = 0;


float slow_turn_const = 0.01;
float slow = 0.01;
float very_slow = 0.001;
float slow_turn = 0;
int delay_num_const = 0;
int Chassis_State = 0;

int abs_slow_turn_flag = 0;
float handle_turn = 0;
double chassis_vector_d = 0;
double chassis_vector_vx = 0;

extern int main_flag;
int micro_switch_test_flag = 0;
//int track_finish_flag = 0;
//int four_feet_ok = 0;
/**底盘初始化
*参数：无
*返回值： 无
*/
void chassis_zero()
{
    chassis.g_fangle = 0;
    chassis.g_fturn = 0;
    chassis.g_ispeed = 0;
    chassis.origin_angle = 0;
    ChassisSignal.m_DriveMode = _FIXED_TRACK_MODE;
    ChassisSignal.m_SpeedMode = _SPEED_SHEET;
    ChassisSignal.m_CtrlFlag._vega_ready_flag = 0;
    ChassisSignal.m_FinishFlag._delay_flag = 0;
    ChassisSignal.m_FinishFlag._position_flag = 0;
    ChassisSignal.m_FinishFlag._rotate_flag = 0;
    ChassisSignal.m_CtrlFlag._routeflag = 0;
    ChassisSignal.m_CtrlFlag._sensor_flag = 0;
    ChassisSignal.m_CtrlFlag._handle_flag = 0;
}

void chassis_init(void)
{
    chassis_init_pos(zx_points_pos_x[0],zx_points_pos_y[0]);
    chassis_zero();
    /*
    ChassisHandle.mode = 2;
    for(int i = 0; i < 10; i++)
    {
        ChassisHandle.btstate[i] = 0;
    }
    for(int i = 0; i < NUM_POINTS; i++)
    {
      if(i <= 100)
      {
        chassis_speed[i] = 100 + i * 9;
        if(chassis_speed[i] >= 900)
          chassis_speed[i] = 900;
        
      }
      else if(i<170&&i>100)
      {
        chassis_speed[i] = chassis_speed[100];
      }
      else if(i >= 170 && i <180)
      {
        chassis_speed[i] = 600;
      }
      else if(i >= 180 && i <190)
      {
        chassis_speed[i] = 400;
      }
      else if(i >= 190 && i <201)
      {
        chassis_speed[i] = 200;
      }
      
      
      if(chassis_speed[i] <= 200)
         chassis_speed[i] = 200;
      
    }
    for(int i = 0; i < NUM_POINTS1; i++)
    {
      if(i <= 50)
        chassis_speed1[i] = 200 + i * 6;//600
      if(i > 50 && i <=350)
        chassis_speed1[i] = chassis_speed1[50];//600
      if(i>350&&i<=400)
        chassis_speed1[i] = chassis_speed1[50] - (i-350)* 6;//200

    }
    for(int i = 0; i < NUM_POINTS2; i++)
    {
      if(i <= 50)
      { 
        chassis_speed2[i] = 100 + i * 16;//900
        if(chassis_speed[i] >= 900)
          chassis_speed[i] = 900;      
      }
      if(i>50&&i<=130)
      {
        chassis_speed2[i] = chassis_speed2[50] - (i-50)*6;//600
        if(chassis_speed2[i] <= 600)
          chassis_speed2[i] = 600;
      }
      if(i>100)
        chassis_speed2[i] = chassis_speed2[100] - (i - 100)*4;
      
      if(chassis_speed2[i]<= 200)
        chassis_speed2[i] = 200;

    }
    //uprintf(CMD_USART,"chassis and vega are ready!!!\r\n");
    */
}

//计算距离
void chassis_calculate_dis_matrix()
{
    double lastdis = 0;
    chassis_dis_to_bgn[0] = 0;
    for(int i = 1; i < chassis_posnum; i++)
    {
        double deltaX = chassis_xpos[i] - chassis_xpos[i - 1];
        double deltaY = chassis_ypos[i] - chassis_ypos[i - 1];
        double distance = sqrt(deltaX * deltaX + deltaY * deltaY) + lastdis;
        chassis_dis_to_bgn[i] = distance;
        lastdis = chassis_dis_to_bgn[i];
    }
    
    
    lastdis = 0;
    chassis_dis_to_end[chassis_posnum - 1] = 0;
    for(int i = chassis_posnum - 2; i >= 0; i--)
    {
        double deltaX = chassis_xpos[i] - chassis_xpos[i + 1];
        double deltaY = chassis_ypos[i] - chassis_ypos[i + 1];
        chassis_dis_to_end[i] = sqrt(deltaX * deltaX + deltaY * deltaY) + lastdis;
        lastdis = chassis_dis_to_end[i];
    }
}
/*速度曲线计算
传入到下一个点的实际距离
返回此刻速度值
*/
int chassis_calculate_speed(double dis_to_nextpos)
{
    double x1 = chassis_dis_to_bgn[chassis_poscnt] - dis_to_nextpos;
    double x2 = chassis_dis_to_end[chassis_poscnt] + dis_to_nextpos;
    if(x1<0)
        x1 *= -1;
    else if(x2<0)
        x2 *=-1;
    double speed = 0;
    if(x1 <= x2)
    {
        speed = (param_a / 2) * pow(1 - pow(EXP, (-2 * x1 / param_b)), 0.5) + 300;
        if(speed >= chassis_speed_max)
        {
            speed = chassis_speed_max;
        }
    }
    else 
    {
        speed = (param_a / 2) * pow(1 - pow(EXP, (-2 * x2 / param_b)), 0.5);
        if(speed >= chassis_speed_max)
        {
            speed = chassis_speed_max;
        }
    }
    return (int)speed;
}

/**底盘重设坐标
*参数：无
*返回值： 无
*/
float action_init_posx = 0;
float action_init_posy = 0;
float action_init_angle = 0;
void chassis_init_pos(float x,float y)
{
    action_init_posx += x - chassis.pos_x;
    action_init_posy += y - chassis.pos_y;
}

/**底盘更新坐标
*参数：无
*返回值： 无
*/
void chassis_update(void)
{ 
    chassis.angle = (chassis.g_vega_angle / 180.f) * PI;
    chassis.pos_x = chassis.g_vega_pos_x/1000 + action_init_posx;
    chassis.pos_y = chassis.g_vega_pos_y/1000 + action_init_posy;
    chassis.speed_x = (chassis.pos_x - chassis.last_pos_x) / 0.005;
    chassis.speed_y = (chassis.pos_y - chassis.last_pos_y) / 0.005;//除时间 m/s
    chassis.last_pos_x = chassis.pos_x;
    chassis.last_pos_y = chassis.pos_y;
    chassis.now_speed = vec_model(vec_create(chassis.speed_x,chassis.speed_y));
}

/*
角度减法函数
*/
float chassis_angle_subtract(float a, float b)
{
    float out = a - b;
    while(out > PI)
    {
        out -= 2 * PI;
    }
    while(out < - PI)
    {
        out += 2 * PI;
    }
    return out;
}

/*
角度pid控制
*/
float chassis_PID_Angle_Control(float target_angle){
    
    float angle_err = chassis_angle_subtract(target_angle, chassis.angle); 
    static float angle_last_err = 0;
    float P_out = angle_err * chassis_turn_angle_KP;
    float D_out = (angle_last_err - angle_err) * chassis_turn_angle_KD;
    angle_last_err = angle_err;
    return P_out + D_out;
}

/**底盘驱动
*参数：float angle 	方向角
*      int   speed    速度
float turn  自转方位角
*返回值： 无
*说明:
*/
void chassis_gostraight(int speed , float angle, float turn, int is_handle)
{
    float Chassis_motor0 = -(speed*cos((ERR_angle_m0 + chassis.angle) - angle));
    float Chassis_motor1 = -(speed*cos((ERR_angle_m1 + chassis.angle) - angle));
    float Chassis_motor3 = -(speed*cos((ERR_angle_m3 + chassis.angle) - angle));
    
    float turn_output = 0;
    if(is_handle)
    {
        turn_output = -turn;//全场定位方向环  
    }
    else
    {
        turn_output = chassis_PID_Angle_Control(turn);
    }
    if(turn_output >200)//陀螺仪角度PID
    {
        turn_output = 200;
    }
    if(turn_output < -200)
    {
        turn_output = -200;
    }
    
    
    maxon_canset3speed((int)(Chassis_motor0 + turn_output),
                       (int)(Chassis_motor3 + turn_output),
                       (int)(Chassis_motor1 + turn_output));
}


void chassis_gostraight_zx(int speed , float angle, float turn, int is_handle)
{
    float Chassis_motor0 = -(speed*cos((ERR_angle_m0 + chassis.angle) - angle));
    float Chassis_motor1 = -(speed*cos((ERR_angle_m1 + chassis.angle) - angle));
    float Chassis_motor3 = -(speed*cos((ERR_angle_m3 + chassis.angle) - angle));
    
    float turn_output = 0;
    if(is_handle)
    {
        turn_output = -turn;//全场定位方向环  
    }
    else
    {
        turn_output = chassis_PID_Angle_Control(turn);
    }
    if(turn_output >350)//陀螺仪角度PID
    {
        turn_output = 350;
    }
    if(turn_output < -350)
    {
        turn_output = -350;
    }
    
    
    maxon_canset3speed((int)(Chassis_motor0 + turn_output),
                       (int)(Chassis_motor3 + turn_output),
                       (int)(Chassis_motor1 + turn_output));
}


void slow_turn_func()
{
    static float last_slow_turn = 0;
    if(fabs(slow_turn - 0) > 1e-5)
    {
        if(slow_turn > 0)
        {
            if(slow_turn > slow_turn_const)
            {
                chassis.g_fturn  += slow_turn_const;
                slow_turn -= slow_turn_const;
            }
            else 
            {
                chassis.g_fturn += slow_turn;
                slow_turn = 0;
            }
        }
        else
        {
            if(slow_turn < -slow_turn_const)
            {
                chassis.g_fturn  -= slow_turn_const;
                slow_turn += slow_turn_const;
            }
            else 
            {
                chassis.g_fturn -= slow_turn;
                slow_turn = 0;
            }
        }
    }
    if(fabs(last_slow_turn - 0) > 1e-5 && fabs(slow_turn - 0) <= 1e-5)
    {
        ChassisSignal.m_FinishFlag._rotate_flag = 1;
    }
    last_slow_turn = slow_turn;
}

void absolute_slow_turn_thread()
{
    if(abs_slow_turn_flag == 0) return;
    if(fabs(chassis_angle_subtract(chassis.g_fturn, handle_turn)) <= 1e-5)
    {
        abs_slow_turn_flag = 0;
    }
    else if(chassis_angle_subtract(chassis.g_fturn, handle_turn) > slow_turn_const)
    {
        chassis.g_fturn = chassis_angle_subtract(chassis.g_fturn, slow_turn_const);
    }
    else if(chassis_angle_subtract(chassis.g_fturn, handle_turn) < -slow_turn_const)
    {
        chassis.g_fturn = chassis_angle_subtract(chassis.g_fturn, -slow_turn_const);
    }
    else
    {
        chassis.g_fturn = handle_turn;
    }
}

void vector_track_ctrl(vec now, vec target, vec direct)
{
    if(vec_is_zero(target))return;
    chassis_vector_d = vec_mul(direct,vec_normal(target));
    chassis_vector_vx = vec_mul(now,vec_unit(target));
    double vx_output = vec_model(target);
    double vy_output = -PID_Release(&position_y_dir_pid,0,chassis_vector_d);
    vec output = vec_add(vec_mul_i(vec_unit(target),vx_output), vec_mul_i(vec_normal(target),vy_output));
    
    chassis.g_ispeed = (int)vec_model(output);
    if(chassis.g_ispeed > 2100) chassis.g_ispeed = 2100;
    if(chassis.g_ispeed < -2100) chassis.g_ispeed = -2100;
    chassis.g_fangle = atan2(output.y,output.x);	
}

void chassis_vector_test()
{
    chassis.g_fturn = 0;
    
    vec direct = vec_create(0 - chassis.pos_x,-1.5 - chassis.pos_y);
    vec target = vec_create(500,-500);
    vec now = vec_create(chassis.speed_x,chassis.speed_y);
    vector_track_ctrl(now, target, direct);
    //chassis.g_fangle = -PI/2;
    //chassis.g_ispeed = (int)PID_Release(&speed_x_dir_pid,test_target_speed,-chassis.speed_y);
}

void chassis_go_track(CHASSIS_DRIVE_MODE theMode, CHASSIS_SPEED_MODE theSpeedMode)
{
    double dis_to_next = 0;
    int prenum = chassis_posnum - chassis_poscnt;
    if(prenum > 10) prenum = 10;//此处更改预先判断的点的最大数量
    int i;
    for(i = prenum - 1; i >= 0; i--)//从远往近判断
    {
        double deltaX = 0, deltaY = 0;
          if(ChassisSignal.m_CtrlFlag._routeflag == 1)
          {
            deltaX = chassis.pos_x - chassis_xpos[chassis_poscnt + i];
            deltaY = chassis.pos_y - chassis_ypos[chassis_poscnt + i]; 
          }else if(ChassisSignal.m_CtrlFlag._routeflag == 2)
          {
            deltaX = chassis.pos_x - chassis_xpos1[chassis_poscnt + i];
            deltaY = chassis.pos_y - chassis_ypos1[chassis_poscnt + i];           
          }else if(ChassisSignal.m_CtrlFlag._routeflag == 3)
          {
            deltaX = chassis.pos_x - chassis_xpos2[chassis_poscnt + i];
            deltaY = chassis.pos_y - chassis_ypos2[chassis_poscnt + i];  
          }
          else if(ChassisSignal.m_CtrlFlag._routeflag == 4)
          {
            deltaX = chassis.pos_x - chassis_xpos3[chassis_poscnt + i];
            deltaY = chassis.pos_y - chassis_ypos3[chassis_poscnt + i];           
          }
          else if(ChassisSignal.m_CtrlFlag._routeflag == 5)
          {
            deltaX = chassis.pos_x - chassis_xpos4[chassis_poscnt + i];
            deltaY = chassis.pos_y - chassis_ypos4[chassis_poscnt + i];           
          }
          else if(ChassisSignal.m_CtrlFlag._routeflag == 6)
          {
            deltaX = chassis.pos_x - chassis_xpos5[chassis_poscnt + i];
            deltaY = chassis.pos_y - chassis_ypos5[chassis_poscnt + i];           
          }
        
        if(deltaX < 0)  deltaX *= -1;
        if(deltaY < 0)  deltaY *= -1;
        dis_to_next = sqrt(deltaX * deltaX + deltaY * deltaY);
        
        int arriveJudge = 0;
        if(theMode == _FIXED_TRACK_MODE)
        {
            if(chassis.g_ispeed >= 700)///动态scope
            {
                scope_path = 0.16;
            }else
            {
                scope_path = 0.03;
            } 
            arriveJudge = (dis_to_next <= 0.03 && ( chassis_poscnt + i >= chassis_posnum - 2)) 
                || (dis_to_next <= scope_path && chassis_poscnt + i >= 0 && chassis_poscnt + i < chassis_posnum - 2);
        }
        if(arriveJudge)//判断经过此点 
        {
            chassis_poscnt += i + 1;
            //uprintf(CMD_USART,"到达 chassis_poscnt = %d  angle = %f\r\n",chassis_poscnt,chassis.g_fangle*180/PI);
            if(chassis_poscnt >= chassis_posnum - 1)//到达目的地
            {
                //uprintf(CMD_USART,"到达终点 chassis_poscnt = %d\r\n",chassis_poscnt);
                chassis.g_ispeed = 0;
                chassis.g_fangle = 0;
                
                ChassisSignal.m_CtrlFlag._sensor_flag = 0;
                ChassisSignal.m_CtrlFlag._routeflag= 0;
                ChassisSignal.m_FinishFlag._position_flag = 1;
                chassis_poscnt = 0;
                return ;
            }
            break;
        }
    }
    
    if(theSpeedMode == _SPEED_SHEET)
    {
        vec target, direct;
        if(chassis_arena == LEFT_ARENA)
        {
          if(ChassisSignal.m_CtrlFlag._routeflag == 1)
          {
            target = vec_create((float)chassis_speed[chassis_poscnt] * cos(chassis_speed_dir[chassis_poscnt]),  (float)chassis_speed[chassis_poscnt] * sin(chassis_speed_dir[chassis_poscnt]));
            direct = vec_create(chassis_xpos[chassis_poscnt] - chassis.pos_x, chassis_ypos[chassis_poscnt] - chassis.pos_y);
          }else if(ChassisSignal.m_CtrlFlag._routeflag == 2)
          {
            target = vec_create((float)chassis_speed1[chassis_poscnt] * cos(chassis_speed_dir1[chassis_poscnt]),  (float)chassis_speed1[chassis_poscnt] * sin(chassis_speed_dir1[chassis_poscnt]));
            direct = vec_create(chassis_xpos1[chassis_poscnt] - chassis.pos_x, chassis_ypos1[chassis_poscnt] - chassis.pos_y);            
          }else if(ChassisSignal.m_CtrlFlag._routeflag == 3)
          {
            target = vec_create((float)chassis_speed2[chassis_poscnt] * cos(chassis_speed_dir2[chassis_poscnt]),  (float)chassis_speed2[chassis_poscnt] * sin(chassis_speed_dir2[chassis_poscnt]));
            direct = vec_create(chassis_xpos2[chassis_poscnt] - chassis.pos_x, chassis_ypos2[chassis_poscnt] - chassis.pos_y);              
          }
        }
        else if(chassis_arena == RIGHT_AREAN)
        {
          if(ChassisSignal.m_CtrlFlag._routeflag == 4)
          {
            target = vec_create((float)chassis_speed[chassis_poscnt] * cos(chassis_speed_dir3[chassis_poscnt]),  (float)chassis_speed[chassis_poscnt] * sin(chassis_speed_dir3[chassis_poscnt]));
            direct = vec_create(chassis_xpos3[chassis_poscnt] - chassis.pos_x, chassis_ypos3[chassis_poscnt] - chassis.pos_y);              
          }else if(ChassisSignal.m_CtrlFlag._routeflag == 5)
          {
            target = vec_create((float)chassis_speed1[chassis_poscnt] * cos(chassis_speed_dir4[chassis_poscnt]),  (float)chassis_speed1[chassis_poscnt] * sin(chassis_speed_dir4[chassis_poscnt]));
            direct = vec_create(chassis_xpos4[chassis_poscnt] - chassis.pos_x, chassis_ypos4[chassis_poscnt] - chassis.pos_y);              
          }else if(ChassisSignal.m_CtrlFlag._routeflag == 6)
          {
            target = vec_create((float)chassis_speed2[chassis_poscnt] * cos(chassis_speed_dir5[chassis_poscnt]),  (float)chassis_speed2[chassis_poscnt] * sin(chassis_speed_dir5[chassis_poscnt]));
            direct = vec_create(chassis_xpos5[chassis_poscnt] - chassis.pos_x, chassis_ypos5[chassis_poscnt] - chassis.pos_y);              
          }
        }
        vec now = vec_create(chassis.speed_x,chassis.speed_y);
        vector_track_ctrl(now, target, direct);
    }
}



void chassis_pid_test()
{
    chassis_init_pos(0,0);
    chassis.g_ispeed = 800;
    chassis.g_fturn = 0;
    float target_x = 0;
    float target_y = -1.5;
    //float last_x = 0;
    //float last_y = 0;
    double deltaX = chassis.pos_x - target_x;
    double deltaY = chassis.pos_y - target_y;
    double dis_to_next = sqrt(deltaX * deltaX + deltaY * deltaY);
    if(dis_to_next < 0.1)
    {
        chassis.g_ispeed = 0;
    }
    /*float now_angle = atan2(target_y - chassis.pos_y,target_x - chassis.pos_x);//点点方向
    float target_angle = atan2(target_y - last_y,target_x - last_x);
    float tmp_g_fangle = PID_Release(&chassis_PID,target_angle,now_angle);
    if(tmp_g_fangle > PI/2) tmp_g_fangle = PI/2;
    if(tmp_g_fangle < -PI/2) tmp_g_fangle = -PI/2;
    chassis.g_fangle = chassis_angle_subtract(now_angle, tmp_g_fangle);*/
    chassis.g_fangle = chassis_angle_pid_ctrl(target_x, target_y);
    
}

void chassis_exe()
{
    chassis_automata();
    if(0 != ChassisSignal.m_CtrlFlag._routeflag && ChassisSignal.m_CtrlFlag._vega_ready_flag == 1)
    {
        //chassis_modify_handle_control();
        chassis_go_track(ChassisSignal.m_DriveMode, ChassisSignal.m_SpeedMode);
        //uprintf(CMD_USART,"speed = %d\r\n",chassis.g_ispeed);
        //chassis.g_ispeed = 200;
    }
    slow_turn_func();
    absolute_slow_turn_thread();
    chassis_gostraight(chassis.g_ispeed ,chassis.g_fangle, chassis.g_fturn, ChassisSignal.m_CtrlFlag._handle_flag); //放在上面的括号里了
}

int open_box1()
{
  static int state = 0;
  switch(state)
  {
  case 0:
    if(open_box == 1)
    {
      chassis.g_ispeed = 0;
      chassis.g_fangle = 0;
      if(chassis_arena == LEFT_ARENA)
      {
        TIM1->CCR1 = 175;
        state = 1;
        uprintf(CMD_USART, "one\r\n");
       // medical_take_off = 1;//xmy
      }else if(chassis_arena == RIGHT_AREAN)
      {
        TIM1->CCR2 = 125;
        state = 2;
        uprintf(CMD_USART, "three\r\n");
        //medical_take_off = 2;
      }
    }
    break;
  case 1:
    if(medical_take_off == 1 || medical_take_off == 2)
    {
      state = 0;
      return 1;
    }
    break;
  case 2:
    if(medical_take_off == 1 || medical_take_off == 2)
    {
      state = 0;
      return 1;
    }
    break;
  }
  return 0;

}

int open_box2()
{
  static int state = 0;
  switch(state)
  {
  case 0:
    if(open_box == 1)
    {
      chassis.g_ispeed = 0;
      chassis.g_fangle = 0; 
      if(chassis_arena == LEFT_ARENA)
      {
        TIM1->CCR2 = 125;
        state = 1;
        uprintf(CMD_USART, "three\r\n");
       // medical_take_off = 2;//xmy
      }else if(chassis_arena == RIGHT_AREAN)
      {
        TIM1->CCR1 = 175;
        state = 2;
        uprintf(CMD_USART, "one\r\n");
       // medical_take_off = 1;
      }
    }
    break;
  case 1:
    if(medical_take_off ==3)
    {
      state = 0;
      return 1;
    }
    break;
  case 2:
    if(medical_take_off == 3)
    {
      state = 0;
      return 1;
    }
    break;
  }
  return 0;

}

void chassis_automata()
{
    
    switch(Chassis_State)
    {
      case 0:
        //do nothing
        break;
      case 1:
        if(chassis_winding_and_deliver() == 1)
        {
            open_box = 1;
            //uprintf(CMD_USART, "switch to 20\r\n");
           if(chassis_arena == LEFT_ARENA)
            slow_turn = slow_turn_angle;
           else if(chassis_arena == RIGHT_AREAN)
             slow_turn = -slow_turn_angle;
           
            slow_turn_const = very_slow;
//            chassis.g_ispeed = -move_speed;
//            chassis.g_fangle = -PI; 
            Chassis_State = 99;
        }
        break;
    case 99:
      if(ChassisSignal.m_FinishFlag._rotate_flag == 1)
      {
        ChassisSignal.m_FinishFlag._rotate_flag = 0;
        Chassis_State = 100;
      }
      break;
      case 100:
        if(open_box1() == 1)
        {
            delay_num_const = 1000;
            Chassis_State = 19;
        }
        break;
    case 19:
      if(ChassisSignal.m_FinishFlag._delay_flag == 1)
      {
        ChassisSignal.m_FinishFlag._delay_flag = 0;
        ChassisSignal.m_CtrlFlag._handle_flag = 0;
        //chassis.g_fturn = 0;//防止方向环突变 
        chassis_update();
           if(chassis_arena == LEFT_ARENA)
            slow_turn = -slow_turn_angle;
           else if(chassis_arena == RIGHT_AREAN)
             slow_turn = slow_turn_angle;
        
        slow_turn_const = slow;
        chassis_poscnt = 0;
        Chassis_State = 2000;
        open_box = 1;
      }
      break;
    case 2000:
      if(ChassisSignal.m_FinishFlag._rotate_flag == 1)
      {
        ChassisSignal.m_FinishFlag._rotate_flag = 0;
        Chassis_State = 20;
      }
      break;
      case 20:
        if(chassis_winding_and_deliver_1() == 1)
        {
            open_box = 1;
            //uprintf(CMD_USART, "switch to 30\r\n");
//            chassis.g_ispeed = move_speed;
//            chassis.g_fangle = -PI; 
           if(chassis_arena == LEFT_ARENA)
            slow_turn = -slow_turn_angle;
           else if(chassis_arena == RIGHT_AREAN)
             slow_turn = slow_turn_angle;
            slow_turn_const = very_slow;
            Chassis_State =199;
            
        }
        break;
    case 199:
      if(ChassisSignal.m_FinishFlag._rotate_flag == 1)
      {
        ChassisSignal.m_FinishFlag._rotate_flag = 0;
        Chassis_State = 200;
      }
      break;
    case 200:
        if(open_box2() == 1)
        {
            open_box = 1;
            //uprintf(CMD_USART, "开箱2\r\n");
            delay_num_const = 1000;
            Chassis_State = 29;
        }
        break;
    case 29:
      if(ChassisSignal.m_FinishFlag._delay_flag == 1)
      {
        ChassisSignal.m_FinishFlag._delay_flag = 0;
        ChassisSignal.m_CtrlFlag._handle_flag = 0;
        //chassis.g_fturn = 0;//防止方向环突变 
        chassis_update();
        chassis_poscnt = 0;
           if(chassis_arena == LEFT_ARENA)
            slow_turn = slow_turn_angle;
           else if(chassis_arena == RIGHT_AREAN)
             slow_turn = -slow_turn_angle;

        slow_turn_const = slow;
        Chassis_State = 3000;
        open_box = 1;
      }
      break;
    case 3000:
      if(ChassisSignal.m_FinishFlag._rotate_flag == 1)
      {
        ChassisSignal.m_FinishFlag._rotate_flag = 0;
        Chassis_State = 30;
      }
      break;
      case 30:
        if(chassis_winding_and_deliver_2() == 1)
        {
            //uprintf(CMD_USART, "switch to 0\r\n");
            TIM1->CCR1 = 160;
            TIM1->CCR2 = 140;
            Chassis_State = 0;
        }
        break;
      case 2:
        if(chassis_calbration_and_craw() == 1)
        {
            //uprintf(CMD_USART, "switch to 3\r\n");
            Chassis_State = 0;
        }
        break;
      case 3:
        if(chassis_throw_thrice_first() == 1)
        {
            Chassis_State = 0;
        }
        break;
      case 4:
        if(chassis_throw_thrice_second() == 1)
        {
            Chassis_State = 0;
        }
        break;
      case 5:
        if(chassis_throw_thrice_third() == 1)
        {
            Chassis_State = 0;
        }
        break;
    }
}


//手柄can的按键id是325（10）
//此函数接收can发来的消息
void chassis_handle(CanRxMsgTypeDef* pRxMsg)
{
    if(0 == main_flag) return;
    
    uint8_t Data[8];
    int i;
    for(i = 0; i < 8; i++)
    {
        Data[i] = pRxMsg->Data[i];
    }
    /*if(Data[0] >= '0' && Data[0] <= '9' && (Data[1] == 'u' || Data[1] == 'd'))
    {
    if(Data[1] == 'u')
    {
    ChassisHandle.btstate[(int)(Data[0] - '0')] = 1;
}
    else if(Data[1] == 'd')
    {
    ChassisHandle.btstate[(int)(Data[0] - '0')] = 0;
}
}*/
    int id = (int)((Data[0] - '0') * 10 + (Data[1] - '0'));
    if(Data[2] == 'd')
    {
        switch(id)
        {
            //拨码开关到00
          case 0:
            ChassisSignal.m_CtrlFlag._handle_flag = 1;
            ChassisSignal.m_CtrlFlag._routeflag = 0;
            Chassis_State = 0;
            if(chassis_speed_max < 600) chassis_speed_max = 600;
            chassis_speed_max += 100;//增加最大速度
            if(chassis_speed_max >= 2000)
                chassis_speed_max = 2000;
            break;
          case 1:
            craw_close();
            break;
          case 2:
            craw_open();
            break;
          case 3:
            break;
          case 6:
            for(int i = 0;i<=10;i++)
            {
                uprintf(ACTION_USART,"ACT0");
            }
            ChassisSignal.m_CtrlFlag._handle_flag = 0;
            ChassisSignal.m_CtrlFlag._routeflag = 1;
            //modify_x = ORIGIN_X - (chassis.pos_x - modify_x);//修改全场定位初始值
            //modify_y = ORIGIN_Y - (chassis.pos_y - modify_y);
            chassis.g_fturn = chassis.angle;//防止方向环突变 
            chassis_update();
            //chassis_modify_pos(chassis_xpos,chassis_ypos,ORIGIN_X,ORIGIN_Y);
            chassis_poscnt = 0;
            Chassis_State = 1;
            break;
          case 7:
            ChassisSignal.m_CtrlFlag._handle_flag = 0;
            
            //ChassisSignal.m_CtrlFlag._routeflag = 1;
            
            Chassis_State = 3;
            break;
          case 8:
            ChassisSignal.m_CtrlFlag._handle_flag = 0;
            
            //ChassisSignal.m_CtrlFlag._routeflag = 1;
            Chassis_State = 2;
            break;
          case 9:
            upper_flag = 8;//投掷后复原
            break;
            //拨码01
          case 10:
            ChassisSignal.m_CtrlFlag._handle_flag = 0;
            
            //ChassisSignal.m_CtrlFlag._routeflag = 1;
            track_max_speed = 1100;
            track_test_offset = 0;
            Chassis_State = 3;
            break;
          case 11:
            ChassisSignal.m_CtrlFlag._handle_flag = 0;
            
            //ChassisSignal.m_CtrlFlag._routeflag = 1;
            track_max_speed = 1100;
            track_test_offset = 0.10;
            Chassis_State = 3;
            break;
          case 12:
            ChassisSignal.m_CtrlFlag._handle_flag = 0;
            
            //ChassisSignal.m_CtrlFlag._routeflag = 1;
            track_max_speed = 1100;
            track_test_offset = 0.20;
            Chassis_State = 3;
            break;
          case 13:
            ChassisSignal.m_CtrlFlag._handle_flag = 0;
            
            //ChassisSignal.m_CtrlFlag._routeflag = 1;
            track_max_speed = 1100;
            track_test_offset = -0.10;
            Chassis_State = 3;
            break;
            
            //拨码10
          case 23:
            if(micro_switch_test_flag == 1)
                micro_switch_test_flag = 0;
            else
                micro_switch_test_flag = 1;
            break;
            
            //拨码开关到11
          case 30:
            ChassisSignal.m_CtrlFlag._handle_flag = 1;
            ChassisSignal.m_CtrlFlag._routeflag = 0;
            Chassis_State = 0;
            if(chassis_speed_max < 600) chassis_speed_max = 600;
            chassis_speed_max += 100;
            if(chassis_speed_max >= 2000)
                chassis_speed_max = 2000;
            break;
          case 31:
            craw_close();
            B15_LOW;
            deliver_back();
            break;
          case 32:
            craw_open();
            break;
          case 33:
            upper_flag = 200;//投不收
            break;
          case 36:
            upper_flag = 3;
            break;
          case 37:
            upper_flag = 6;
            break;
          case 38:
            upper_flag = 305;
            break;
          case 39:
            upper_flag = 8;
            break;
        }
    }
    
}

//摇杆的can的id为324
void chassis_rocker(CanRxMsgTypeDef* pRxMsg)
{
    if(0 == main_flag) return;
    uint8_t Data[8];
    int i;
    for(i = 0; i < 8; i++)
    {
        Data[i] = pRxMsg->Data[i];
    }
    //常数修改零偏
    ChassisHandle.ry = (int)Data[0] - 128;
    ChassisHandle.rx = (int)Data[1] - 131;
    ChassisHandle.ly = (int)Data[2] - 119;
    ChassisHandle.lx = (int)Data[3] - 125;
    //变换坐标系
    ChassisHandle.ry *= -1;
    ChassisHandle.ly *= -1;
}

void chassis_modify_handle_control()
{
    if(Chassis_State >= 3 && Chassis_State <= 5 )
    {
        if(ChassisHandle.lx > 30  || ChassisHandle.lx < -30)//&& Chassis_State  >= 3 && Chassis_State <= 5)
        {
            catch_bone_modify_x -= 0.002 * ChassisHandle.lx * cos(chassis.angle) / 128;
        }
        if(ChassisHandle.rx > 30  || ChassisHandle.rx < -30)//&& Chassis_State  >= 3 && Chassis_State <= 5)
        {
            catch_bone_modify_x -= 0.002 * ChassisHandle.rx * cos(chassis.angle) / 128;
        }
    }
}

void chassis_handle_control() 
{   
    float dis = 0,dis2 = 0;
    
    switch(ChassisHandle.mode)
    {
        /*
      case 0:
        chassis.g_fangle = atan2(ChassisHandle.ly, ChassisHandle.rx);
        dis = sqrt(ChassisHandle.ly * ChassisHandle.ly + ChassisHandle.rx * ChassisHandle.rx);
        chassis.g_ispeed = (int)(chassis_speed_max * dis * dis / 16384);//速度叠加
        if(chassis.g_ispeed > chassis_speed_max) chassis.g_ispeed = (int)chassis_speed_max;
        if(chassis.g_ispeed < 0) chassis.g_ispeed = 0;
        if(ChassisHandle.btstate[8] == 1)
        chassis.g_fturn += 0.01;
        if(ChassisHandle.btstate[7] == 1)
        chassis.g_fturn -= 0.01;
        break;
      case 1:
        chassis.g_fangle = atan2(ChassisHandle.ly, ChassisHandle.lx);
        dis = sqrt(ChassisHandle.ly * ChassisHandle.ly + ChassisHandle.lx * ChassisHandle.lx);
        chassis.g_ispeed = (int)(chassis_speed_max * dis * dis / 16384);//速度叠加
        if(chassis.g_ispeed > chassis_speed_max) chassis.g_ispeed = (int)chassis_speed_max;
        if(chassis.g_ispeed < 0) chassis.g_ispeed = 0;
        dis2 = sqrt(ChassisHandle.ry * ChassisHandle.ry + ChassisHandle.rx * ChassisHandle.rx);
        if(dis2 > 100)
        {
        handle_turn = chassis_angle_subtract(atan2(ChassisHandle.ry, ChassisHandle.rx), - PI/2);
        abs_slow_turn_flag = 1;
    }
        break;
        */
      case 2:
        chassis.g_fangle = atan2(ChassisHandle.ly, ChassisHandle.lx) + chassis.angle;
        dis = sqrt(ChassisHandle.ly * ChassisHandle.ly + ChassisHandle.lx * ChassisHandle.lx);
        chassis.g_ispeed = (int)(chassis_speed_max * dis * dis / 16384);//速度叠加
        if(chassis.g_ispeed > chassis_speed_max) chassis.g_ispeed = (int)chassis_speed_max;
        if(chassis.g_ispeed < 0) chassis.g_ispeed = 0;
        
        
        
        dis2 = sqrt(ChassisHandle.ry * ChassisHandle.ry + ChassisHandle.rx * ChassisHandle.rx);
        if(dis2 > 110)
        {
            chassis.g_fturn = 150 * chassis_angle_subtract(atan2(ChassisHandle.ry, ChassisHandle.rx), - PI/2);
        }
        else
        {
            chassis.g_fturn = 0;
        }
        

        break;
    }
}

/*void press_rcv_callback(CanRxMsgTypeDef* pRxMsg)
{
if(pRxMsg->Data[0] == '1') 
{
ChassisSignal.m_CtrlFlag._press_flag = 1;
uprintf(CMD_USART,"pressed\r\n");
    }  
    else if(pRxMsg->Data[0] == '0')
{
ChassisSignal.m_CtrlFlag._press_flag = 0;
    }
}*/

float speed_to_targetpoint(int now_pos,int target_pos)
{
    float delt_speed = chassis_speed[target_pos] - chassis_speed[now_pos];//-200
    float delt_dis = sqrt((chassis_xpos[now_pos] - chassis_xpos[target_pos])*(chassis_xpos[now_pos] - chassis_xpos[target_pos]) + (chassis_ypos[now_pos] - chassis_ypos[target_pos])*(chassis_ypos[now_pos] - chassis_ypos[target_pos]));
    float delt_dis_nowpoint = sqrt((chassis_xpos[now_pos] - chassis.pos_x)*(chassis_xpos[now_pos] - chassis.pos_x) + (chassis_ypos[now_pos] - chassis.pos_y)*(chassis_ypos[now_pos] - chassis.pos_y));
    
    float now_speed = chassis_speed[now_pos] + delt_dis_nowpoint*delt_speed/delt_dis;
    if(chassis_speed[target_pos] >= chassis_speed[now_pos])
    {
        if(now_speed >= chassis_speed[target_pos])
            now_speed = chassis_speed[target_pos];
    }else if(chassis_speed[target_pos] < chassis_speed[now_pos])
    {
        if(now_speed <= chassis_speed[target_pos])
            now_speed = chassis_speed[target_pos];
    }
    
    
    return now_speed;
}


float chassis_angle_pid_ctrl(float target_x_next, float target_y_next)
{
    /*float now_x = chassis.now_speed[1] - chassis.now_speed[2] * cos(PI/3) - chassis.now_speed[0] * cos(PI/3);
    float now_y = chassis.now_speed[2] * cos(PI/6) - chassis.now_speed[0] * cos(PI/6);
    float now_angle = atan2(chassis.speed_y,chassis.speed_x);
    float target_angle = atan2(target_y_next - chassis.pos_y, target_x_next - chassis.pos_x);
    float tmp_g_fangle = 0;
    if(fabs(chassis.speed_x) < 1e-5 && fabs(chassis.speed_y) < 1e-5)
    tmp_g_fangle = 0;
    else
    tmp_g_fangle = PID_Release(&chassis_PID,target_angle,now_angle);
    if(tmp_g_fangle > PI/2) tmp_g_fangle = PI/2;
    if(tmp_g_fangle < -PI/2) tmp_g_fangle = -PI/2;
    return chassis_angle_subtract(target_angle, tmp_g_fangle);*/
    return 0;
}

void chassis_modify_pos(float x[],float y[],float x0,float y0)
{
    if(chassis.angle <= 0.0017 && chassis.angle >= -0.0017)//小于0.1°不处理
        return;
    
    int temp_num = NUM_POINTS;
    for(int i = 0; i < temp_num; i++)
    {
        x[i] = x0 + sqrt((x[i] - x0) * (x[i] - x0) + (y[i] - y0) * (y[i] - y0)) * cos(atan2((y[i] - y0),(x[i] - x0)) - chassis.angle);
        y[i] = y0 + sqrt((x[i] - x0) * (x[i] - x0) + (y[i] - y0) * (y[i] - y0)) * sin(atan2((y[i] - y0),(x[i] - x0)) - chassis.angle);
    }
}


/* ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/

/**速度矢量相加，返回值为合速度矢量
*参数：vec1和vec2为待相加的矢量
*返回值： 合速度矢量
*说明: 用于point_tracer使用
*作者: zx
*/
vec speed_vec_add(vec vec1 , vec vec2){
  float v1_x , v1_y , v2_x , v2_y,speed,angle;
  v1_x = cosf(vec1.y)* vec1.x;
  v1_y = sinf(vec1.y)* vec1.x;
  v2_x = cosf(vec2.y)* vec2.x;
  v2_y = sinf(vec2.y)* vec2.x;
  speed = sqrtf( (v1_x+v2_x) * (v1_x+v2_x) + (v1_y+v2_y)*(v1_y+v2_y));
  angle = atan2f( v2_y + v1_y , v1_x + v2_x );
  return vec_create(speed,angle);
}

vec speed_vec_mul(vec *vec1 ,float k){
  vec1->x = vec1->x * k;
  return vec_create(vec1->x , vec1->y);
}

/*line_control函数中的PID*/
PID_Struct_zx line_control_PID = {
5000,   //kp
0,      //Ki
50,      //Kd
0,      //i （中间变量）
0,      //last_error
500,    //i_max
0,      //last_d
0.005   //I_TIME
};
//以下变量为line_control内部调用
int total_line_control_flag = 1;    //为0时禁用line_control，为1时启用
int line_control_flag_first = 1;    //为1时：为第一次跑一个新点（设置这个flag的目的主要为减少计算次数）
float origin_distance,origin_angle;
/**根据与轨迹（直线）的误差生成纠正的速度矢量
*参数：float p1_x，p1_y 地面坐标系下开始点的位置
*参数：float p2_x，p2_y 地面坐标系下开始点的位置
*参数：float max_speed 最大速度（限幅使用）
*返回值： 底盘驱动的方向矢量 vec.x 为速度大小，vec.y为速度方向（方向为：-PI~PI)
*说明: 纠正轨迹使用，配合point_tracer使用
*作者: zx
*/
vec line_control(float p1_x , float p1_y ,float p2_x,float p2_y, float max_speed){  
  float current_agnle , current_distance , error_vertical , error_on_origin ,angle_sub;
  float reference_point_x , reference_point_y;
  float target_angle , target_speed;
  //float out_put_speed = 0.0f;
  if(total_line_control_flag == 1){
    if(line_control_flag_first == 1){
    origin_angle = atan2f( p2_y - p1_y , p2_x - p1_x );    
    origin_distance = sqrtf( (p2_x - p1_x)*(p2_x - p1_x) + (p2_y - p1_y)*(p2_y - p1_y) );
    line_control_flag_first = 0;
    }  
    
    current_agnle = atan2f( p2_y - chassis.pos_y , p2_x - chassis.pos_x );
    current_distance = sqrtf( (p2_x - chassis.pos_x)*(p2_x - chassis.pos_x) + (p2_y - chassis.pos_y)*(p2_y - chassis.pos_y) );
    angle_sub = fabsf(origin_angle - current_agnle);
    error_on_origin = cosf(angle_sub)*current_distance;
    error_vertical = fabsf(sinf(angle_sub)*current_distance);  //控制量

    reference_point_x = (origin_distance - error_on_origin)*cosf(origin_angle) + p1_x;
    reference_point_y = (origin_distance - error_on_origin)*sinf(origin_angle) + p1_y;  

    target_angle = atan2f(reference_point_y - chassis.pos_y , reference_point_x - chassis.pos_x);   //用来矫正的速度矢量的方向
    
    if(error_vertical <= 1e-3){
      target_speed = 0;
    }else
    target_speed = fabsf(PID_Release_zx(&line_control_PID , 0 , error_vertical));

    if(target_speed >= max_speed) target_speed = max_speed;
    if(target_speed <= -max_speed) target_speed = -max_speed;
  
    // what's beneath is the test progarm
     uprintf(CMD_USART , "error_vertical : %f , speed : %f , angle : %f\r\n" ,error_vertical , target_speed , target_angle);
     chassis_gostraight_zx( (int)target_speed , target_angle , 0 , 0);
  }
  return vec_create(target_speed , target_angle);
}


/**根据目标点更新新的速度方向
*参数：float point_y 	 point_x  地面坐标系下目标点的位置
*返回值： 底盘的速度方向角
*说明: by zx
*/
float point_tracer_angle_return( float point_x , float point_y ){
  //float origin_distance = sqrtf( (chassis.pos_x - point_x)*(chassis.pos_x - point_x) + (chassis.pos_y - point_y)*(chassis.pos_y - point_y) );
  //float out_put_speed = 0.0f;
  //float origin_angle = atan2f( point_y - chassis.pos_y , point_x - chassis.pos_x );
  //float current_agnle , current_distance , error_vertical;
  
  float chassis_target_x = point_x - chassis.pos_x;  
  float chassis_target_y = point_y - chassis.pos_y;   //坐标系变换，在保证角度环（小车坐标系不变的情况下），讲目标点由地面坐标系变换到小车坐标系中
  float mid = atan2f(chassis_target_y , chassis_target_x);
  float angle = chassis_angle_subtract( mid , chassis.angle);
  return angle;
}

float point_tracer_angle_return_static(float start_x, float start_y , float end_x , float end_y){
  float angle = atan2f( end_y-start_y , end_x-start_x );
  return angle;
}


//以下为 speed_trapezium 和 BoostAndSlowUpdate 用到的变量和参数
float Boost_Slow_Period = 0.105;
float Boost_Period = 0.1;
float Slow_Period = 0.16;
int first_time_controler = 1; //值为1时：对一个新点使用此函数（不用多次调用BoostAndSlowUpdate）
float last_point_x,last_point_y,origin_distance;

/**梯形速度曲线 加速阶段和减速阶段根据不同距离和速度进行自动调整 
*参数：float total_distance 起始点到终点的总距离 start_speed 开始时速度 final_speed结束时速度 max_speed最大速度
*返回值： 无
*说明: by zx 结合 speed_trapezium使用，speed_trapezium需要的函数
*/
void BoostAndSlowUpdate(float total_distance , int start_speed , int final_speed , int max_speed){
  Slow_Period = -0.195*total_distance + 0.00205*(max_speed - final_speed + 50) -0.7925;
  
  if(Slow_Period >= 0.6) Slow_Period = 0.6;  //Slow_Period 限制幅度
  else if(Slow_Period <= 0.1) Slow_Period = 0.1;
  else Slow_Period = Slow_Period;

  if(total_distance >= 5){    //boost_period 根据经验误差估计
    Boost_Period = 0.08;
  }
  else if(total_distance < 5 && total_distance > 1.5){
    Boost_Period = 0.12;
  }
  else{ //小于1.5m
    if(max_speed - start_speed >300)
    Boost_Period = 0.3;
    else Boost_Period = 0.1;
  }
}

//float last_pos , this_time_pos;

/**速度曲线生成 梯形曲线
*参数：float point_y 	 point_x  地面坐标系下目标点的位置 start_speed 开始时速度 final_speed结束时速度 max_speed最大速度
*返回值： 电机当前的速度
*说明: by zx
*/
int speed_trapezium (float point_x , float point_y , int start_speed , int final_speed , int max_speed){
  float distance_to_target,speed;
  int int_speed;
  // if(first_time_controler == 0){
  //   this_time_pos = chassis.pos_y;
  //   uprintf(CMD_USART , "speed is : %f , %f , %f\r\n", (last_pos - this_time_pos)/0.005, chassis.pos_x, chassis.pos_y);
  //   last_pos = this_time_pos;
  // } //this part of code were uesd to test this function
  
  if(first_time_controler == 1){
    last_point_x = chassis.pos_x;
    last_point_y = chassis.pos_y;
    first_time_controler = 0;
    origin_distance = sqrtf( (point_x - last_point_x)*(point_x - last_point_x) + (point_y - last_point_y)*(point_y - last_point_y) );
    BoostAndSlowUpdate(origin_distance , start_speed , final_speed , max_speed);
    //last_pos = chassis.pos_y;
  }
  distance_to_target = sqrtf( (point_x - chassis.pos_x)*(point_x - chassis.pos_x) + (point_y - chassis.pos_y)*(point_y - chassis.pos_y) );

  distance_to_target =  1 - distance_to_target / origin_distance ; //正则化，使距离都限制在0~1范围内
  if(distance_to_target >=  1) distance_to_target = 1;
  if(distance_to_target <=  0) distance_to_target = 0;

  if(origin_distance <= 1) //如果距离小于20cm不进行梯形速度规划 使用从起始速度到最终速度的直接线性规划（一条连接起始速度和最终速度的直线）
  {
    speed = (final_speed - start_speed)*distance_to_target + start_speed;
  }
  else{
    //以下为一个分段函数，用这个函数计算速度 （梯形速度控制）
    if(distance_to_target <= Boost_Period){     //对于max_speed==start_speed 时进行优化
      if(start_speed == max_speed) speed = max_speed;
      else
      speed = (max_speed - start_speed)* distance_to_target/Boost_Period + start_speed; 
    }
    else if(distance_to_target > Slow_Period && distance_to_target <= 1 - Slow_Period){
      speed = max_speed;
    }
    else{
      if(final_speed == max_speed) int_speed = max_speed; //对于max_speed==final_speed 时进行优化
      else
      speed = (final_speed - max_speed)/ Slow_Period * distance_to_target + ( Slow_Period * final_speed - final_speed + max_speed)/ Slow_Period;
    }
  }
  int_speed = (int)speed;

  if(int_speed > max_speed) //速度限制
    int_speed = max_speed;
  else if(int_speed <= 0) 
    int_speed = 0;

  //uprintf(CMD_USART,"speed : %d  " , int_speed);
  return int_speed;
}

int point_tracer_flag = 0;  //为1时：可以执行point_tracer；为0时：禁用point_tracer PS:此变量主要为调试时方便控制
int point_arrived_flag = 0; //到达点后为1
/**到达点后，将各种flag置位
*参数：void
*返回值： void
*说明: 为point_tracer内部函数，配合point_tracer使用
*作者: zx
*/
void point_arrive(){
  point_arrived_flag = 1;
  point_tracer_flag = 1;
  first_time_controler = 1;
  line_control_flag_first = 1;
  point_retrack_first_ref_flat = 1;  
  uprintf(CMD_USART,"\r\nArrived !!%d \r\n",count);
}

/**【核心函数】 point_tracer,跑点函数
*参数：float start_x , float start_y 为起始点地面坐标系坐标
*参数：float point_x , float point_y 为最终点地面坐标系坐标
*参数：int start_speed , int final_speed , int max_speed; start_speed 开始时速度，max_speed最大速度，final_speed到达终点时速度
*返回值： int; <1> 返回值为0时：可以开始跑点，并且未到达目标点
*返回值： int; <2> 返回值为-1时：目前不程序不允许跑点
*返回值： int; <3> 返回值为1时：到达目标点
*说明: 核心函数，用于跑点，最终配合point_collection_tracer（跑点集）使用
*作者: zx
*/
vec combine_vec , line_control_vec;   //point_tracer内部变量
int point_tracer (float start_x , float start_y ,float point_x , float point_y , int start_speed , int final_speed , int max_speed){
  float toward_angle,toward_speed;
  float distance = sqrtf( (chassis.pos_x - point_x)*(chassis.pos_x - point_x) + (chassis.pos_y - point_y)*(chassis.pos_y - point_y) );
  if( point_tracer_flag == 1 && distance >= ARRIVE_DISTANCE ) //可以开始跑点，并且未到达目标点
  { 
    //toward_angle = point_tracer_angle_return(point_x , point_y);
    toward_angle = point_tracer_angle_return_static(start_x , start_y , point_x , point_y);
    toward_speed = speed_trapezium (point_x , point_y , start_speed , final_speed , max_speed);
    
    //combine_vec = vec_create(toward_speed , toward_angle);
    line_control_vec = line_control(start_x , start_y , point_x , point_y , 700);
    float k1 = line_control_vec.x /750;
    speed_vec_mul( &line_control_vec , 1+ k1);
    combine_vec = speed_vec_add( vec_create(toward_speed , toward_angle) , line_control_vec);
    //chassis_update();
    chassis_gostraight_zx( (int)combine_vec.x , combine_vec.y , 0 , 0);
    return 0;
  }
  else {
    if(point_tracer_flag != 1) //不可跑点
    return -1;

    if(distance < ARRIVE_DISTANCE) //到达点
    {
      chassis_gostraight_zx( final_speed , combine_vec.y , 0 , 0);
      point_arrive();
      return 1;
    }
    
  }
  return -1;
}

int ENBALE_POINT_COLLECTION_TRACER = 0;

// float points_x[7]={3 , 3.7 , 3 , 2.3 , 3 , 3.7 ,3};
// float points_y[7]={6.5 , 7.5 , 8.5 , 9.5 , 10.5 , 11.5 , 12.5};
// int speed_control[8]={125 ,350, 200 ,830 , 200, 830, 125 , 50};
//int max_speed[];
// float points_x[1]={3};
// float points_y[1]={10};
// int speed_control[2]={125 ,0};
int count = 0;
int point_count_control_flag = 0;

int point_retrack_first_ref_flat = 1;
int point_retrack(float start_x ,float start_y , float final_X , float final_y){
  static float total_distance;
  float distance_now;
  if(point_retrack_first_ref_flat == 1){
    total_distance = sqrtf( (final_X - start_x)*(final_X - start_x) + (final_y - start_y)*(final_y - start_y) );
    point_retrack_first_ref_flat = 0;
    return 1;
  }
  distance_now = sqrtf( (chassis.pos_x - start_x)*(chassis.pos_x - start_x) + (chassis.pos_y - start_y)*(chassis.pos_y - start_y) );
  if(distance_now > total_distance + ARRIVE_DISTANCE*1.5) 
  {
    count++;
    point_retrack_first_ref_flat = 0;
    return 0;
  }
  else return 1;
}

void point_collection_tracer(int point_num){
  int mid_control = 0;
  if(ENBALE_POINT_COLLECTION_TRACER == 1 && count < point_num ){
    if(count < point_num-1){
      if(count == 0){      
        //mid_control = point_tracer(zx_points_pos_x[count],zx_points_pos_y[count],zx_points_pos_x[count+1],zx_points_pos_y[count+1],speed_zx[count],speed_zx[count + 1] , max_speed_zx[count]);
        mid_control = point_tracer(zx_points_pos_x[count],zx_points_pos_y[count],zx_points_pos_x[count+1],zx_points_pos_y[count+1],650,650 , 500);
        //point_retrack(zx_points_pos_x[count],zx_points_pos_y[count],zx_points_pos_x[count+1],zx_points_pos_y[count+1]);
      }  
      if(count != 0){
        if(point_arrived_flag == 1){      
          //mid_control = point_tracer(zx_points_pos_x[count],zx_points_pos_y[count],zx_points_pos_x[count+1],zx_points_pos_y[count+1],speed_zx[count],speed_zx[count + 1], max_speed_zx[count]);
          mid_control = point_tracer(zx_points_pos_x[count],zx_points_pos_y[count],zx_points_pos_x[count+1],zx_points_pos_y[count+1],650,650, 500);
          //point_retrack(zx_points_pos_x[count],zx_points_pos_y[count],zx_points_pos_x[count+1],zx_points_pos_y[count+1]);
        }
      }
    }
    else{   //count == point_num-1
      mid_control = point_tracer(zx_points_pos_x[count-1],zx_points_pos_y[count-1],zx_points_pos_x[count] ,zx_points_pos_y[count], 150 , 0 , 200);
      //point_retrack(zx_points_pos_x[count-1],zx_points_pos_y[count-1],zx_points_pos_x[count] ,zx_points_pos_y[count]);
    }
    /* 以下为count++的控制 */
    if(mid_control == 1){
      point_count_control_flag ++;
    }
    else point_count_control_flag = 0;
    if(point_count_control_flag == 1){
      count++;
    }
  }
  
  if(count >= point_num){
    count = 0;
    chassis_gostraight_zx(0,0,chassis.angle,0);
    ENBALE_POINT_COLLECTION_TRACER = 0;
    uprintf(CMD_USART , "END TRACER !");
  }
}

int go_to_point_test_flag = 0;
void go_to_point_for_test(float point_x , float point_y){
  float toward_angle,toward_speed;
  float distance = sqrtf( (chassis.pos_x - point_x)*(chassis.pos_x - point_x) + (chassis.pos_y - point_y)*(chassis.pos_y - point_y) );
  if( go_to_point_test_flag == 1 && distance >= ARRIVE_DISTANCE ) //可以开始跑点，并且未到达目标点
  { 
    toward_angle = point_tracer_angle_return(point_x , point_y);
    
    toward_speed = speed_trapezium (point_x , point_y , 125 , 50 , 850);
    combine_vec = vec_create(toward_speed , toward_angle);
    //chassis_update();
    chassis_gostraight_zx( (int)combine_vec.x , combine_vec.y , 0 , 0);
    return;
  }
  else {
    if(go_to_point_test_flag != 1){ //不可跑点
      chassis_gostraight_zx( 0 , combine_vec.y , chassis.angle , 0);
      return;
    }
    if(distance < ARRIVE_DISTANCE) //到达点
    {
      //chassis_gostraight_zx( 50 , combine_vec.y , 0 , 0);
      //point_arrived_flag = 1;
      //point_tracer_flag = 1;
      first_time_controler = 1;
      go_to_point_test_flag = 0;
      //go_to_point_test_flag = 1;
      //point_retrack_first_ref_flat = 1;  
      uprintf(CMD_USART,"\r\ngo_point_arrived %f,%f\r\n",chassis.pos_x , chassis.pos_y);
      chassis_gostraight_zx(0 , 0 , 0 , 0);
      return;
    }
    
  }
  return;
}