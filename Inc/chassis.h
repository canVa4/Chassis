#ifndef __chassis_H
#define __chassis_H
#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f4xx_hal.h"
#include "main.h"
#include "maxon.h"
#include "usart.h"    
#include "math.h"  
#include "track.h"
#include <stdbool.h>
#include "vega_action.h"
#include "can.h"
#include "robomaster.h"
#include "vec.h"
#include "vavle.h"
#include "point_zx.h"
#include "points.h"
#include "pid.h"

#define MAX_SPEED_ZX 850

#define PI 3.1415926535
#define EXP 2.718281828
#define ORIGIN_X chassis_xpos[0] 
#define ORIGIN_Y chassis_ypos[0] 
  
#define ORIGIN_X1 chassis_xpos1[0] 
#define ORIGIN_Y1 chassis_ypos1[0] 
  
#define ORIGIN_X2 chassis_xpos2[0] 
#define ORIGIN_Y2 chassis_ypos2[0]
  
#define ORIGIN_X3 chassis_xpos3[0] 
#define ORIGIN_Y3 chassis_ypos3[0]
  
#define ORIGIN_X4 chassis_xpos4[0] 
#define ORIGIN_Y4 chassis_ypos4[0] 
  
#define ORIGIN_X5 chassis_xpos5[0] 
#define ORIGIN_Y5 chassis_ypos5[0] 
  
#define RED_ARENA 0
#define BLUE_ARENA 1
#define LEFT_ARENA 3
#define RIGHT_AREAN 4

#define LINE_CONTROL_NUMBER 100

#define ARRIVE_DISTANCE 0.02


extern int open_box;

extern int point_tracer_flag;
   
typedef struct
{
    float g_vega_pos_x; 
    float g_vega_pos_y;   
    float g_vega_angle;
	
    float pos_x,pos_y;
    float angle;

    int dis_vertical_camera;
    int dis_horizone_camera;
    int ccd_pos;
    
    int g_ispeed;
    float g_fangle;
    float g_fturn;
    //int now_speed[3];
    float speed_x;
    float speed_y;
    float last_pos_x;
    float last_pos_y;
    float now_speed;
    
    float origin_angle;
} Chassis;  


typedef enum{
  _FIXED_TRACK_MODE,
  _FIXED_TRACK_LASER_MODE,
  _FREE_TRACK_MODE,
  _FIXED_TRACK_PRESS_MODE,
  _FIXED_TRACK_OUTBRIDGE_MODE
}CHASSIS_DRIVE_MODE;

typedef enum{
    _SPEED_SHEET,
    _SPEED_SHEET_LASER_BRIDGE,
    _SPEED_SHEET_LASER_THROW,
    _SPEED_SHEET_OUTBRIDGE,
    _SPEED_CURVE,
    _SPEED_SHEET_PRESS
}CHASSIS_SPEED_MODE;

typedef enum{
  _UNPRESSED,
  _PRESSED
}CHASSIS_PRESS_MODE;

typedef struct
{
    int _delay_flag;
    int _position_flag;
    int _rotate_flag;
}FinishFlag;

typedef struct
{
    int _behind_motor_left_press;
    int _behind_motor_right_press;
    int _motor_both_press;
    int _grasp_bone_press;
    int _left_edge_press;
    int _right_edge_press;
    int _front_edge_left_press;
    int _front_edge_right_press;
    int _up_slide_press;
    int _down_slide_press;
    int _front_slide_up_press;
    int _front_slide_down_press;
}PressFlag;
  
typedef struct
{
    int _routeflag;
    int _sensor_flag;
    int _handle_flag;
    int _vega_ready_flag;
    PressFlag _press_flag;
}CtrlFlag;

typedef struct 
{
    CHASSIS_DRIVE_MODE m_DriveMode;
    CHASSIS_SPEED_MODE m_SpeedMode;
    FinishFlag m_FinishFlag;
    CtrlFlag m_CtrlFlag;
}CHASSIS_SIGNAL;

typedef struct 
{
    int mode;
    int lx;
    int ly;
    int rx;
    int ry;
    int btstate[10];
}CHASSIS_HANDLE;

//extern double chassis_speed_max ;
extern int chassis_arena;
extern CHASSIS_HANDLE ChassisHandle;
extern CHASSIS_SIGNAL ChassisSignal;
extern float ERR_angle_m3 , ERR_angle_m1 , ERR_angle_m0  ;
extern float chassis_turn_angle_KP;
extern float chassis_turn_angle_KD;

extern float action_init_posx;
extern float action_init_posy;
extern float action_init_angle;

extern float catch_bone_modify_x;
extern float catch_bone_modify_y;

extern float scope_path;
extern int scan_wave;
//extern int sensor_flag;
//extern int four_feet_ok;
extern float laser_dis;
extern float chassis_distance_KP;
extern float chassis_distance_KD;
extern float chassis_distance_KI;
extern float chassis_distance_2_KP;
extern float chassis_speed_max;//底盘速度曲线最大速度
extern int chassis_posnum;
extern int chassis_poscnt;
extern float param_a;
extern float param_b;
extern float motor0_error;
extern float motor1_error;
extern float motor2_error;

extern int Chassis_State;
extern int delay_num_const;
extern float slow_turn;
extern float quickturn_KP;
extern float quickturn_KD;  
extern Chassis chassis;
extern double chassis_vector_d;
extern double chassis_vector_vx;
extern float test_target_speed;
extern int micro_switch_test_flag;

void chassis_init(void);
float chassis_angle_subtract(float a, float b);
void chassis_init_pos(float x,float y);
void chassis_update(void);
void chassis_gostraight(int speed , float angle, float turn, int is_handle); 
void chassis_calculate_dis_matrix();
extern void chassis_exe();   
void chassis_handle_control();
void chassis_handle(CanRxMsgTypeDef* pRxMsg);
void chassis_rocker(CanRxMsgTypeDef* pRxMsg);  
void press_rcv_callback(CanRxMsgTypeDef* pRxMsg);
float chassis_PID_Sensor_Control(float target_distance);
void chassis_automata();
void chassis_go_track(CHASSIS_DRIVE_MODE theMode, CHASSIS_SPEED_MODE theSpeedMode);
float speed_to_targetpoint(int now_pos,int target_pos);
float chassis_angle_pid_ctrl(float target_x_next, float target_y_next);   
void chassis_modify_pos(float x[],float y[],float x0,float y0);
    
void chassis_gostraight1(int speed , float angle, float turn, int is_handle);

void chassis_modify_handle_control();

float point_tracer_angle_return( float point_x , float point_y );
int point_tracer (float start_x , float start_y ,float point_x , float point_y , int start_speed , int final_speed , int max_speed);

void point_collection_tracer(int point_num);

vec line_control(float p1_x , float p1_y ,float p2_x,float p2_y, float max_speed);
extern PID_Struct_zx line_control_PID;
extern int total_line_control_flag;
extern int line_control_flag_first;

extern int ENBALE_POINT_COLLECTION_TRACER;

extern int first_time_controler;
extern float Boost_Period ;
extern float Slow_Period ;

extern int count;
extern int point_retrack_first_ref_flag;

void state_reset();

void chassis_gostraight_zx(int speed , float angle, float turn, int is_handle);
extern int go_to_point_test_flag;
void go_to_point_for_test(float point_x , float point_y);
#ifdef __cplusplus
}
#endif
#endif /*__ chassis_H */