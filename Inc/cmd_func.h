#ifndef __cmd_func_H
#define __cmd_func_H
#ifdef __cplusplus
 extern "C" {
#endif
#include "stm32f4xx_hal.h"
#include "main.h"
#include "usart.h"
#include "cmd.h"
#include "stdlib.h"
#include "maxon.h"
#include "math.h"
#include "chassis.h"
#include "track.h"
#include "vavle.h"
     
extern float point_x,point_y;


void cmd_hello_func(int argc,char *argv[]);  
void cmd_go_straight_func(int argc,char *argv[]);
void cmd_reset_vega(int argc, char *argv[]);
void cmd_angle_pid(int argc, char *argv[]);
void cmd_print_pos(int argc, char *argv[]);
void send_wang_can(int id,int num,char a);
void cmd_param_func(int argc, char *argv[]);
void cmd_debug_func(int argc, char *argv[]);
void cmd_robomaster_position(int argc, char *argv[]);
void cmd_r_sp_pid(int argc, char *argv[]);
void cmd_r_speed(int argc, char *argv[]);
void cmd_valve_open(int argc, char *argv[]);
void cmd_upper_func(int argc, char *argv[]);
void cmd_chassis_pid(int argc, char *argv[]);
void cmd_slide_pos(int argc,char *argv[]);
void cmd_open_box1(int argc,char *argv[]);


void cmd_chassis_gostraight(int argc,char *argv[]);
void cmd_print_angle(int argc,char *argv[]);

void cmd_test_angle(int argc,char *argv[]);
void cmd_start_point_tracer(int argc,char *argv[]);
void stop_flag(int argc,char *argv[]);
void cmd_point_tracer(int argc,char *argv[]);

void cmd_vega_init(int argc,char *argv[]);

void cmd_point_collection_tracer(int argc,char *argv[]);

extern int start_speed , final_speed , max_speed ;

extern float line_control_p_x,line_control_p_y,start_point_x,start_point_y;
void cmd_line_control_PID(int argc,char *argv[]);
void cmd_line_control_test(int argc,char *argv[]);
void cmd_set_boost_slow(int argc,char *argv[]);




extern float go_to_point_x,go_to_point_y;
void cmd_go_to_point_for_test(int argc,char *argv[]);
#ifdef __cplusplus
}
#endif
#endif /*__ cmd_func_H */