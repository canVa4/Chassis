#ifndef __track_H
#define __track_H
#ifdef __cplusplus
extern "C" {
#endif
    
#include <math.h>
#include "chassis.h"
#include "points.h"
#include "cmd_func.h"




extern float chassis_speed_max;
extern int chassis_posnum;
extern float error_x;
extern float error_y;
extern float modify_x;
extern float modify_y;
extern float modify_angle;
extern int track_max_speed;
extern float track_test_offset;
//extern void track_init(int flag_track);
extern uint8_t track_section_flag;
int chassis_throw_thrice_first();
int chassis_throw_thrice_second();
int chassis_throw_thrice_third();
extern int chassis_calbration_and_craw();
extern int chassis_winding_and_deliver();
extern int chassis_winding_and_deliver_1();
extern int chassis_winding_and_deliver_2();
    
    
    
    
        
#ifdef __cplusplus
}
#endif
#endif /*__ track_H */