#ifndef __points_H
#define __points_H
#ifdef __cplusplus
extern "C" {
#endif
    
#include <math.h>
#include "chassis.h"

#define INPUT_NUM 7  //输入控制点个数
#define NUM 3     //两个控制点间的采样个数
#define OUT_NUM NUM*(INPUT_NUM-1) //总输出点数 
#define NUM_POINTS 200////修改！！！！！！！！！！！！
#define NUM_POINTS1 400
#define NUM_POINTS2 200
  
extern float chassis_xpos[NUM_POINTS];
extern float chassis_ypos[NUM_POINTS];
extern float chassis_speed[NUM_POINTS];
extern float chassis_speed_dir[NUM_POINTS];

extern float chassis_xpos1[NUM_POINTS1];
extern float chassis_ypos1[NUM_POINTS1];
extern float chassis_speed1[NUM_POINTS1];
extern float chassis_speed_dir1[NUM_POINTS1];

extern float chassis_xpos2[NUM_POINTS2];
extern float chassis_ypos2[NUM_POINTS2];
extern float chassis_speed2[NUM_POINTS2];
extern float chassis_speed_dir2[NUM_POINTS2];

extern float chassis_xpos3[NUM_POINTS];
extern float chassis_ypos3[NUM_POINTS];
extern float chassis_speed_dir3[NUM_POINTS];

extern float chassis_xpos4[NUM_POINTS1];
extern float chassis_ypos4[NUM_POINTS1];
extern float chassis_speed_dir4[NUM_POINTS1];

extern float chassis_xpos5[NUM_POINTS2];
extern float chassis_ypos5[NUM_POINTS2];
extern float chassis_speed_dir5[NUM_POINTS2];
    

    
#ifdef __cplusplus
}
#endif
#endif /*__ points_H */