#ifndef _pid_H_
#define _pid_H_

typedef struct {
	float KP;
	float KI;
	float KD;
	float i;
	float last_err;
	float i_max;
	float last_d;
	float I_TIME;
} PID_Struct;

typedef struct {
	float angular_x;
	float angular_y;
	float angular_z;

	float angle_roll;
	float angle_yaw;
	float angle_pitch;
} Ring_Param_struct;

#define Limit(value,max)     if(value>max)value=max;else if(value<-max)value=-max

float PID_Release(PID_Struct *PID, float target, float now);
void reset_PID(PID_Struct * s);
void PID_init();

#endif
