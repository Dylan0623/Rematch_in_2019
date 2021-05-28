#ifndef _MOTO_H
#define _MOTO_H



#define Kp0  20
#define Ki0  20

#define Kp10  20
#define Ki10  20
#define Kd10  10
#define Kp11  20
#define Ki11  20


int calculate_speed(int pwm_all);

void mode_0(int target,int speed);

void mode_1(int target1_ang,int target2_ang,int angel,int target_sp,int speed);
#endif

