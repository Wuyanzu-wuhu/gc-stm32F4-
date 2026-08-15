#ifndef __PID_H
#define __PID_H
float PID_dianji(float error,float *integrate,float * last_error,float kp,float ki,float kd);
float PID_angle(float error,float *integrate,float * last_error,float kp,float ki,float kd);
float PID_vision1(float error,float *integrate,float * last_error,float kp,float ki,float kd);
float PID_vision2(float error,float *integrate,float * last_error,float kp,float ki,float kd);
#endif
