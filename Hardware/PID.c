#include "string.h"
#include "stdio.h"
#include "stdarg.h"


float PID_dianji(float error,float *integrate,float * last_error,float kp,float ki,float kd)//记住限幅这里的问题
{    float P,I,D=0.0f;
        float output=0;
       P=kp*error;
         *integrate=*integrate+error;
       
         if(*integrate>8000)
         { *integrate=8000;
         }
         else if(*integrate<-8000)
         { *integrate=-8000;
         }
         I=ki* *integrate;
         D=kd*(error-*last_error);
         output=(P+I+D);
         if (output>8000)
         {  output=8000;
         
         }
         else if (output<-8000)
         {  output=-8000;
         
         }
          *last_error=error;    
    return output;         
}
float PID_angle(float error,float *integrate,float * last_error,float kp,float ki,float kd)
{    float P,I,D=0.0f;
        float output=0;
       P=kp*error;
         *integrate=*integrate+error;
       
         if(*integrate>0.5f)
         { *integrate=0.5;
         }
         else if(*integrate<-0.5f)
         { *integrate=-0.5;
         }
         I=ki* *integrate;
         D=kd*(error-*last_error);
         output=(P+I+D);
         if (output>100)
         {  output=100;
         
         }
         else if (output<-100)
         {  output=-100;
         
         }
          *last_error=error;    
    return output;         
}
float PID_vision1(float error,float *integrate,float * last_error,float kp,float ki,float kd)//第一关pid限幅
{    float P,I,D=0.0f;
        float output=0;
       P=kp*error;
         *integrate=*integrate+error;
       
         if(*integrate>0.5f)
         { *integrate=0.5;
         }
         else if(*integrate<-0.5f)
         { *integrate=-0.5;
         }
         I=ki* *integrate;
         D=kd*(error-*last_error);
         output=(P+I+D);
         if (output>3)
         {  output=3;
         
         }
         else if (output<-3)
         {  output=-3;
         
         }
          *last_error=error;    
    return output;         
}
float PID_vision2(float error,float *integrate,float * last_error,float kp,float ki,float kd)//第二关pid限幅
{    float P,I,D=0.0f;
        float output=0;
       P=kp*error;
         *integrate=*integrate+error;
       
         if(*integrate>0.5f)
         { *integrate=0.5;
         }
         else if(*integrate<-0.5f)
         { *integrate=-0.5;
         }
         I=ki* *integrate;
         D=kd*(error-*last_error);
         output=(P+I+D);
         if (output>3)
         {  output=3;
         
         }
         else if (output<-3)
         {  output=-3;
         
         }
          *last_error=error;    
    return output;         
}