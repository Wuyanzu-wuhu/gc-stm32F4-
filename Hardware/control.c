#include "stm32f4xx.h"

#include "HWT101.h"
#include "PID.h"
#include "Delay.h"
#include "math.h"
#include "control.h"
#include "MC.h"
#include "K230.h"
#include "SaoM.h"
#include "Servo.h"
#include "lcd.h"

DingChen_State_Type DingChen_State = Scan_Qr;
PID_State_Type PID_State = Scan_Qr_S;
Scan_Qr_State_Type Scan_Qr_State=Scan_Qr_Idle;
Grap1_State_Type Grap1_State=Grap1_Idle;
Grap_and_put1_State_Type  Grap_and_put1_State=Grap_and_put1_Idle;
put1_State_Type put1_State=put1_Idle;
Grap2_State_Type  Grap2_State=Grap2_Idle;
Grap_and_put2_State_Type Grap_and_put2_State= Grap_and_put2_Idle;
put2_State_Type put2_State=put2_Idle;




Scan_Qr_Data_Type Scan_Qr_Data = {0};
Grap1_Data_Type Grap1_Data= {0};
Grap_and_put1_Data_Type Grap_and_put1_Data= {0};
put1_Data_Type put1_Data= {0};
Grap2_Data_Type  Grap2_Data= {0};
Grap_and_put2_Data_Type Grap_and_put2_Data= {0};
put2_Data_Type put2_Data= {0};

uint8_t PID_Task_Completed_Flag = 0;
uint8_t Special_Task_Completed_Flag= 0;
uint8_t Delay_Flag= 0;
uint8_t Delay_Unit;       //延时单位，延时时间=延时单位*时间片轮询时间

PID_Mode_Type PID_Mode=Straight_and_Angle;

static char buf1[3];
static char buf2[1];
static char buf3[3];
/*系统定时器计数变量*/
volatile uint32_t System_Tick_10ms=0;
/*关于PID的变量区*/




void DingChen_State_Function(void *arg)
{  
    switch(DingChen_State)
    {
        case Scan_Qr:
            // PID_Control();           //PID的共用函数
            // ScanQR_StateMachine();       //独特函数
				   PID_State_Function();
				Scan_Qr_State_Function();
            
            if(PID_Task_Completed_Flag == 1 && Scan_Qr_State == Scan_Qr_Idle) {
                Scan_Qr_State = Scan_Qr_Start;
                PID_Task_Completed_Flag = 0;
            }
            else if(Special_Task_Completed_Flag == 1) {
                DingChen_State = Grap1;
                PID_State = Grap1_S;
                Scan_Qr_State = Scan_Qr_Idle;  
				  			Special_Task_Completed_Flag = 0;  
							  PID_Task_Completed_Flag = 0;
                
                // memset(&scanData, 0, sizeof(scanData));
            }
            break;

        case Grap1:
            // PID_Control();
            // Grap_StateMachine();
             PID_State_Function();
				    Grap1_State_Function();
            if(PID_Task_Completed_Flag == 1 && Grap1_State == Grap1_Idle) {
                Grap1_State = Grap1_Start;
                PID_Task_Completed_Flag = 0;
            }
            else if(Special_Task_Completed_Flag == 1) {
                DingChen_State = Grap_and_put1;
                PID_State = Grap_and_put1_S;
                Grap1_State =  Grap1_Idle;
                Special_Task_Completed_Flag = 0;
							 PID_Task_Completed_Flag = 0;
                // memset(&grapData, 0, sizeof(grapData));
            }
            break;

        case Grap_and_put1:
            // PID_Control();
            // Grap_and_put1_StateMachine();
             PID_State_Function();
				  Grap_and_put1_State_Function();
            if(PID_Task_Completed_Flag == 1 && Grap_and_put1_State == Grap_and_put1_Idle) {
                Grap_and_put1_State = Grap_and_put1_Start;
                PID_Task_Completed_Flag = 0;
            }
            else if(Special_Task_Completed_Flag == 1) {
                DingChen_State = put1;
                PID_State = put1_S;
                Grap_and_put1_State = Grap_and_put1_Idle;
                Special_Task_Completed_Flag = 0;
							 PID_Task_Completed_Flag = 0;
                // memset(&Grap_and_put1Data, 0, sizeof(Grap_and_put1Data));
            }
            break;

        case put1:
            // PID_Control();
            // put1_StateMachine();
             PID_State_Function();
				    put1_State_Function();
            if(PID_Task_Completed_Flag == 1 && put1_State == put1_Idle) {
                put1_State =put1_Start;
                PID_Task_Completed_Flag = 0;
            }
            else if(Special_Task_Completed_Flag == 1) {
                DingChen_State =Grap2;
                PID_State =Grap2_S;
                put1_State = put1_Idle;
                Special_Task_Completed_Flag = 0;
							 PID_Task_Completed_Flag = 0;
                // memset(&put1Data, 0, sizeof(put1Data));
            }
            break;
          case Grap2:
            // PID_Control();
            // Grap2_StateMachine();
             PID_State_Function();
				    Grap2_State_Function();
            if(PID_Task_Completed_Flag == 1 && Grap2_State == Grap2_Idle) {
                Grap2_State = Grap2_Start;
                PID_Task_Completed_Flag = 0;
            }
            else if(Special_Task_Completed_Flag == 1) {
                DingChen_State = Grap_and_put2;
                PID_State =Grap_and_put2_S;
                Grap2_State = Grap2_Idle;
                Special_Task_Completed_Flag = 0;
							 PID_Task_Completed_Flag = 0;
                // memset(&Grap2Data, 0, sizeof(Grap2Data));
            }
            break;
					 case Grap_and_put2:
            // PID_Control();
            // Grap_and_put2_StateMachine();
             PID_State_Function();
				Grap_and_put2_State_Function();
            if(PID_Task_Completed_Flag == 1 && Grap_and_put2_State == Grap_and_put2_Idle) {
                Grap_and_put2_State = Grap_and_put2_Start;
                PID_Task_Completed_Flag = 0;
            }
            else if(Special_Task_Completed_Flag == 1) {
                DingChen_State = put2;
                PID_State =put2_S;
                Grap_and_put2_State = Grap_and_put2_Idle;
                Special_Task_Completed_Flag = 0;
							 PID_Task_Completed_Flag = 0;
                // memset(&Grap_and_put2Data, 0, sizeof(Grap_and_put2Data));
            }
            break;
					case put2:
            // PID_Control();
            // put2_StateMachine();
             PID_State_Function();
				put2_State_Function();
            if(PID_Task_Completed_Flag == 1 && put2_State == put2_Idle) {
                put2_State = put2_Start;
                PID_Task_Completed_Flag = 0;
            }
            else if(Special_Task_Completed_Flag == 1) {
                DingChen_State = final;
                PID_State =final_S;
                put2_State = put2_Idle;
                Special_Task_Completed_Flag = 0;
							 PID_Task_Completed_Flag = 0;
                // memset(&put2Data, 0, sizeof(put2Data));
            }
            break;
					case final:
            // PID_Control();
            // put1_StateMachine();
             PID_State_Function();
				
            break;
        default:
            break;
    }
}




void PID_State_Function(void)
{
    switch(PID_State)
    {
        
         case Scan_Qr_S:{
            static const PID_Data_Type seq[] = {
             
        //  {Straight_and_Angle,   0,    0,  0,    15, 6,   120, 0,  80},
			
	//{Straight_and_Angle,   0,    0,  0,    13, 8,   120, 0,  80},
	{Straight_and_Angle,   0,    0,  0,    13, 6,   120, 0,  80},
							
// {Straight_and_Angle, 0,    0,  0,  10,  10,  20, 2,  20},
   // {Straight_and_Angle, 0,    0,  0,  15,   0,  36, 6,  30},
            };
     
            static uint8_t S_Task_State_Number= 0;
            static uint8_t Continue_Flag = 0;
            if (Continue_Flag == 0 && S_Task_State_Number < sizeof(seq)/sizeof(seq[0])) {
  
            
            Chassis_Core(seq[S_Task_State_Number].type, seq[S_Task_State_Number].target_yaw, seq[S_Task_State_Number].vx1, seq[S_Task_State_Number].vy1,seq[S_Task_State_Number].vx2,seq[S_Task_State_Number].vy2,
                       seq[S_Task_State_Number].t_acc, seq[S_Task_State_Number].t_cru, seq[S_Task_State_Number].t_dec,1);
           Continue_Flag = 1;
        }
        else if ( Continue_Flag == 1) {
             Chassis_Core(Straight_and_Angle, 0, 0, 0, 0, 0, 0, 0, 0, 0); // 标志位为0根本不改麦轮函数的任何变量
            
            if (g_sc_state == SC_IDLE) {
               S_Task_State_Number++;
                 Continue_Flag  = (S_Task_State_Number <  sizeof(seq)/sizeof(seq[0])) ? 0 : 2;
            }
        }
        else if (Continue_Flag == 2) {
            Chassis_Core(Straight_and_Angle, 0, 0, 0, 0, 0, 0, 0, 0, 0);  // 全部结束后保险发一帧0速
				PID_Task_Completed_Flag = 1;
			
        }  
        }    
				
            break;
            
        case Grap1_S:
           {
            static const PID_Data_Type seq[] = {
               	{Straight_and_Angle,  0,   0 ,0 ,  30,  -3,   60, 20,  60}, //10  -1
  //     	 {Straight_and_Angle, 0,  0, 0,  20, -1.5,  40, 6, 40},
            };
     
            static uint8_t S_Task_State_Number= 0;
            static uint8_t Continue_Flag = 0;
            if (Continue_Flag == 0 && S_Task_State_Number < sizeof(seq)/sizeof(seq[0])) {
  
            
            Chassis_Core(seq[S_Task_State_Number].type, seq[S_Task_State_Number].target_yaw, seq[S_Task_State_Number].vx1, seq[S_Task_State_Number].vy1,seq[S_Task_State_Number].vx2,seq[S_Task_State_Number].vy2,
                       seq[S_Task_State_Number].t_acc, seq[S_Task_State_Number].t_cru, seq[S_Task_State_Number].t_dec,1);
           Continue_Flag = 1;
        }
        else if ( Continue_Flag == 1) {
             Chassis_Core(Straight_and_Angle, 0, 0, 0, 0, 0, 0, 0, 0, 0);  // 标志位为0根本不改麦轮函数的任何变量
            
            if (g_sc_state == SC_IDLE) {
               S_Task_State_Number++;
                 Continue_Flag  = (S_Task_State_Number <  sizeof(seq)/sizeof(seq[0])) ? 0 : 2;
            }
        }
        else if (Continue_Flag == 2) {
            Chassis_Core(Straight_and_Angle, 0, 0, 0, 0, 0, 0, 0, 0, 0);  // 全部结束后保险发一帧0速
					PID_Task_Completed_Flag = 1;
        }  
        }    
				
            break;
            
        case  Grap_and_put1_S:
             {
            static const PID_Data_Type seq[] = { 	
//{Straight_and_Angle,  0, 0, 0,  -10,  0,   60, 0, 0},
//{Straight_and_Angle,  -90, -10, 0, 0 ,   15,  120, 200, 0 }, //120, 280, 100   //140, 270, 100
//{Straight_and_Angle,  -180, 0, 10, 2 ,  0,   100, 0 ,60 },   //120 0 100          //100, 0, 60

//{Straight_and_Angle,  0, 0, 0,  -10,  0,   80, 0, 0},
//{Straight_and_Angle,  -90, -10, 0, 0 ,   15,  120, 200, 0 }, //120, 280, 100   //140, 270, 100
//{Straight_and_Angle,  -180, 0, 10, 2 ,  0,   100, 0 ,60 },   //120 0 100          //100, 0, 60
//				{Straight_and_Angle,  -180, 0, 0, -4, 2 ,   80, 0 ,90 }, 		
				
//				{Straight_and_Angle,  0, 0, 0,  -10,  0,   80, 5, 0},
//        {Straight_and_Angle,  -90, -10, 0, 0 ,   20,  120, 120, 0 }, //120, 280, 100   //140, 270, 100
//        {Straight_and_Angle,  -180, 0, 20, 0 ,  0,   100, 0 ,40 },   //120 0 100          //100, 0, 60
					
	
//				{Straight_and_Angle,  0, 0, 0,  -10,  0,   80, 5, 0},
//        {Straight_and_Angle,  -90, -10, 0, 0 ,   25,  80, 125, 0 }, //80 125 0
//        {Straight_and_Angle,  -180, 0, 25, 0 ,  0,   75, 0 ,25 },   //120 0 100          //100, 0, 60
							
							{Straight_and_Angle,  0, 0, 0,  -14,  0,  70, 5, 0},
        {Straight_and_Angle,  -90, -10, 0, 0 ,   25,  100, 100, 0 }, //80 125 0
        {Straight_and_Angle,  -180, 0, 25, 0 ,  0,   45, 0 ,25 },   //120 0 100          //100, 0, 60
							
            };
     
            static uint8_t S_Task_State_Number= 0;
            static uint8_t Continue_Flag = 0;
            if (Continue_Flag == 0 && S_Task_State_Number < sizeof(seq)/sizeof(seq[0])) {
  
            
            Chassis_Core(seq[S_Task_State_Number].type, seq[S_Task_State_Number].target_yaw, seq[S_Task_State_Number].vx1, seq[S_Task_State_Number].vy1,seq[S_Task_State_Number].vx2,seq[S_Task_State_Number].vy2,
                       seq[S_Task_State_Number].t_acc, seq[S_Task_State_Number].t_cru, seq[S_Task_State_Number].t_dec,1);
           Continue_Flag = 1;
        }
        else if ( Continue_Flag == 1) {
             Chassis_Core(Straight_and_Angle, 0, 0, 0, 0, 0, 0, 0, 0, 0);  // 标志位为0根本不改麦轮函数的任何变量
            
            if (g_sc_state == SC_IDLE) {
               S_Task_State_Number++;
                 Continue_Flag  = (S_Task_State_Number <  sizeof(seq)/sizeof(seq[0])) ? 0 : 2;
            }
        }
        else if (Continue_Flag == 2) {
            Chassis_Core(Straight_and_Angle, 0, 0, 0, 0, 0, 0, 0, 0, 0);  // 全部结束后保险发一帧0速
					PID_Task_Completed_Flag = 1;
        }  
        }    
				
            break;
            
            
        case put1_S:
            {
            static const PID_Data_Type seq[] = {  
{Straight_and_Angle,  -180, 0, 0, 25 ,  -2,   80, 0, 0},
{Straight_and_Angle,  90, 25, -2, 0 ,  -25,   100, 0, 60},


//{Straight_and_Angle,  -180, 0, 0, 17 ,  0,   160, 0, 0},
//{Straight_and_Angle,  88, 20, 0, 0 ,  -15,   100, 65, 100},
						//120 0 100,
            };
     
            static uint8_t S_Task_State_Number= 0;
            static uint8_t Continue_Flag = 0;
            if (Continue_Flag == 0 && S_Task_State_Number < sizeof(seq)/sizeof(seq[0])) {
  
            
            Chassis_Core(seq[S_Task_State_Number].type, seq[S_Task_State_Number].target_yaw, seq[S_Task_State_Number].vx1, seq[S_Task_State_Number].vy1,seq[S_Task_State_Number].vx2,seq[S_Task_State_Number].vy2,
                       seq[S_Task_State_Number].t_acc, seq[S_Task_State_Number].t_cru, seq[S_Task_State_Number].t_dec,1);
           Continue_Flag = 1;
        }
        else if ( Continue_Flag == 1) {
             Chassis_Core(Straight_and_Angle, 0, 0, 0, 0, 0, 0, 0, 0, 0);  // 标志位为0根本不改麦轮函数的任何变量
            
            if (g_sc_state == SC_IDLE) {
               S_Task_State_Number++;
                 Continue_Flag  = (S_Task_State_Number <  sizeof(seq)/sizeof(seq[0])) ? 0 : 2;
            }
        }
        else if (Continue_Flag == 2) {
            Chassis_Core(Straight_and_Angle, 0, 0, 0, 0, 0, 0, 0, 0, 0);  // 全部结束后保险发一帧0速
					PID_Task_Completed_Flag = 1;
        }  
        }    
				
            break;
      case Grap2_S:
            {
            static const PID_Data_Type seq[] = { 
							
 {Straight_and_Angle,  90, 0, 0,  -2,  -25,   120, 0, 0},
 {Straight_and_Angle,  0, -2, -25,  -15,  0,   60, 0,90},
            };
     
            static uint8_t S_Task_State_Number= 0;
            static uint8_t Continue_Flag = 0;
            if (Continue_Flag == 0 && S_Task_State_Number < sizeof(seq)/sizeof(seq[0])) {
  
            
            Chassis_Core(seq[S_Task_State_Number].type, seq[S_Task_State_Number].target_yaw, seq[S_Task_State_Number].vx1, seq[S_Task_State_Number].vy1,seq[S_Task_State_Number].vx2,seq[S_Task_State_Number].vy2,
                       seq[S_Task_State_Number].t_acc, seq[S_Task_State_Number].t_cru, seq[S_Task_State_Number].t_dec,1);
          Continue_Flag = 1;
        }
        else if ( Continue_Flag == 1) {
             Chassis_Core(Straight_and_Angle, 0, 0, 0, 0, 0, 0, 0, 0, 0);  // 标志位为0根本不改麦轮函数的任何变量
            
            if (g_sc_state == SC_IDLE) {
               S_Task_State_Number++;
                 Continue_Flag  = (S_Task_State_Number <  sizeof(seq)/sizeof(seq[0])) ? 0 : 2;
            }
        }
        else if (Continue_Flag == 2) {
            Chassis_Core(Straight_and_Angle, 0, 0, 0, 0, 0, 0, 0, 0, 0);  // 全部结束后保险发一帧0速
					PID_Task_Completed_Flag = 1;
        }  
        }    
				
            break;
                   
        case Grap_and_put2_S:
            {
            static const PID_Data_Type seq[] = { 


					{Straight_and_Angle,  0, 0, 0,  -14,  0,   70, 5, 0},
        {Straight_and_Angle,  -90, -10, 0, 0 ,   25,  100, 100, 0 }, //80 125 0
        {Straight_and_Angle,  -180, 0, 25, 0 ,  0,   45, 0 ,25 },   //120 0 100          //100, 0, 60
							
            };
     
            static uint8_t S_Task_State_Number= 0;
            static uint8_t Continue_Flag = 0;
            if (Continue_Flag == 0 && S_Task_State_Number < sizeof(seq)/sizeof(seq[0])) {
  
            
            Chassis_Core(seq[S_Task_State_Number].type, seq[S_Task_State_Number].target_yaw, seq[S_Task_State_Number].vx1, seq[S_Task_State_Number].vy1,seq[S_Task_State_Number].vx2,seq[S_Task_State_Number].vy2,
                       seq[S_Task_State_Number].t_acc, seq[S_Task_State_Number].t_cru, seq[S_Task_State_Number].t_dec,1);
           Continue_Flag = 1;
        }
        else if ( Continue_Flag == 1) {
             Chassis_Core(Straight_and_Angle, 0, 0, 0, 0, 0, 0, 0, 0, 0); // 标志位为0根本不改麦轮函数的任何变量
            
            if (g_sc_state == SC_IDLE) {
               S_Task_State_Number++;
                 Continue_Flag  = (S_Task_State_Number <  sizeof(seq)/sizeof(seq[0])) ? 0 : 2;
            }
        }
        else if (Continue_Flag == 2) {
            Chassis_Core(Straight_and_Angle, 0, 0, 0, 0, 0, 0, 0, 0, 0);  // 全部结束后保险发一帧0速
					PID_Task_Completed_Flag = 1;
        }  
        }    
				
            break;
            
        case put2_S:
            {
            static const PID_Data_Type seq[] = { 
{Straight_and_Angle,  -180, 0, 0, 25 ,  -2,   80, 0, 0},
{Straight_and_Angle,  90, 25, -2, 0 ,  -25,   100, 0, 60},


            };
     
            static uint8_t S_Task_State_Number= 0;
            static uint8_t Continue_Flag = 0;
            if (Continue_Flag == 0 && S_Task_State_Number < sizeof(seq)/sizeof(seq[0])) {
  
            
            Chassis_Core(seq[S_Task_State_Number].type, seq[S_Task_State_Number].target_yaw, seq[S_Task_State_Number].vx1, seq[S_Task_State_Number].vy1,seq[S_Task_State_Number].vx2,seq[S_Task_State_Number].vy2,
                       seq[S_Task_State_Number].t_acc, seq[S_Task_State_Number].t_cru, seq[S_Task_State_Number].t_dec,1);
           Continue_Flag = 1;
        }
        else if ( Continue_Flag == 1) {
             Chassis_Core(Straight_and_Angle, 0, 0, 0, 0, 0, 0, 0, 0, 0); // 标志位为0根本不改麦轮函数的任何变量
            
            if (g_sc_state == SC_IDLE) {
               S_Task_State_Number++;
                 Continue_Flag  = (S_Task_State_Number <  sizeof(seq)/sizeof(seq[0])) ? 0 : 2;
            }
        }
        else if (Continue_Flag == 2) {
            Chassis_Core(Straight_and_Angle, 0, 0, 0, 0, 0, 0, 0, 0, 0);  // 全部结束后保险发一帧0速
				PID_Task_Completed_Flag = 1;
        }  
        }    
				
            break;
            
            
        case final_S:
            {
            static const PID_Data_Type seq[] = {
							{Straight_and_Angle,  90, 0, 0,  -2,  -25,   120, 0, 0},
 {Straight_and_Angle,  0, -2, -25,  -20,  0,   50, 100,0},
 {Straight_and_Angle,   0,    -20,  0,    -14, -3,   120, 0,  80},
							
               
            };
     
            static uint8_t S_Task_State_Number= 0;
            static uint8_t Continue_Flag = 0;
            if (Continue_Flag == 0 && S_Task_State_Number < sizeof(seq)/sizeof(seq[0])) {
  
            
            Chassis_Core(seq[S_Task_State_Number].type, seq[S_Task_State_Number].target_yaw, seq[S_Task_State_Number].vx1, seq[S_Task_State_Number].vy1,seq[S_Task_State_Number].vx2,seq[S_Task_State_Number].vy2,
                       seq[S_Task_State_Number].t_acc, seq[S_Task_State_Number].t_cru, seq[S_Task_State_Number].t_dec,1);
           Continue_Flag = 1;
        }
        else if ( Continue_Flag == 1) {
             Chassis_Core(Straight_and_Angle, 0, 0, 0, 0, 0, 0, 0, 0, 0);  // 标志位为0根本不改麦轮函数的任何变量
            
            if (g_sc_state == SC_IDLE) {
               S_Task_State_Number++;
                 Continue_Flag  = (S_Task_State_Number <  sizeof(seq)/sizeof(seq[0])) ? 0 : 2;
            }
        }
        else if (Continue_Flag == 2) {
            Chassis_Core(Straight_and_Angle, 0, 0, 0, 0, 0, 0, 0, 0, 0);  // 全部结束后保险发一帧0速
					PID_Task_Completed_Flag = 1;
        }  
        }    
				
            break;
            

      
        default:
            break;
    }
}

void Scan_Qr_State_Function(void)
{  
    static uint8_t cnt = 0;
    if (++cnt < 5)
        return;      
  

    switch(Scan_Qr_State)
    {
        case Scan_Qr_Idle:
				
            break;
            
        case Scan_Qr_Start:
        {      
					
					
					
			Servo_SmoothSetAngle_CH3(6, 100);
					
				 if(SaoM_complete_Flag == 1) {
					ProcessReceivedExpression();
							sprintf(buf1, "%d%d%d", task1, task2, task3);
    LCD_ShowString(0, 0, (u8*)buf1, BLACK, WHITE, 100, 0);  // 第0行显示
								sprintf(buf2, "+");
    LCD_ShowString(150, 60, (u8*)buf2, BLACK, WHITE,100 , 0);  // 第0行显示
						
					sprintf(buf3, "%d%d%d",  task4, task5, task6);
    LCD_ShowString(150, 150, (u8*)buf3, BLACK, WHITE, 100, 0);  // 第0行显示
					
                Scan_Qr_State = Scan_Qr_Success;
            }
        
            
            break;
        }
            
        case Scan_Qr_Success:
            Special_Task_Completed_Flag = 1;          // 通知上层状态机任务完成
            break;
            
        default:
            break;
    }
}
void Grap1_State_Function(void)
{      static uint8_t cnt = 0;
    if (++cnt < 5)
			return;    
		 // cnt = 0;  
    switch(Grap1_State)
    {
        case Grap1_Idle:
					
            break;
            
      case Grap1_Start:
        {
            static uint8_t inited = 0;
            if (!inited) {
        
               Servo_SmoothSetAngle_CH3(180,100);
                Delay_ms(50);
							  Servo_SmoothSetAngle_CH1(30,1);
					     Delay_ms(80);
							Servo_SmoothSetAngle_CH2(0,1);
										     Delay_ms(80);
                    char qr_buf[8];
                    qr_buf[0] = '0' + task1;
                    qr_buf[1] = '0' + task2;
                    qr_buf[2] = '0' + task3;
                    qr_buf[3] = '+';
                    qr_buf[4] = '0' + task4;
                    qr_buf[5] = '0' + task5;
                    qr_buf[6] = '0' + task6;
                    qr_buf[7] = '\0';
                    K230_SendQR(qr_buf);
   
     
							g_visual_stop_flag = 0; g_visual_dx = 0; g_visual_dy = 0;
               
                inited = 1;
            }
            Grap1_State = Grap1_Visual_1;
            inited = 0;
            break;
        }
            
        case Grap1_Visual_1:
        {
          
                
            
              Grap1_Wei_Tiao_Function(0);        
            if (g_visual_stop_flag) {         
                g_visual_stop_flag = 0; g_visual_dx = 0; g_visual_dy = 0;
                uint8_t addr_map[4] = {0x05, 0x08, 0x06, 0x09};
                uint8_t i;
                for (i = 0; i < 4; i++) {
                    ZDT_Speed_Control(addr_map[i], straight, 0);
                    Delay_ms(1);
                }
                ZDT_DJ_TongBu_Control();          
                Delay_ms(1);
             
           
             
                
                       
                Grap1_State =  Grap1_Action_1;
            }
            break;
        }
            
        case Grap1_Action_1:
        {          Delay_ms(1); 
                ZDT_JueDui_Position_Control(0x02, behind, 200, 200, 2000);
            Delay_ms(100);
            ZDT_JueDui_Position_Control(0x03, behind, 700, 250, 2700);
            Delay_ms(500);
            Servo_SmoothSetAngle_CH1(0, 1);
            Delay_ms(110);
            ZDT_JueDui_Position_Control(0x03, behind, 700, 250, 0);
            Delay_ms(500);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, 1800);
            Delay_ms(100);
            Servo_SmoothSetAngle_CH3(40, 70);
            Delay_ms(800);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, 2081);
            Delay_ms(100);
            ZDT_JueDui_Position_Control(0x03, behind, 700, 250, 2101);
            Delay_ms(500);
            Servo_SmoothSetAngle_CH1(30, 1);
            Delay_ms(110);
            ZDT_JueDui_Position_Control(0x03, behind, 700, 250, 0);
            Delay_ms(500);
            Servo_SmoothSetAngle_CH3(180, 70);
            Delay_ms(800);
					
					
                Grap1_State = Grap1_Next_1;
                
            
            break;
        }
            
        case Grap1_Next_1:
        {    
					Servo_SmoothSetAngle_CH2(120, 1);
            Delay_ms(110);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, 0);   /* ← 50,0 → 200,200 */
            Delay_ms(100);
            K230_SendTaskCmd(0x3F);
            Delay_ms(50);
            	g_visual_stop_flag = 0; g_visual_dx = 0; g_visual_dy = 0;
            Grap1_State = Grap1_Visual_2;
             
             
              
            
            break;
        }
            
        case Grap1_Visual_2:
        {
            
            if (g_visual_stop_flag) {        // 收到 "0\n"，到位了
              g_visual_stop_flag = 0; g_visual_dx = 0; g_visual_dy = 0;
             
                
                       
                Grap1_State = Grap1_Action_2;
            }
              
            
            break;
        }
            
        case Grap1_Action_2:
        {      
              Delay_ms(1);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, 2000);
            Delay_ms(100);
            ZDT_JueDui_Position_Control(0x03, behind, 700, 250, 2700);
            Delay_ms(500);
            Servo_SmoothSetAngle_CH1(0, 1);
            Delay_ms(110);
            ZDT_JueDui_Position_Control(0x03, behind, 700, 250, 0);
            Delay_ms(500);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, 1800);
            Delay_ms(100);
            Servo_SmoothSetAngle_CH3(40, 70);
            Delay_ms(800);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, 2081);
            Delay_ms(100);
            ZDT_JueDui_Position_Control(0x03, behind, 700, 250, 2101);
            Delay_ms(500);
            Servo_SmoothSetAngle_CH1(30, 1);
            Delay_ms(110);
            ZDT_JueDui_Position_Control(0x03, behind, 700, 250, 0);
            Delay_ms(500);
            Servo_SmoothSetAngle_CH3(180, 70);
            Delay_ms(800);
                Grap1_State = Grap1_Next_2;
              
            
            break;
        }
            
        case Grap1_Next_2:
        {   Servo_SmoothSetAngle_CH2(240, 1);
            Delay_ms(110);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, 0);   /* ← 50,0 → 200,200 */
            Delay_ms(100);
            K230_SendTaskCmd(0x3F);
            Delay_ms(50);
           	g_visual_stop_flag = 0; g_visual_dx = 0; g_visual_dy = 0;
            Grap1_State = Grap1_Visual_3;
           
            
            break;
        }
            
        case Grap1_Visual_3:
        {
             
            if (g_visual_stop_flag) {        // 收到 "0\n"，到位了
              g_visual_stop_flag = 0; g_visual_dx = 0; g_visual_dy = 0;
             
                
                       
                Grap1_State = Grap1_Action_3;
            }
            
            break;
        }
            
        case Grap1_Action_3:
        { Delay_ms(1);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, 2000);
            Delay_ms(100);
            ZDT_JueDui_Position_Control(0x03, behind, 700, 250, 2700);
            Delay_ms(500);
            Servo_SmoothSetAngle_CH1(0, 1);
            Delay_ms(110);
            ZDT_JueDui_Position_Control(0x03, behind, 700, 250, 0);
            Delay_ms(500);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, 1800);
            Delay_ms(100);
            Servo_SmoothSetAngle_CH3(40, 70);
            Delay_ms(800);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, 2081);
            Delay_ms(100);
            ZDT_JueDui_Position_Control(0x03, behind, 700, 250, 2101);
            Delay_ms(500);
            Servo_SmoothSetAngle_CH1(30, 1);
            Delay_ms(110);
            ZDT_JueDui_Position_Control(0x03, behind, 700, 250, 0);
            Delay_ms(500);
            Servo_SmoothSetAngle_CH3(6, 70);               /* 最后一颗回6° */
            Delay_ms(800);
                
                Grap1_State = Grap1_Success;
                
            
            break;
        }
        case Grap1_Success:
            Special_Task_Completed_Flag = 1;
            break;
            
        default:
            break;
    }
}


void Grap_and_put1_State_Function(void)
{     
    static uint8_t cnt = 0;
    if (++cnt < 5)
        return;    
        
    switch(Grap_and_put1_State)
    {
        case Grap_and_put1_Idle:
            break;
        
        case Grap_and_put1_Start:
        {
            static uint8_t inited = 0;
            if (!inited) {
                 Servo_SmoothSetAngle_CH3(180,100);
                Delay_ms(1100);
							 ZDT_JueDui_Position_Control(0x02, behind, 50, 0, 0);
					     Delay_ms(100);
               
                Delay_ms(20);
                K230_SendTaskCmd(0x2F);       
                Delay_ms(50);
                
                /* 清标志 - 防止历史残留 */
                g_visual_stop_flag = 0; g_visual_dx = 0; g_visual_dy = 0;
              
                
               
                
                inited = 1;
            }
            Grap_and_put1_State = Grap_and_put1_Visual_1;
            inited = 0;
            break;
        }
        
        case Grap_and_put1_Visual_1:
        {
            Grap2_Wei_Tiao_Function(-180);        
            if (g_visual_stop_flag) {         
               g_visual_stop_flag = 0; g_visual_dx = 0; g_visual_dy = 0;
                uint8_t addr_map[4] = {0x05, 0x08, 0x06, 0x09};
                uint8_t i;
                for (i = 0; i < 4; i++) {
                    ZDT_Speed_Control(addr_map[i], straight, 0);
                    Delay_ms(1);
                }
                ZDT_DJ_TongBu_Control();          
                Delay_ms(1);
                                
                
            
               
                Grap_and_put1_State = Grap_and_put1_Action1;    
            }
            break;
        }   
        
												case Grap_and_put1_Action1:
					{
							/* ========== 查表：角度、速度、延时、水平位置 ========== */
							const double a1 = (task1==1)?135.1:(task1==2)?180.3:221;
							const double s1 = (a1==135.1)?90:180;        // 135.1°用50速，其余100
							const double d1 = s1 * 10 + 100;          // 50→600, 100→1100
							const double x1 = (task1==1)?801:(task1==2)?2311:851;

							const double a2 = (task2==1)?135.1:(task2==2)?180.3:221;
							const double s2 = (a2==135.1)?90:180;
							const double d2 = s2 * 10 + 100;
							const double x2 = (task2==1)?801:(task2==2)?2311:851;

							const double a3 = (task3==1)?135.1:(task3==2)?180.3:221;
							const double s3 = (a3==135.1)?90:180;
							const double d3 = s3 * 10 + 100;
							const double x3 = (task3==1)?801:(task3==2)?2311:851;
							/* ================================================== */
         /* ===== 1号盘(0°)取物 → task1色环放下 ===== */
            Delay_ms(1);
            Servo_SmoothSetAngle_CH2(0, 1);
            //Delay_ms(110);
            Servo_SmoothSetAngle_CH3(40, 100);
            Delay_ms(1100);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, 2081);
            Delay_ms(10);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 2101);
            Delay_ms(502);
            Servo_SmoothSetAngle_CH1(0, 1);
            Delay_ms(110);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 0);
            Delay_ms(502);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, 1800);
            Delay_ms(100);
            Servo_SmoothSetAngle_CH3(a1, s1);
						 Servo_SmoothSetAngle_CH2(240, 1);
            Delay_ms(d1);					
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, x1);
            Delay_ms(100);         
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 9500);
            Delay_ms(801);
            Servo_SmoothSetAngle_CH1(30, 1);
            Delay_ms(110);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 0);
            Delay_ms(801);

            /* ===== 2号盘(120°)取物 → task2色环放下 ===== */
            Delay_ms(1);
            Servo_SmoothSetAngle_CH2(120, 1);
            //Delay_ms(110);
            Servo_SmoothSetAngle_CH3(40, s1);
            Delay_ms(d1);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, 2081);
            Delay_ms(10);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 2101);
            Delay_ms(502);
            Servo_SmoothSetAngle_CH1(0, 1);
            Delay_ms(110);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 0);
            Delay_ms(502);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, 1800);
            Delay_ms(100);
            Servo_SmoothSetAngle_CH3(a2, s2);
            Delay_ms(d2);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, x2);
            Delay_ms(100);
            Servo_SmoothSetAngle_CH2(240, 1);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 9500);
            Delay_ms(801);
            Servo_SmoothSetAngle_CH1(30, 1);
            Delay_ms(110);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 0);
            Delay_ms(801);

            /* ===== 3号盘(240°)取物 → task3色环放下 ===== */
            Delay_ms(1);
            Servo_SmoothSetAngle_CH2(240, 1);
            //Delay_ms(110);
            Servo_SmoothSetAngle_CH3(40, s2);
            Delay_ms(d2);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, 2081);
            Delay_ms(10);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 2101);
            Delay_ms(502);
            Servo_SmoothSetAngle_CH1(0, 1);
            Delay_ms(110);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 0);
            Delay_ms(502);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, 1800);
            Delay_ms(100);
            Servo_SmoothSetAngle_CH3(a3, s3);
            Delay_ms(d3);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, x3);
            Delay_ms(100);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 9500);
            Delay_ms(801);
            Servo_SmoothSetAngle_CH1(30, 1);
            Delay_ms(110);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 0);
            Delay_ms(801);

            /* ===== task1色环取物 → 1号盘(0°)放回 ===== */
            Delay_ms(1);
            Servo_SmoothSetAngle_CH2(0, 1);
            //Delay_ms(110);
            Servo_SmoothSetAngle_CH3(a1, s1);
            Delay_ms(d1);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, x1);
            Delay_ms(100);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 9500);
            Delay_ms(801);
            Servo_SmoothSetAngle_CH1(0, 1);
            Delay_ms(110);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 0);
            Delay_ms(801);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, 1800);
            Delay_ms(100);
            Servo_SmoothSetAngle_CH3(40, s1);
            Delay_ms(d1);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, 2081);
            Delay_ms(100);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 2101);
            Delay_ms(502);
            Servo_SmoothSetAngle_CH1(30, 1);
            Delay_ms(110);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 0);
            Delay_ms(502);

            /* ===== task2色环取物 → 2号盘(120°)放回 ===== */
            Delay_ms(1);
            Servo_SmoothSetAngle_CH2(120, 1);
            //Delay_ms(110);
            Servo_SmoothSetAngle_CH3(a2, s2);
            Delay_ms(d2);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, x2);
            Delay_ms(100);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 9500);
            Delay_ms(801);
            Servo_SmoothSetAngle_CH1(0, 1);
            Delay_ms(110);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 0);
            Delay_ms(801);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, 1800);
            Delay_ms(100);
            Servo_SmoothSetAngle_CH3(40, s2);
            Delay_ms(d2);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, 2081);
            Delay_ms(100);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 2101);
            Delay_ms(502);
            Servo_SmoothSetAngle_CH1(30, 1);
            Delay_ms(110);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 0);
            Delay_ms(502);

            /* ===== task3色环取物 → 3号盘(240°)放回 ===== */
            Delay_ms(1);
            Servo_SmoothSetAngle_CH3(a3, s3);
            Delay_ms(d3);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, x3);
            Delay_ms(100);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 9500);
            Delay_ms(801);
            Servo_SmoothSetAngle_CH1(0, 1);
            Delay_ms(110);
            Servo_SmoothSetAngle_CH2(240, 100);
            Delay_ms(1100);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 0);
            Delay_ms(801);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, 1800);
            Delay_ms(100);
            Servo_SmoothSetAngle_CH3(40, s3);
            Delay_ms(d3);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, 2081);
            Delay_ms(100);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 2101);
            Delay_ms(502);
            Servo_SmoothSetAngle_CH1(30, 1);
            Delay_ms(110);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 0);
            Delay_ms(502);

            Servo_SmoothSetAngle_CH3(6, 100);
            
							Grap_and_put1_State = Grap_and_put1_Success;
							break;
					}
										
 
        case Grap_and_put1_Success:
            Special_Task_Completed_Flag = 1;
            break;
            
        default:
            break;
    }
}


void put1_State_Function(void)
{      static uint8_t cnt = 0;
    if (++cnt < 5)
			return;    
    switch(put1_State)
    {
        case put1_Idle:
            break;
				
				case put1_Start:
       {
            static uint8_t inited = 0;
            if (!inited) {
           Servo_SmoothSetAngle_CH3(180,100);
                Delay_ms(1100);
								ZDT_JueDui_Position_Control(0x02, behind, 50, 0, 0);
					     Delay_ms(100);
                Delay_ms(20);
                K230_SendTaskCmd(0x2F);       
                Delay_ms(50);
							
							
                /* 清标志 */
              
                g_visual_stop_flag = 0; g_visual_dx = 0; g_visual_dy = 0;
                
               
                
                inited = 1;
            }
         put1_State=put1_Visual_1;
            inited = 0;
            break;
        }
			 
				   case put1_Visual_1:
        {
           
            Grap2_Wei_Tiao_Function(90);        // 复用同一套微调逻辑（前后左右）
            
            if (g_visual_stop_flag) {         // 收到 "0\n"，对准完成
               g_visual_stop_flag = 0; g_visual_dx = 0; g_visual_dy = 0;
							uint8_t addr_map[4] = {0x05, 0x08, 0x06, 0x09};
                uint8_t i;
                for (i = 0; i < 4; i++) {
                    ZDT_Speed_Control(addr_map[i], straight, 0);  // 零速
                    Delay_ms(1);
                }
                ZDT_DJ_TongBu_Control();          // 同步发送
							
								
        
               
                put1_State =put1_Action1;    // 进动作执行
            }
            break;
        }   
			
													case put1_Action1:
					{
							const double a1 = (task1==1)?135.3:(task1==2)?180.3:221.4;
            const double s1 = (a1==135.3)?90:180;                          /* ← 133 → 135.3 (修bug) */
            const double d1 = s1 * 10 + 100;
            const double x1 = (task1==1)?771:(task1==2)?2301:851;

            const double a2 = (task2==1)?135.3:(task2==2)?180.3:221.4;
            const double s2 = (a2==135.3)?90:180;                          /* ← 133 → 135.3 */
            const double d2 = s2 * 10 + 100;
            const double x2 = (task2==1)?771:(task2==2)?2301:851;

            const double a3 = (task3==1)?135.3:(task3==2)?180.3:221.4;
            const double s3 = (a3==135.3)?90:180;                          /* ← 133 → 135.3 */
            const double d3 = s3 * 10 + 100;
            const double x3 = (task3==1)?771:(task3==2)?2301:851;

            /* ===== task1 → 1号盘(0°) ===== */
            Delay_ms(1);
            Servo_SmoothSetAngle_CH2(0, 1);
            //Delay_ms(110);
            Servo_SmoothSetAngle_CH3(40, 100);
            Delay_ms(1100);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, 2081);
            Delay_ms(10);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 2101);
            Delay_ms(502);
            Servo_SmoothSetAngle_CH1(0, 1);
            Delay_ms(110);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 0);
            Delay_ms(502);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, 1800);
            Delay_ms(100);
            Servo_SmoothSetAngle_CH3(a1, s1);
					 Servo_SmoothSetAngle_CH2(240, 1);
            Delay_ms(d1);			     
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, x1);
            Delay_ms(100);       
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 9500);
            Delay_ms(801);
            Servo_SmoothSetAngle_CH1(30, 1);
            Delay_ms(110);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 0);
            Delay_ms(801);

            /* ===== task2 → 2号盘(120°) ===== */
            Delay_ms(1);
            Servo_SmoothSetAngle_CH2(120, 1);
            //Delay_ms(110);
            Servo_SmoothSetAngle_CH3(40, s1);
            Delay_ms(d1);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, 2081);
            Delay_ms(10);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 2101);
            Delay_ms(502);
            Servo_SmoothSetAngle_CH1(0, 1);
            Delay_ms(110);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 0);
            Delay_ms(502);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, 1800);
            Delay_ms(100);
            Servo_SmoothSetAngle_CH3(a2, s2);
            Delay_ms(d2);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, x2);
            Delay_ms(100);
            Servo_SmoothSetAngle_CH2(240, 1);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 9500);
            Delay_ms(801);
            Servo_SmoothSetAngle_CH1(30, 1);
            Delay_ms(110);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 0);
            Delay_ms(801);

            /* ===== task3 → 3号盘(240°) ===== */
            Delay_ms(1);
            Servo_SmoothSetAngle_CH2(240, 1);
            //Delay_ms(110);
            Servo_SmoothSetAngle_CH3(40, s2);
            Delay_ms(d2);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, 2081);
            Delay_ms(10);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 2101);
            Delay_ms(502);
            Servo_SmoothSetAngle_CH1(0, 1);
            Delay_ms(110);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 0);
            Delay_ms(502);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, 1800);
            Delay_ms(100);
            Servo_SmoothSetAngle_CH3(a3, s3);
            Delay_ms(d3);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, x3);
            Delay_ms(100);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 9500);
            Delay_ms(801);
            Servo_SmoothSetAngle_CH1(30, 1);
            Delay_ms(110);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 0);
            Delay_ms(801);
            
            Servo_SmoothSetAngle_CH3(6, 100);
        

							put1_State = put1_Success;
							break;
					}
								
 
        case put1_Success:
            Special_Task_Completed_Flag = 1;
            break;
            
        default:
            break;
    }
}

void Grap2_State_Function(void)
{     
    static uint8_t cnt = 0;
    if (++cnt < 5)
        return;    
    
    
    switch(Grap2_State)
    {
        case Grap2_Idle:
            break;
            
        case Grap2_Start:
        {
            static uint8_t inited = 0;
            if (!inited) {
              Servo_SmoothSetAngle_CH3(180,100);
                Delay_ms(50);
							   ZDT_JueDui_Position_Control(0x02, behind, 50, 0, 0);
					     Delay_ms(100);
							  Servo_SmoothSetAngle_CH1(30,1);
					     Delay_ms(110);
							Servo_SmoothSetAngle_CH2(0,1);
										     Delay_ms(110);
              
                
              
               K230_SendTaskCmd(0x4F);
               Delay_ms(50);
               	g_visual_stop_flag = 0; g_visual_dx = 0; g_visual_dy = 0;
               
                inited = 1;
            }
            
            Grap2_State = Grap2_Visual_1;    // 进视觉对齐
            inited = 0;
            break;
        }
            
          case Grap2_Visual_1:
        {
          
                
            
              Grap1_Wei_Tiao_Function(0);        
            if (g_visual_stop_flag) {         
              g_visual_stop_flag = 0; g_visual_dx = 0; g_visual_dy = 0;
                uint8_t addr_map[4] = {0x05, 0x08, 0x06, 0x09};
                uint8_t i;
                for (i = 0; i < 4; i++) {
                    ZDT_Speed_Control(addr_map[i], straight, 0);
                    Delay_ms(1);
                }
                ZDT_DJ_TongBu_Control();          
               
              
             
                
                       
                Grap2_State =  Grap2_Action_1;
            }
            break;
        }
            
        case Grap2_Action_1:
        {
                   Delay_ms(1); 
                ZDT_JueDui_Position_Control(0x02, behind, 200, 200, 2000);
            Delay_ms(100);
            ZDT_JueDui_Position_Control(0x03, behind, 700, 250, 2700);
            Delay_ms(500);
            Servo_SmoothSetAngle_CH1(0, 1);
            Delay_ms(110);
            ZDT_JueDui_Position_Control(0x03, behind, 700, 250, 0);
            Delay_ms(500);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, 1800);
            Delay_ms(100);
            Servo_SmoothSetAngle_CH3(40, 70);
            Delay_ms(800);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, 2081);
            Delay_ms(100);
            ZDT_JueDui_Position_Control(0x03, behind, 700, 250, 2101);
            Delay_ms(500);
            Servo_SmoothSetAngle_CH1(30, 1);
            Delay_ms(110);
            ZDT_JueDui_Position_Control(0x03, behind, 700, 250, 0);
            Delay_ms(500);
            Servo_SmoothSetAngle_CH3(180, 70);
            Delay_ms(800);
					
                Grap2_State = Grap2_Next_1;
                
            
            break;
        }
            
        case Grap2_Next_1:
        {        	Servo_SmoothSetAngle_CH2(120,1);
					     Delay_ms(110);
					    ZDT_JueDui_Position_Control(0x02, behind, 50, 0, 0);
					     Delay_ms(100);
                K230_SendTaskCmd(0x3F);      // Next，视觉切到下一个颜色
                Delay_ms(50);
             	g_visual_stop_flag = 0; g_visual_dx = 0; g_visual_dy = 0;
                
             
                Grap2_State = Grap2_Visual_2;
              
            
            break;
        }
            
        case Grap2_Visual_2:
        {
            
            if (g_visual_stop_flag) {        // 收到 "0\n"，到位了
                g_visual_stop_flag = 0; g_visual_dx = 0; g_visual_dy = 0;
             
                
                       
                Grap2_State = Grap2_Action_2;
            }
              
            
            break;
        }
            
        case Grap2_Action_2:
        {      
             
              Delay_ms(1);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, 2000);
            Delay_ms(100);
            ZDT_JueDui_Position_Control(0x03, behind, 700, 250, 2700);
            Delay_ms(500);
            Servo_SmoothSetAngle_CH1(0, 1);
            Delay_ms(110);
            ZDT_JueDui_Position_Control(0x03, behind, 700, 250, 0);
            Delay_ms(500);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, 1800);
            Delay_ms(100);
            Servo_SmoothSetAngle_CH3(40, 70);
            Delay_ms(800);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, 2081);
            Delay_ms(100);
            ZDT_JueDui_Position_Control(0x03, behind, 700, 250, 2101);
            Delay_ms(500);
            Servo_SmoothSetAngle_CH1(30, 1);
            Delay_ms(110);
            ZDT_JueDui_Position_Control(0x03, behind, 700, 250, 0);
            Delay_ms(500);
            Servo_SmoothSetAngle_CH3(180, 70);
            Delay_ms(800);
             
                Grap2_State = Grap2_Next_2;

              
            
            break;
        }
            
        case Grap2_Next_2:
        {   Servo_SmoothSetAngle_CH2(240,1);
										     Delay_ms(110);
					ZDT_JueDui_Position_Control(0x02, behind, 50, 0, 0);
					     Delay_ms(100);
                K230_SendTaskCmd(0x3F);      // Next，切到颜色3
                Delay_ms(50);
                g_visual_stop_flag = 0; g_visual_dx = 0; g_visual_dy = 0;
                
            
                Grap2_State = Grap2_Visual_3;
           
            
            break;
        }
            
        case Grap2_Visual_3:
        {
             
            if (g_visual_stop_flag) {        // 收到 "0\n"，到位了
                g_visual_stop_flag = 0; g_visual_dx = 0; g_visual_dy = 0;
             
                
                       
                Grap2_State = Grap2_Action_3;
            }
            
            break;
        }
            
        case Grap2_Action_3:
        { Delay_ms(1);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, 2000);
            Delay_ms(100);
            ZDT_JueDui_Position_Control(0x03, behind, 700, 250, 2700);
            Delay_ms(500);
            Servo_SmoothSetAngle_CH1(0, 1);
            Delay_ms(110);
            ZDT_JueDui_Position_Control(0x03, behind, 700, 250, 0);
            Delay_ms(500);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, 1800);
            Delay_ms(100);
            Servo_SmoothSetAngle_CH3(40, 70);
            Delay_ms(800);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, 2081);
            Delay_ms(100);
            ZDT_JueDui_Position_Control(0x03, behind, 700, 250, 2101);
            Delay_ms(500);
            Servo_SmoothSetAngle_CH1(30, 1);
            Delay_ms(110);
            ZDT_JueDui_Position_Control(0x03, behind, 700, 250, 0);
            Delay_ms(500);
            Servo_SmoothSetAngle_CH3(6, 70);               /* 最后一颗回6° */
            Delay_ms(800);
                
                Grap2_State = Grap2_Success;

                
            
            break;
        }
        case Grap2_Success:
            Special_Task_Completed_Flag = 1;
            break;
            
        default:
            break;
    }
}


void Grap_and_put2_State_Function(void)
{     
    static uint8_t cnt = 0;
    if (++cnt < 5)
        return;    
    switch(Grap_and_put2_State)
    {
        case Grap_and_put2_Idle:
            break;
        
        case Grap_and_put2_Start:
        {
            static uint8_t inited = 0;
            if (!inited) {
                 Servo_SmoothSetAngle_CH3(180,100);
                Delay_ms(1100);
							 ZDT_JueDui_Position_Control(0x02, behind, 50, 0, 0);
					     Delay_ms(100);
               
                Delay_ms(20);
                K230_SendTaskCmd(0x5F);       
                Delay_ms(50);
                
                g_visual_stop_flag = 0; g_visual_dx = 0; g_visual_dy = 0;
                
               
                
                inited = 1;
            }
            Grap_and_put2_State = Grap_and_put2_Visual_1;
            inited = 0;
            break;
        }
        
        case Grap_and_put2_Visual_1:
        {
            Grap2_Wei_Tiao_Function(-180);        
            if (g_visual_stop_flag) {         
                g_visual_stop_flag = 0; g_visual_dx = 0; g_visual_dy = 0;
                uint8_t addr_map[4] = {0x05, 0x08, 0x06, 0x09};
                uint8_t i;
                for (i = 0; i < 4; i++) {
                    ZDT_Speed_Control(addr_map[i], straight, 0);
                    Delay_ms(1);
                }
                ZDT_DJ_TongBu_Control();          
                Delay_ms(1);
                                
                
            
               
                Grap_and_put2_State = Grap_and_put2_Action1;    
            }
            break;
        }   
									case Grap_and_put2_Action1:
					{
						
								const double a4 = (task4==1)?135.1:(task4==2)?180.3:221;
							const double s4 = (a4==135.1)?90:180;        // 135.1°用50速，其余100
							const double d4 = s4 * 10 + 100;          // 50→600, 100→1100
							const double x4 = (task4==1)?801:(task4==2)?2311:851;

							const double a5 = (task5==1)?135.1:(task5==2)?180.3:221;
							const double s5 = (a5==135.1)?90:180;
							const double d5 = s5 * 10 + 100;
							const double x5 = (task5==1)?801:(task5==2)?2311:851;

							const double a6 = (task6==1)?135.1:(task6==2)?180.3:221;
							const double s6 = (a6==135.1)?90:180;
							const double d6 = s6 * 10 + 100;
							const double x6 = (task6==1)?801:(task6==2)?2311:851;
		

						/* ===== 1号盘(0°)取物 → task4色环放下 ===== */
            Delay_ms(1);
            Servo_SmoothSetAngle_CH2(0, 1);
            //Delay_ms(110);
            Servo_SmoothSetAngle_CH3(40, 100);
            Delay_ms(1100);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, 2081);
            Delay_ms(10);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 2101);
            Delay_ms(502);
            Servo_SmoothSetAngle_CH1(0, 1);
            Delay_ms(110);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 0);
            Delay_ms(502);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, 1800);
            Delay_ms(100);
            Servo_SmoothSetAngle_CH3(a4, s4);
            Servo_SmoothSetAngle_CH2(240, 1);
            Delay_ms(d4);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, x4);
            Delay_ms(100);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 9500);
            Delay_ms(801);
            Servo_SmoothSetAngle_CH1(30, 1);
            Delay_ms(110);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 0);
            Delay_ms(801);

            /* ===== 2号盘(120°)取物 → task5色环放下 ===== */
            Delay_ms(1);
            Servo_SmoothSetAngle_CH2(120, 1);
            //Delay_ms(110);
            Servo_SmoothSetAngle_CH3(40, s4);
            Delay_ms(d4);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, 2081);
            Delay_ms(10);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 2101);
            Delay_ms(502);
            Servo_SmoothSetAngle_CH1(0, 1);
            Delay_ms(110);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 0);
            Delay_ms(502);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, 1800);
            Delay_ms(100);
            Servo_SmoothSetAngle_CH3(a5, s5);
            Delay_ms(d5);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, x5);
            Delay_ms(100);
            Servo_SmoothSetAngle_CH2(240, 1);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 9500);
            Delay_ms(801);
            Servo_SmoothSetAngle_CH1(30, 1);
            Delay_ms(110);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 0);
            Delay_ms(801);

            /* ===== 3号盘(240°)取物 → task6色环放下 ===== */
            Delay_ms(1);
            Servo_SmoothSetAngle_CH2(240, 1);
            //Delay_ms(110);
            Servo_SmoothSetAngle_CH3(40, s5);
            Delay_ms(d5);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, 2081);
            Delay_ms(10);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 2101);
            Delay_ms(502);
            Servo_SmoothSetAngle_CH1(0, 1);
            Delay_ms(110);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 0);
            Delay_ms(502);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, 1800);
            Delay_ms(100);
            Servo_SmoothSetAngle_CH3(a6, s6);
            Delay_ms(d6);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, x6);
            Delay_ms(100);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 9500);
            Delay_ms(801);
            Servo_SmoothSetAngle_CH1(30, 1);
            Delay_ms(110);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 0);
            Delay_ms(801);

            /* ===== task4色环取物 → 1号盘(0°)放回 ===== */
            Delay_ms(1);
            Servo_SmoothSetAngle_CH2(0, 1);
            //Delay_ms(110);
            Servo_SmoothSetAngle_CH3(a4, s4);
            Delay_ms(d4);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, x4);
            Delay_ms(10);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 9500);
            Delay_ms(801);
            Servo_SmoothSetAngle_CH1(0, 1);
            Delay_ms(110);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 0);
            Delay_ms(801);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, 1800);
            Delay_ms(100);
            Servo_SmoothSetAngle_CH3(40, s4);
            Delay_ms(d4);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, 2081);
            Delay_ms(100);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 2101);
            Delay_ms(502);
            Servo_SmoothSetAngle_CH1(30, 1);
            Delay_ms(110);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 0);
            Delay_ms(502);

            /* ===== task5色环取物 → 2号盘(120°)放回 ===== */
            Delay_ms(1);
            Servo_SmoothSetAngle_CH2(120, 1);
            //Delay_ms(110);
            Servo_SmoothSetAngle_CH3(a5, s5);
            Delay_ms(d5);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, x5);
            Delay_ms(100);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 9500);
            Delay_ms(801);
            Servo_SmoothSetAngle_CH1(0, 1);
            Delay_ms(110);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 0);
            Delay_ms(801);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, 1800);
            Delay_ms(100);
            Servo_SmoothSetAngle_CH3(40, s5);
            Delay_ms(d5);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, 2081);
            Delay_ms(100);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 2101);
            Delay_ms(502);
            Servo_SmoothSetAngle_CH1(30, 1);
            Delay_ms(110);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 0);
            Delay_ms(502);

            /* ===== task6色环取物 → 3号盘(240°)放回 ===== */
            Delay_ms(1);
            Servo_SmoothSetAngle_CH3(a6, s6);
            Delay_ms(d6);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, x6);
            Delay_ms(100);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 9500);
            Delay_ms(801);
            Servo_SmoothSetAngle_CH1(0, 1);
            Delay_ms(110);
            Servo_SmoothSetAngle_CH2(240, 100);
            Delay_ms(1100);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 0);
            Delay_ms(801);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, 1800);
            Delay_ms(100);
            Servo_SmoothSetAngle_CH3(40, s6);
            Delay_ms(d6);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, 2081);
            Delay_ms(100);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 2101);
            Delay_ms(502);
            Servo_SmoothSetAngle_CH1(30, 1);
            Delay_ms(110);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 0);
            Delay_ms(502);

            Servo_SmoothSetAngle_CH3(6, 100);

							Grap_and_put2_State = Grap_and_put2_Success;
							break;

					}
 
        case Grap_and_put2_Success:
            Special_Task_Completed_Flag = 1;
            break;
            
        default:
            break;
    }
}

void put2_State_Function(void)
{     
    static uint8_t cnt = 0;
    if (++cnt < 5)
        return;    
    
    switch(put2_State)
    {
        case put2_Idle:
            break;
				
				case put2_Start:
       {
            static uint8_t inited = 0;
            if (!inited) {
           Servo_SmoothSetAngle_CH3(180,100);
                Delay_ms(1100);
								ZDT_JueDui_Position_Control(0x02, behind, 50, 0, 0);
					     Delay_ms(100);
                Delay_ms(20);
                K230_SendTaskCmd(0x5F);       
                Delay_ms(50);
							
							
              g_visual_stop_flag = 0; g_visual_dx = 0; g_visual_dy = 0;
                
               
                
                inited = 1;
            }
         put2_State=put2_Visual_1;
            inited = 0;
            break;
        }
			 
				   case put2_Visual_1:
        {
           
            Grap2_Wei_Tiao_Function(90);        // 复用同一套微调逻辑（前后左右）
            
            if (g_visual_stop_flag) {         // 收到 "0\n"，对准完成
               g_visual_stop_flag = 0; g_visual_dx = 0; g_visual_dy = 0;
							uint8_t addr_map[4] = {0x05, 0x08, 0x06, 0x09};
                uint8_t i;
                for (i = 0; i < 4; i++) {
                    ZDT_Speed_Control(addr_map[i], straight, 0);  // 零速
                    Delay_ms(1);
                }
                ZDT_DJ_TongBu_Control();          // 同步发送
							
								
        
               
                put2_State =put2_Action1;    // 进动作执行
            }
            break;
        }   
			
											 case put2_Action1:
				{ 
					/* ========== 查表：角度、速度、延时、水平位置 ========== */
							const double a4 = (task4==1)?131.5:(task4==2)?180.3:221.4;
							const double s4 = (a4==131.5)?90:180;
							const double d4 = s4 * 10 + 100;
							const double x4 = (task4==1)?781:(task4==2)?2301:841;

							const double a5 = (task5==1)?131.5:(task5==2)?180.3:221.4;
							const double s5 = (a5==131.5)?90:180;
							const double d5 = s5 * 10 + 100;
							const double x5 = (task5==1)?781:(task5==2)?2301:841;

							const double a6 = (task6==1)?131.5:(task6==2)?180.3:221.4;
							const double s6 = (a6==131.5)?90:180;
							const double d6 = s6 * 10 + 100;
							const double x6 = (task6==1)?781:(task6==2)?2301:841;
							/* ================================================== */


				Delay_ms(1);
            Servo_SmoothSetAngle_CH2(0, 1);
            //Delay_ms(110);
            Servo_SmoothSetAngle_CH3(40, 100);
            Delay_ms(1100);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, 2081);
            Delay_ms(10);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 2101);
            Delay_ms(502);
            Servo_SmoothSetAngle_CH1(0, 1);
            Delay_ms(110);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 0);
            Delay_ms(502);
            ZDT_JueDui_Position_Control(0x02, straight, 200, 200, 1300);  /* clk/dir保持原样 */
            Delay_ms(100);
            Servo_SmoothSetAngle_CH3(a4, s4);
            Servo_SmoothSetAngle_CH2(240, 1);//这个可以改改看看
            Delay_ms(d4);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, x4);
            Delay_ms(100);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 3600);     /* clk保持原样2800 */
            Delay_ms(801);
            Servo_SmoothSetAngle_CH1(30, 1);
            Delay_ms(110);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 0);
            Delay_ms(801);

            /* ===== task5 → 2号盘(120°) ===== */
            Delay_ms(1);
            Servo_SmoothSetAngle_CH2(120, 1);
            //Delay_ms(110);
            Servo_SmoothSetAngle_CH3(40, s4);
            Delay_ms(d4);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, 2081);
            Delay_ms(10);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 2101);
            Delay_ms(502);
            Servo_SmoothSetAngle_CH1(0, 1);
            Delay_ms(110);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 0);
            Delay_ms(502);
            ZDT_JueDui_Position_Control(0x02, straight, 200, 200, 1300);
            Delay_ms(100);
            Servo_SmoothSetAngle_CH3(a5, s5);
            Delay_ms(d5);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, x5);
            Delay_ms(100);
            Servo_SmoothSetAngle_CH2(240, 1);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 3600);
            Delay_ms(801);
            Servo_SmoothSetAngle_CH1(30, 1);
            Delay_ms(110);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 0);
            Delay_ms(801);

            /* ===== task6 → 3号盘(240°) ===== */
            Delay_ms(1);
            Servo_SmoothSetAngle_CH2(240, 1);
            //Delay_ms(110);
            Servo_SmoothSetAngle_CH3(40, s5);
            Delay_ms(d5);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, 2081);
            Delay_ms(10);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 2101);
            Delay_ms(502);
            Servo_SmoothSetAngle_CH1(0, 1);
            Delay_ms(110);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 0);
            Delay_ms(502);
            ZDT_JueDui_Position_Control(0x02, straight, 200, 200, 1300);
            Delay_ms(100);
            Servo_SmoothSetAngle_CH3(a6, s6);
            Delay_ms(d6);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, x6);
            Delay_ms(100);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 3600);
            Delay_ms(801);
            Servo_SmoothSetAngle_CH1(30, 1);
            Delay_ms(110);
            ZDT_JueDui_Position_Control(0x03, behind, 300, 250, 0);
            Delay_ms(801);

            Servo_SmoothSetAngle_CH3(6, 100);
						Delay_ms(2);
            ZDT_JueDui_Position_Control(0x02, behind, 200, 200, 0);
            Delay_ms(2);
						put2_State = put2_Success;
							break;
				}
 
        case put2_Success:
            Special_Task_Completed_Flag = 1;
            break;
            
        default:
            break;
    }
}
