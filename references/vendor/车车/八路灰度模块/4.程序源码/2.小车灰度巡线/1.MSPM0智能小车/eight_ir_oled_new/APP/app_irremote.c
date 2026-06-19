#include "app_irremote.h"


motion_state_t get_CarState = MOTION_RUN;


uint8_t gIRdata = CLEAR_DATA;

void Printf_Irremote(void)
{
    gIRdata = get_infrared_command();
    clear_infrared_command();
    
    if(gIRdata !=CLEAR_DATA)
    {
        printf("Key data:%x\r\n",gIRdata);
        gIRdata = CLEAR_DATA;
    }

}



void IR_Control_Car(void)
{
    gIRdata = get_infrared_command();
    clear_infrared_command();
    
    switch(gIRdata)
    {
        case IR_POWER:		break;	
        case IR_UP:			 Motor_Run(200,200) ;break;		
        case IR_LIGHT:		break;	
        case IR_LEFT:	 	Motor_Left(200,200) ;break;			
        case IR_BEEP:		break;			
        case IR_RIGHT:	 Motor_Right(200,200) ;break;		
        case IR_LEFT_SPIN:break;	 	
        case IR_BACK:		 Motor_Back(200,200) ;break;   		
        case IR_RIHGT_SPIN:	break;	
        case IR_ADD:	break;			
        case IR_0:			 Motor_Stop(1) ;break;		
        case IR_SUB:	break;			
        case IR_1:	break;			
        case IR_2:	break;break;			
        case IR_3:	break;			
        case IR_4:	break;			
        case IR_5:	break;			
        case IR_6:	break;			
        case IR_7:	break;			
        case IR_8:	break;
        case IR_9:  break;
        default: break;

    
    }
    gIRdata = CLEAR_DATA;

}


