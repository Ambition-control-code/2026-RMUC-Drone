#include "main.h"

void Start_INS_Task(void const * argument)
{
	for(;;)
	{
		

	 
	 if(offset_OK_flag==0)offset_OK_flag = cali_gyro_hook();
INS_task();
	
//	 INS_task();
//	 switch(offset_OK_flag)
//	  {
//		  case 0:
//		 offset_OK_flag= cali_gyro_hook();

//			  break;
//		  
//		  default:
//			  break;
//	  }

	osDelay(1);
	}
}

//recieve_1
