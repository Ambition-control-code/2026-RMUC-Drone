#include "main.h"
#include "usbd_cdc_if.h"
#include <string.h>  // 
#include "usbd_cdc_if.h"
int ZM_head_flag;
extern  float INS_quat[4];
float ZM_last_yaw,ZM_yaw_diff,ZM_yaw_counts;

extKalman_t yaw_kalman;         //卡尔曼   
extKalman_t pitch_kalman;
extern GIMBAL_control gimbal_control;

 uint32_t vision_receive_counter = 0;
 uint32_t vision_timer = 0;

Status_zimiao status_zimiao;
Status_c status_c;
PCRecieveVision recieve_visiondata;
PCSendtoVision send_to_vision;

union{
PCRecieveVision recieve_visiondata;
u8 recieve[29];
}recieve_union;

//union{
//PCSendtoVision send_to_vision;
//u8 send[43];	
//}send_union;

//crc校验
//uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
//{
//   uint16_t wExpected = 0;
//   if ((pchMessage == NULL) || (dwLength <= 2))
//   {
//      return 0;
//   }
//   wExpected = Get_CRC16_Check_Sum ( pchMessage, dwLength - 2, 0XFFFF);

//   return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) ==pchMessage[dwLength - 1]);
//}

//接收视觉
uint8_t get_vis;
void  recieve_vision_cdc(u8 *buf){
	get_vis++;
//	memcpy(recieve_union.recieve, visiondata, sizeof(PCRecieveVision));
if(buf[0]==0x0d&&buf[28]==0xff){
	
//	if (Verify_CRC16_Check_Sum(recieve_union.recieve,sizeof(RecieveVision))){	
  memcpy(&recieve_union.recieve, buf, 29); 
//	}
 }

} 

//发送数
void send_visiondata(void){

//	    memset(&send_union, 0, sizeof(send_union));
	send_to_vision.header[0]=0x0d;
	send_to_vision.header[1]=0x0d;
	
	send_to_vision.mode=status_c;;
	
  send_to_vision.q0=INS_quat[0];   //w
	send_to_vision.q1=INS_quat[1];
	send_to_vision.q2=INS_quat[2];
	send_to_vision.q3=INS_quat[3];
	
	
	//发送角度
send_to_vision.yaw_vision=imu_angle.yaw/57.3f;
send_to_vision.yaw_w=imu_angle.gyro_z;
send_to_vision.pitch_vision=imu_angle.pitch/57.3f;//视觉
send_to_vision.pitch_w=imu_angle.gyro_y;
send_to_vision.bullet_speed=24.0f;
send_to_vision.bullet_count=0;
send_to_vision.end[0]=0xff;
send_to_vision.end[1]=0xff;
//  memcpy(&send_union.send,&PCSendtoVision , 43);

u8 buf[43];
	memcpy(&buf,&send_to_vision,43);
  usb_cdc_send(buf,43);
}
   
void Vision_Control(void const * argument)
{
	KalmanCreate( &yaw_kalman, 1, 40 );
	KalmanCreate( &pitch_kalman, 1, 40 );
	for(;;){

		send_visiondata();
		status_zimiao=recieve_union.recieve_visiondata.mode;
//		gimbal_control.pitch_zimiao=recieve_union.recieve_visiondata.pitch*57.3f;
//		gimbal_control.yaw_zimiao=recieve_union.recieve_visiondata.yaw*57.3f;
		gimbal_control.pitch_zimiao=KalmanFilter(&pitch_kalman,recieve_union.recieve_visiondata.pitch*57.3f);
	  gimbal_control.yaw_zimiao=KalmanFilter(&yaw_kalman,recieve_union.recieve_visiondata.yaw*57.3f);
		
//		ZM_yaw_diff=gimbal_control.yaw_zimiao-ZM_last_yaw;

//	    if(ZM_yaw_diff > 150.0f)
//			  {ZM_yaw_counts --;
//				ZM_head_flag = 1;}
//			else
//        {ZM_head_flag = 0;}
//			if(ZM_yaw_diff < -150.0f)
//			   {ZM_yaw_counts ++;
//			    ZM_head_flag = 1;}
//			else
//            {ZM_head_flag = 0;}
//		ZM_last_yaw = gimbal_control.yaw_zimiao;    
	osDelay(1);
	}
}

