#ifndef VISION_TASK_H
#define VISION_TASK_H
#include "main.h"
#define VISION_TASK_SUM   3


typedef enum{
	MEIMIAODAO=0,     //不控制
  MIAODAO,           //控制不开火
	MIAODAOKAIHUO       //控制且开火
}Status_zimiao;
typedef enum{
	SHOUMIAO=0,
  ZIMIAO,
}Status_c;


#pragma pack(1)
typedef struct {
	uint8_t header[2];
uint8_t mode;//1自瞄
	float q0;
	float q1;
	float q2;
	float q3;//16
	float yaw_vision;
	float yaw_w;//角速度
	float pitch_vision;
	float pitch_w;
	float bullet_speed;//4
	//次数
	uint16_t bullet_count;
//	float roll;
	//弹速
  //	 uint8_t reserved[4];  //  1
//    uint16_t crc;          // CRC 
	uint8_t end[2];
	
}PCSendtoVision;//43
#pragma pack()



#pragma pack(1)
typedef struct{
uint8_t header[2];
uint8_t mode;       //模式，0手动，1控制不开火  2控制且开火
  float yaw; 
	float yaw_w;
	float yaw_acc;
	float pitch;
  float pitch_w;	
	float pitch_acc;
//  float distance;         ///////////////////////////
//    uint8_t fire_advise;    //开火建议
//    uint8_t control;//1使能 0关闭
//    uint8_t shoot;//射击命
//    uint16_t crc;	// CRC   
uint8_t end[2];	
}PCRecieveVision;//视觉发来的   29   
#pragma pack()



extern PCSendtoVision send_to_vision;
extern PCRecieveVision recieve_visiondata;

extern Status_zimiao status_zimiao;
extern Status_c status_c;
//void recieve_vision_cdc(uint8_t *data, uint8_t length);
//uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void  recieve_vision_cdc(u8 *buf);
void send_visiondata(void);
void Vision_Control(void const * argument);
//void Start_Vision_Task(void const * argument);
void update_aim_continuous(void);
void deal_zhuantou(void);
#endif

