#include "main.h"
#include "string.h"

#ifndef PI
#define PI 3.1416
#endif

motor_t motor[num];
int8_t motor_id = 1;
extern uint8_t motor_enabled;
extern int Pan;

/**
************************************************************************
* @brief:      	dm4340_motor_init: DM4310电机初始化函数
* @param:      	void
* @retval:     	void
* @details:    	初始化DM4340型号的电机，设置默认参数和控制模式。
*               初始化Motor1，设置ID、控制模式和命令模式等信息。
************************************************************************
**/
void dm4340_motor_init(void)
{
	// 初始化Motor1电机结构
	memset(&motor[Motor1], 0, sizeof(motor[Motor1])); //分配内存空间
	memset(&motor[Motor2], 0, sizeof(motor[Motor2]));
	memset(&motor[Motor3], 0, sizeof(motor[Motor3]));

	// 设置Motor1的电机信息
	motor[Motor1].id = 1;
	motor[Motor1].ctrl.mode = 0;		// 0: MIT模式   1: 位置速度模式   2: 速度模式
	motor[Motor1].cmd.mode = 0;
	
	motor[Motor1].ctrl.pos_set=0;
	motor[Motor1].ctrl.kp_set=0;//4
	motor[Motor1].ctrl.kd_set=0.0;//0.4
	motor[Motor1].ctrl.tor_set=0;
	motor[Motor1].ctrl.vel_set=0;//1
//	motor[Motor1].ctrl.kp_set=5.0;//1.6
//	motor[Motor1].ctrl.kd_set=0.4;//0.4
//	motor[Motor1].ctrl.pos_set=imu_angle.yaw_real* PI / 180.0f;
//	motor[Motor1].ctrl.tor_set=0;
//	motor[Motor1].ctrl.vel_set=0.8;//1

	// 设置Motor2的电机信息
	motor[Motor2].id = 2;
	motor[Motor2].ctrl.mode = 0;
	motor[Motor2].cmd.mode = 0;
	
	motor[Motor2].ctrl.kd_set=0.8;
	motor[Motor2].ctrl.kp_set=180;
	motor[Motor2].ctrl.pos_set=0;
	motor[Motor2].ctrl.tor_set=-0.2;
	motor[Motor2].ctrl.vel_set=0.25;
	
	// 设置Motor3的电机信息
	motor[Motor3].id = 3;
	motor[Motor3].ctrl.mode = 0;
	motor[Motor3].cmd.mode = 0;
	
	motor[Motor3].ctrl.kd_set=0.5;
	motor[Motor3].ctrl.kp_set=15;
	motor[Motor3].ctrl.pos_set=0;
	motor[Motor3].ctrl.tor_set=0;
	motor[Motor3].ctrl.vel_set=0.0;
	
}


/**
************************************************************************
* @brief:      	ctrl_send: 发送电机控制命令函数
* @param:      	void
* @retval:     	void
* @details:    	根据当前电机ID（motor_id），向对应电机发送控制命令。
*               调用dm4310_ctrl_send函数向指定电机发送控制命令，以响应外部命令。
************************************************************************
**/
void ctrl_send(void)
{
	dm4310_ctrl_send(&hcan1, &motor[Motor1]);
	dm4310_ctrl_send(&hcan1, &motor[Motor2]);
	dm4310_ctrl_send(&hcan1, &motor[Motor3]);

}


/**
************************************************************************
* @brief:      	can1_rx_callback: CAN1接收回调函数
* @param:      	void
* @retval:     	void
* @details:    	处理CAN1接收中断回调，根据接收到的ID和数据，执行相应的处理。
*               当接收到ID为0时，调用dm4310_fbdata函数更新Motor的反馈数据。
************************************************************************
**/
uint8_t rx_data[8] = {0};
uint16_t rec_id;
void can1_rx_callback(void)
{
//	uint16_t rec_id;
//	uint8_t rx_data[8] = {0};
	canx_receive_data(&hcan1, &rec_id, rx_data);
	switch (rec_id)
	{
		case 18: dm4310_fbdata(&motor[Motor3], rx_data); break;	
	}
}
/**
************************************************************************
* @brief:      	can1_rx_callback: CAN2接收回调函数
* @param:      	void
* @retval:     	void
* @details:    	处理CAN1接收中断回调，根据接收到的ID和数据，执行相应的处理。
*               当接收到ID为0时，调用dm4310_fbdata函数更新Motor的反馈数据。
************************************************************************
**/
uint16_t rec_id;
void can2_rx_callback(void)
{
	
	uint8_t rx_data[8] = {0};
	canx_receive_data(&hcan2, &rec_id, rx_data);
	switch (rec_id)
	{
		case 0: dm4310_fbdata(&motor[Motor2], rx_data); break;
	}
}

