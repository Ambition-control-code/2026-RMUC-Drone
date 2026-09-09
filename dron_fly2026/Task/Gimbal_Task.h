#ifndef _CONTROL_H
#define _CONTROLH

void Gimbal_control(void const *argument );

typedef struct Angle
{
	float yaw_target;
	float pitch_target;							//target 为中间值
	float yaw_set;
	float yaw_speed_set;
	float pitch_set;	//两个轴的设定值
	float roll_set;
	float pitch_speed_set;
	float pitch_low;
	float pitch_top;
	float yaw_right;
	float yaw_left;									//上下左右极限值
//	float yaw_real;
//	float pitch_real;								//两个轴的真实值
//	float yaw_bias;
//	float pitch_bias;								//两个轴的基础角度
//	u8 yaw_back_flag;									//回中标志位
//	u8 pitch_back_flag;									//回中标志位
//	u8 AUTO_MDOE;
	float pitch_remote_last;
	float pitch_remote_now;
	float pitch_differ;
	float pitch_set_last;
//	void (*f_angle_init)(struct Angle *angle);
//	void (*f_angle_update)(struct Angle *angle);
	float pitch_first;
	float yaw_first;
	
	float yaw_aim_set;
	float pitch_aim_set;
	float shoot_delay_offset;
	float fly_delay_offset;
	float edge_offset;
	float auto_distance;
	float total_offset;
	float auto_angle;
	float pitch_delta;
	float pitch_zimiao;
	float yaw_zimiao;
}GIMBAL_control;

//typedef enum{
//  CHASSIS_STOP=0,//底盘轮子全给零
//	GIMBAL_FOLLOW_CHASSIS,//底盘不动(闭yaw轴)
//	CHASSIS_NO_FOLLOW_GIMBAL_YAW,//不跟随云台
//	CHASSIS_SPIN_R,//陀螺模式
//	CHASSIS_FOLLOW_GIMBAL_YAW,//跟随云台	
//}chassis_mode_e;
////枚举类型随便选一个
//typedef enum{
//	HAND=0,
//    AUTOM,
//	BUFF
//}gimbal_mode_e;
struct vision_compensate//视觉补偿
{
	float need_more_pitch;
	float need_more_yaw;
	float need_distance;
	
////	float unlock_pitch;
////	float unlock_yaw;
	
};

struct KalmanFilter_gimbal//视觉补偿经卡尔曼滤波后的值
{
	float KalmanFilter_pitch;
	float KalmanFilter_yaw;
	
};
void damiao_handler(void);
//extern  KalmanFilter_gimbal go;
void Gimbal_Control(void const * argument);
float normalize_angle(float angle);
float angle_difference(float current, float target);
#endif

