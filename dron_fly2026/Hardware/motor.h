#ifndef MOTHR__H
#define MOTHR__H
#include "stm32f4xx.h"                  // Device header

#include "kalman.h"

typedef struct{
	int32_t raw_value;   					 //编码器不经处理的原始值
	int32_t last_raw_value;					 //上一次的编码器原始值
	int32_t ecd_value;                       //经过处理后连续的编码器值
	int32_t diff;							 //两次编码器之间的差值
	int32_t temp_count;                      //计数用
	uint8_t buf_count;						 //滤波更新buf用
	int32_t ecd_bias;						 //初始编码器值	
	int32_t ecd_raw_rate;					 //通过编码器计算得到的速度原始值
	int32_t rate_buf[6];	     //buf，for filter
	int32_t round_cnt;						 //圈数
	int32_t filter_rate;					 //速度
	int32_t filter_rate_max;				 //最大速度
	float ecd_angle;						 //角度
	float init_angle; 						 //开始读一次数据
	int16_t current;
	int16_t velo;
	int16_t electricity;
	int16_t re_speed;
	int8_t tempature;
	float flag;
//	float real_angle;
//	float zero_angle;
//	float relative_zero_angle;
//	float ALL_angle;
//	int tool_flag;
}dji_motor;

typedef struct {
    // 反馈数据
    float position_rad;      // 输出轴位置(弧度),[-pi, pi]
    float velocity_rad_s;    // 输出轴速度(弧度/?)
    float torque_nm;         // 输出轴扭矩(Nm)
    float temperature_mos;   // MOS温度()
    float temperature_rotor; // 转子温度()
    uint8_t error_state;     // 错误状态
    uint8_t can_id;          // CAN ID(0-255)
	//控制参数
	float target_position;   // 目标位置(??)
    float max_speed;         // 最大速度(??/?)
    float max_torque;        // 最大扭矩(Nm)
    
    // 状态标志
    uint8_t is_enabled;      // 拾能
    uint8_t control_mode;    // 控制状态
} damiao_motor;



typedef struct
{
	float kp;
	float ki;
	float kd;
	float kf;
	
	float pout;
	float iout;
	float dout;
	float fout;
	float poutmax;
	float ioutmax;
	float foutmax;
	float I_Separate_Threshold;
	float doutmax;
	float outmax;
	uint8_t d_first_enable;
	float set;
	float preset;
	float real;
	float pre_real;
	float out;
	float out_last;
	float dead;
	float err;							
	float err_last;					
	float err_llast;				
	float integral;	
	float i_var_speed_a;              // variable-speed integral: ramp width
	float i_var_speed_b;              // variable-speed integral: full-speed window
    float dt;                         // controller period (s); if 0, treated as 1.0
    float lpf_alpha;                  // derivative low pass filter alpha
    first_order_filter_t d_filter;    // derivative low pass filter state
	
}PID_T;


extern volatile dji_motor FEED,SHOOT_L,SHOOT_R;

extern volatile damiao_motor Yaw_damiao;

//void GMmotor_run (float setPit,float setYaw,float setRoll,float Pit_Groy,float Yaw_Groy,float Roll_Groy, float Pit_ang, float Yaw_ang,float Roll_ang);
void GMmotor_run (float setPit,float setRoll,float Pit_Groy,float Roll_Groy, float Pit_ang, float Roll_ang);
void STmotor_run (float setPit,float setYaw,float Pit_Groy,float Yaw_Groy, float Pit_ang, float Yaw_ang);
void PID_Init_All(void);
//void GMPID_Init_Hand(PID_T*GMPitchPid_Ang,PID_T*GMYawPid_Ang,PID_T*GMrollPid_Ang,PID_T*GMPitchPid_Spe,PID_T*GMYawPid_Spe,PID_T*GMrollPid_Spe);
//void GMPID_Init_Zimiao(PID_T*GMPitchPid_Ang_ZM,PID_T*GMYawPid_Ang_ZM,PID_T*GMPitchPid_Spe_ZM,PID_T*GMYawPid_Spe_ZM);
void FirePID_Init(PID_T *Feedpid_Spe,PID_T*STLpid_Spe,PID_T*STRpid_Spe);
void Firemotor_run (float L_setST,float R_setST,float L_realspe,float R_realspe,float Feedget_spe,float Feedset_spe) ;
void parseDamiaoFeedback(uint8_t *data, damiao_motor *motor);
void GMPID_Init_Hand(PID_T*GMPitchPid_Ang,PID_T*GMYawPid_Ang,PID_T*GMrollPid_Ang,PID_T*GMPitchPid_Spe,PID_T*GMYawPid_Spe,PID_T*GMrollPid_Spe,PID_T*Dm4310_Ang,PID_T*Dm4310_Spe);
void GMPID_Init_Zimiao(PID_T*GMPitchPid_Ang_ZM,PID_T*GMYawPid_Ang_ZM,PID_T*GMPitchPid_Spe_ZM,PID_T*GMYawPid_Spe_ZM,PID_T*Dm4310_Ang_ZM,PID_T*Dm4310_Spe_ZM);
#endif
