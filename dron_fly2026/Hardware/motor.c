#include "main.h"

uint8_t usedata[8];
#define RATE_BUF_SIZE 6

#define MYABS(a)    ((a >= 0) ? (a) : -(a)) 
#define VALUE_LIMIT(v, l, h)   ((v) < (l) ? (l) : ((v) > (h) ? (h) : (v)))
#define VALUE_ABS_LIMIT(v, a)  VALUE_LIMIT(v, -a, a)
volatile dji_motor Pitch={0,0,0,0,0,0,0,0,0,0,0,0};
volatile dji_motor Yaw={0,0,0,0,0,0,0,0,0,0,0,0};
volatile dji_motor Roll={0,0,0,0,0,0,0,0,0,0,0,0};
volatile dji_motor FEED,SHOOT_L,SHOOT_R;
volatile damiao_motor Yaw_damiao;

float gravity_ff ;
PID_T GMPitchPid_Ang={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
PID_T GMPitchPid_Spe={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
PID_T GMYawPid_Ang={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
PID_T GMYawPid_Spe={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
PID_T GMrollPid_Ang={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
PID_T GMrollPid_Spe={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	 
PID_T GMPitchPid_Ang_ZM={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
PID_T GMPitchPid_Spe_ZM={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
PID_T GMYawPid_Ang_ZM={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
PID_T GMYawPid_Spe_ZM={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
PID_T STLpid_Spe={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
PID_T STRpid_Spe={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
PID_T Feedpid_Spe={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
PID_T Dm4310_Ang={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
PID_T Dm4310_Spe={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
PID_T Dm4310_Ang_ZM={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
PID_T Dm4310_Spe_ZM={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
////////////vofa+
//typedef union{
//	uint8_t vu[3];
//	uint8_t vn[3];
//}typedef_vofa;

//typedef_vofa sendvofa1;
//typedef_vofa sendvofa2;

extern int Pan;
extern float mech_roll,diff_xianwei;
extern float roll_limit_lock;
float MR,mR,MP;
static uint8_t encoder_init_count = 0;
void getEncoderData(volatile dji_motor *v)
{
	int i=0;
	int32_t temp_sum = 0;    
	v->last_raw_value = v->raw_value;
	v->raw_value = (usedata[0]<<8)|usedata[1];
	v->diff = v->raw_value - v->last_raw_value;
	if(v->diff < -4096)    //���α������ķ���ֵ���̫�󣬱�ʾȦ�������˸ı�
	{
		v->round_cnt++;
		v->ecd_raw_rate = v->diff + 8192;
	}
	else if(v->diff>4096)
	{
		v->round_cnt--;
		v->ecd_raw_rate = v->diff - 8192;
	}		
	else
	{
		v->ecd_raw_rate = v->diff;
	}
	//����õ������ı��������ֵ
	v->ecd_value = v->raw_value + v->round_cnt * 8192;
	//����õ��Ƕ�ֵ����Χ���������
	v->ecd_angle = (float)(v->raw_value - v->ecd_bias)*360/8192 + v->round_cnt * 360;
	v->rate_buf[v->buf_count++] = v->ecd_raw_rate;
	if(v->buf_count == RATE_BUF_SIZE)
	{
		v->buf_count = 0;
	}
	
	
	//�����ٶ�ƽ��ֵ
	for(i = 0;i < RATE_BUF_SIZE; i++)
	{
		temp_sum += v->rate_buf[i];
	}
	v->filter_rate = (int32_t)(temp_sum/RATE_BUF_SIZE);
	
	if(fabs((float)v->filter_rate_max) < fabs((float)v->filter_rate))
	{
		v->filter_rate_max = v->filter_rate;
	}
	v->current = (uint16_t)(usedata[4] << 8 | usedata[5]);
	v->tempature = usedata[6];
	v->re_speed= (uint16_t)(usedata[2] << 8 | usedata[3]);//�ٶ������
	
	if(v->flag==0){
		encoder_init_count++;
		if(encoder_init_count > 10){
			v->init_angle=v->ecd_angle;
			v->flag=1;
		}
	}
}
extern uint32_t hcan1_dead_flag1,hcan1_dead_flag2,hcan2_dead_flag;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{   
  CAN_RxHeaderTypeDef	RxHeader;
  if(hcan->Instance==CAN1)
	{
		HAL_CAN_GetRxMessage(&hcan1,CAN_FILTER_FIFO0,&RxHeader,usedata);
		switch(RxHeader.StdId)
		{	
			
			case 0x202:getEncoderData(&SHOOT_L); break;     //L
  		case 0x203 :getEncoderData(&SHOOT_R);  break;   //R	
      case 0x204 : getEncoderData(&FEED);  break;     //FEED
			
			case 0x205 : getEncoderData(&Pitch);  hcan1_dead_flag1 = 0;	 break;  //6020���  3508
//	  case 0x206 : getEncoderData(&Yaw);   break;	
      case 0x206 : getEncoderData(&Roll);  	hcan1_dead_flag2 = 0;		break;	

		}	
	}
	if(hcan->Instance==CAN2)
	{
		HAL_CAN_GetRxMessage(&hcan2,CAN_FILTER_FIFO0,&RxHeader,usedata);
        switch(RxHeader.StdId)                   
        {
				
			case 0x00  : dm4310_fbdata(&motor[Motor1], usedata);
					hcan2_dead_flag = 0;
	       Pan=0;break;
		
	    }
    }
}


float PID_Cal(PID_T *pid, float get, double set)
{	
	float abs_error;
	float speed_ratio = 1.0f;
	float dt = (pid->dt > 0.0f) ? pid->dt : 1.0f;
//dead_zone 
	pid->set = set;
	pid->real = get;

	pid->err_last = pid->err;
	pid->err = pid->set - pid->real;
	
	abs_error = fabs(pid->err);

	if (abs_error < pid->dead)
	{
		pid->set = pid->real;
		pid->err = 0.0f;
		abs_error = 0.0f;
	}

	if ((pid->i_var_speed_a == 0.0f) && (pid->i_var_speed_b == 0.0f))
	{
		speed_ratio = 1.0f;
	}
	else if (abs_error <= pid->i_var_speed_b)
	{
		speed_ratio = 1.0f;
	}
	else if ((abs_error > pid->i_var_speed_b) && (abs_error < pid->i_var_speed_a + pid->i_var_speed_b))
	{
		speed_ratio = (pid->i_var_speed_a + pid->i_var_speed_b - abs_error) / pid->i_var_speed_a;
	}
	else
	{
		speed_ratio = 0.0f;
	}

	pid->pout = pid->kp * pid->err;
	pid->pout = VALUE_LIMIT(pid->pout, -pid->poutmax, pid->poutmax);

	if (pid->I_Separate_Threshold == 0.0f || abs_error < pid->I_Separate_Threshold)
	{
		pid->integral += speed_ratio * dt * pid->err;
		pid->iout = pid->ki * pid->integral;
	}
	else
	{
		pid->integral = 0.0f;
		pid->iout = 0.0f;
	}
	pid->iout = VALUE_LIMIT(pid->iout, -pid->ioutmax, pid->ioutmax);

	float raw_dout;
	if (pid->d_first_enable)
	{
		raw_dout = -pid->kd * (pid->real- pid->pre_real) / dt;
	}
	else
	{
		raw_dout = pid->kd * (pid->err - pid->err_last) / dt;
	}
	float alpha = VALUE_LIMIT(pid->lpf_alpha, 0.0f, 1.0f);
	pid->dout = first_order_filter(&pid->d_filter, raw_dout, alpha);
	pid->dout = VALUE_LIMIT(pid->dout, -pid->doutmax, pid->doutmax);

	pid->fout = (pid->set - pid->preset) * pid->kf;
	pid->fout = VALUE_LIMIT(pid->fout, -pid->foutmax, pid->foutmax);

	pid->out = pid->pout + pid->iout + pid->dout + pid->fout;
	pid->out = VALUE_LIMIT(pid->out, -pid->outmax, pid->outmax);

	pid->preset = pid->set;
	pid->pre_real = pid->real;
	pid->out_last = pid->out;
	return pid->out;
}

// ======================pitch��������
     //��������

#define GRAVITY_TABLE_SIZE  5


typedef struct {
    float angle_deg;   // 
    float torque;      //
} GravityPoint;
const GravityPoint GravityTable[GRAVITY_TABLE_SIZE] = {
    {0.0f, 4500.0f },
    {  10.0f, 3000.0f },
    {  20.0f, 2200.0f },
    {  30.0f, 1500.0f },
    {  34.0f, 1200.0f }
};

float Gravity_Compensation(float pitch_deg)
{
   
    if(pitch_deg <= GravityTable[0].angle_deg)
        return GravityTable[0].torque;
    if(pitch_deg >= GravityTable[GRAVITY_TABLE_SIZE-1].angle_deg)
        return GravityTable[GRAVITY_TABLE_SIZE-1].torque;

    for(int i = 0; i < GRAVITY_TABLE_SIZE - 1; i++)
    {
        if(pitch_deg >= GravityTable[i].angle_deg && 
           pitch_deg < GravityTable[i+1].angle_deg)
        {
            float ratio = (pitch_deg - GravityTable[i].angle_deg) /
                          (GravityTable[i+1].angle_deg - GravityTable[i].angle_deg);
            return GravityTable[i].torque + 
                   ratio * (GravityTable[i+1].torque - GravityTable[i].torque);
        }
    }
    return 0.0f;
}

    
// 0��4700    30��1600

float gyro_display_for_roll = 0,diskp =-10;

void GMmotor_run (float setPit,float setRoll,float Pit_Groy,float Roll_Groy, float Pit_ang, float Roll_ang)
{ 

	float m0,m1,m2,m3=0;
	float M0,M1,M2,M3=0;

	gravity_ff = Gravity_Compensation(imu_angle.pitch);
	gyro_display_for_roll = (imu_angle.gyro_z * diskp)*((0-imu_angle.pitch)/35);
	
	if(vt13_rc.mode_sw==0||vt13_rc.mode_sw==1)//手瞄
	{
		
	m0 = PID_Cal(&GMPitchPid_Ang, Pit_ang, setPit);//Pit 
	MP = -PID_Cal(&GMPitchPid_Spe, Pit_Groy, m0) +gravity_ff;//Pit  
		
		mR=PID_Cal(&GMrollPid_Ang,Roll_ang,setRoll);
		MR=PID_Cal(&GMrollPid_Spe,Roll_Groy,mR)
    		+ gyro_display_for_roll;

	}
	else if((vt13_rc.mouse_right==1||vt13_rc.fn_left==1)&& (vt13_rc.mode_sw==1))	//自瞄
   {
	  m0 = PID_Cal(&GMPitchPid_Ang_ZM, Pit_ang, setPit);
	  MP = -PID_Cal(&GMPitchPid_Spe_ZM, Pit_Groy, m0)+gravity_ff;//
		 
     mR=PID_Cal(&GMrollPid_Ang,Roll_ang,setRoll);
		 MR=PID_Cal(&GMrollPid_Spe,Roll_Groy,mR);
	
   }
				if(hcan1_dead_flag1>500)	MP = 0.0f;
				if(hcan1_dead_flag2>500)	MR = 0.0f;
  PHC_GIMBAL(MP,MR);                         //          56
}

//
void Firemotor_run (float L_setST,float R_setST,float L_realspe,float R_realspe,float Feedget_spe,float Feedset_spe) 
{
	
	float M0,M1,M2,M3 =0;
	M1 = PID_Cal(&STLpid_Spe, L_realspe, L_setST);
	M2 = PID_Cal(&STRpid_Spe, R_realspe,-R_setST);
  
	M3 = PID_Cal(&Feedpid_Spe, Feedget_spe ,Feedset_spe );

   Set_Shoot_Motor_Current(M0,M1,M2,M3); 

}

/*����pid*/
void GMPID_Init_Hand(PID_T*GMPitchPid_Ang,PID_T*GMYawPid_Ang,PID_T*GMrollPid_Ang,PID_T*GMPitchPid_Spe,PID_T*GMYawPid_Spe,PID_T*GMrollPid_Spe,PID_T*Dm4310_Ang,PID_T*Dm4310_Spe)
{
//	GMPitchPid_Ang->kp = 25.0f;//30
//	GMPitchPid_Ang->ki = 0.001f;//0.01
//	GMPitchPid_Ang->kd = 3.0f;//5
//	GMPitchPid_Ang->poutmax=12000.0f;
//	GMPitchPid_Ang->ioutmax=100.0f;
//	GMPitchPid_Ang->doutmax=12000.0f;
//	GMPitchPid_Ang->outmax=12000.0f;
	GMPitchPid_Ang->kp = 30.0f;//p         30
	GMPitchPid_Ang->ki = 0.0f;//i           0
	GMPitchPid_Ang->kd = 0.15f;//d            0.4
	GMPitchPid_Ang->kf = 0.0f;//1000ǰ��  50
	GMPitchPid_Ang->d_first_enable = 0;//����
	GMPitchPid_Ang->dead=0.0f;//����
	GMPitchPid_Ang->I_Separate_Threshold=1;//1
	GMPitchPid_Ang->i_var_speed_a = 0.0f;//��������
	GMPitchPid_Ang->i_var_speed_b = 0.0f;//��������
	GMPitchPid_Ang->dt = 0.001f;
	GMPitchPid_Ang->poutmax=12000.0f;//������
	GMPitchPid_Ang->ioutmax=4000.0f;
	GMPitchPid_Ang->doutmax=12000.0f;
	GMPitchPid_Ang->foutmax=2000.0f;
	GMPitchPid_Ang->outmax=16000.0f;
	GMPitchPid_Ang->lpf_alpha=0.2f;//΢�ֵ�ͨ
	GMPitchPid_Ang->d_filter.out=0.0f;
	GMPitchPid_Ang->out_last=0.0f;
	
	
	
//	GMPitchPid_Spe->kp = -26.0f;  //-35
//	GMPitchPid_Spe->ki = 0.0f;//0.05
//	GMPitchPid_Spe->kd =2.0f;//3
//	GMPitchPid_Spe->poutmax=30000.0f;
//	GMPitchPid_Spe->ioutmax=1000.0f;
//	GMPitchPid_Spe->doutmax=10000.0f;
//	GMPitchPid_Spe->outmax=30000.0f;
  GMPitchPid_Spe->kp =22.0f;  //    22
	GMPitchPid_Spe->ki = 0.00f;//   0
	GMPitchPid_Spe->kd = 0.0f;           // 0
	GMPitchPid_Spe->kf =0.0f;//100      50
	GMPitchPid_Spe->d_first_enable = 0;
	GMPitchPid_Spe->dead=0.1f;//
	GMPitchPid_Spe->I_Separate_Threshold=0;
	GMPitchPid_Spe->i_var_speed_a = 0.0f;
	GMPitchPid_Spe->i_var_speed_b = 0.0f;
	GMPitchPid_Spe->dt = 0.001f;
	GMPitchPid_Spe->poutmax=20000.0f;
	GMPitchPid_Spe->ioutmax=4000.0f;
	GMPitchPid_Spe->doutmax=12000.0f;
	GMPitchPid_Spe->foutmax=0.0f;
	GMPitchPid_Spe->outmax=20000.0f;
	GMPitchPid_Spe->lpf_alpha=0.1f;
	GMPitchPid_Spe->d_filter.out=0.0f;
	GMPitchPid_Spe->out_last=0.0f;
	
	
//	GMrollPid_Ang->kp = 0.0f;//28
//	GMrollPid_Ang->ki = 0.00f;//0.00
//	GMrollPid_Ang->kd = 0.0f;//5
//	GMrollPid_Ang->poutmax=12000.0f;
//	GMrollPid_Ang->ioutmax=100.0f;
//	GMrollPid_Ang->doutmax=12000.0f;
//	GMrollPid_Ang->outmax=12000.0f;
	GMrollPid_Ang->kp = 50.0f;//p   32     30
	GMrollPid_Ang->ki =  0.0f;//i  0.0003
	GMrollPid_Ang->kd =  0.0f;//d
	GMrollPid_Ang->kf =  0.0f;//1000ǰ��
	GMrollPid_Ang->d_first_enable = 0;//����
	GMrollPid_Ang->dead=0.1f;//����
	GMrollPid_Ang->I_Separate_Threshold=4;//���ַ���//4
	GMrollPid_Ang->i_var_speed_a = 0.0f;//��������
	GMrollPid_Ang->i_var_speed_b = 0.0f;//��������
	GMrollPid_Ang->dt = 0.001f;
	GMrollPid_Ang->poutmax=22000.0f;//������
	GMrollPid_Ang->ioutmax=4000.0f;
	GMrollPid_Ang->doutmax=12000.0f;
	GMrollPid_Ang->foutmax=2000.0f;
	GMrollPid_Ang->outmax=16000.0f;
	GMrollPid_Ang->lpf_alpha=0.1f;//΢�ֵ�ͨ
	GMrollPid_Ang->d_filter.out=0.0f;
	GMrollPid_Ang->out_last=0.0f;
	
	
//	GMrollPid_Spe->kp = 0.0f;  //-28
//	GMrollPid_Spe->ki = 0.0f;//-0.01
//	GMrollPid_Spe->kd = 0.0f;//3
//	GMrollPid_Spe->poutmax=30000.0f;
//	GMrollPid_Spe->ioutmax=1000.0f;
//	GMrollPid_Spe->doutmax=10000.0f;
//	GMrollPid_Spe->outmax=30000.0f;
	GMrollPid_Spe->kp = -25.0f;//p  -32    
	GMrollPid_Spe->ki = -0.0f;//i  -0.0005
	GMrollPid_Spe->kd = 0.0f;//d
	GMrollPid_Spe->kf = 0.0f;//1000ǰ��
	GMrollPid_Spe->d_first_enable = 0;//����
	GMrollPid_Spe->dead=0.1f;//����
	GMrollPid_Spe->I_Separate_Threshold=6;//���ַ���//4
	GMrollPid_Spe->i_var_speed_a = 0.0f;//��������
	GMrollPid_Spe->i_var_speed_b = 0.0f;//��������
	GMrollPid_Spe->dt = 0.001f;
	GMrollPid_Spe->poutmax=20000.0f;//������
	GMrollPid_Spe->ioutmax=2000.0f;
	GMrollPid_Spe->doutmax=12000.0f;
	GMrollPid_Spe->foutmax=2000.0f;
	GMrollPid_Spe->outmax=28000.0f;
	GMrollPid_Spe->lpf_alpha=0.1f;//΢�ֵ�ͨ
	GMrollPid_Spe->d_filter.out=0.0f;
	GMrollPid_Spe->out_last=0.0f;
	
	

//	Dm4310_Ang->kp = 35.0f;//30w
//	Dm4310_Ang->ki = 0.15f;//0��01
//	Dm4310_Ang->kd = 0.0f;//4
//	Dm4310_Ang->poutmax=12000.0f;
//	Dm4310_Ang->ioutmax=100.0f;
//	Dm4310_Ang->doutmax=12000.0f;
//	Dm4310_Ang->outmax=12000.0f;
//  Dm4310_Ang->dead=0.2f;//����
	
	Dm4310_Ang->kp = 24.0f;//p  24
	Dm4310_Ang->ki = 0.0f;//i  0.2
	Dm4310_Ang->kd = 0.00f;//d
	Dm4310_Ang->kf = 0.0f;//1000ǰ��   5
	Dm4310_Ang->d_first_enable = 0;//����
	Dm4310_Ang->dead=0.1f;//����0.1
	Dm4310_Ang->I_Separate_Threshold=0;//���ַ���//8
	Dm4310_Ang->i_var_speed_a = 0.0f;//��������
	Dm4310_Ang->i_var_speed_b = 0.0f;//��������
	Dm4310_Ang->dt = 0.001f;
	Dm4310_Ang->poutmax=12000.0f;//������
	Dm4310_Ang->ioutmax=100.0f;
	Dm4310_Ang->doutmax=12000.0f;
	Dm4310_Ang->foutmax=2000.0f;
	Dm4310_Ang->outmax=12000.0f;
	Dm4310_Ang->lpf_alpha=0.1f;//΢�ֵ�ͨ
	Dm4310_Ang->d_filter.out=0.0f;
	Dm4310_Ang->out_last=0.0f;

//	Dm4310_Spe->kp = -50.0f;  //30
//	Dm4310_Spe->ki = -0.02f;//0.15
//	Dm4310_Spe->kd = 2.0f;//4
//	Dm4310_Spe->poutmax=30000.0f;
//	Dm4310_Spe->ioutmax=1000.0f;
//	Dm4310_Spe->doutmax=10000.0f;
//	Dm4310_Spe->outmax=30000.0f;
//	Dm4310_Spe->dead=0.2f;   //����
  Dm4310_Spe->kp = -32.0f;//p  -32
	Dm4310_Spe->ki = 0.0f;//i  -0.001
	Dm4310_Spe->kd = 0.0005f;//d
	Dm4310_Spe->kf = 0.0f;//1000ǰ��   50
	Dm4310_Spe->d_first_enable = 0;//����
	Dm4310_Spe->dead=0.1f;//���� 0.1
	Dm4310_Spe->I_Separate_Threshold=0;//���ַ���//8
	Dm4310_Spe->i_var_speed_a = 0.0f;//��������
	Dm4310_Spe->i_var_speed_b = 0.0f;//��������
	Dm4310_Spe->dt = 0.001f;
	Dm4310_Spe->poutmax=30000.0f;//������
	Dm4310_Spe->ioutmax=1000.0f;
	Dm4310_Spe->doutmax=10000.0f;
	Dm4310_Spe->foutmax=2000.0f;
	Dm4310_Spe->outmax=16000.0f;
	Dm4310_Spe->lpf_alpha=0.1f;//΢�ֵ�ͨ
	Dm4310_Spe->d_filter.out=0.0f;
	Dm4310_Spe->out_last=0.0f;

}

//����
void GMPID_Init_Zimiao(PID_T*GMPitchPid_Ang_ZM,PID_T*GMYawPid_Ang_ZM,PID_T*GMPitchPid_Spe_ZM,PID_T*GMYawPid_Spe_ZM,PID_T*Dm4310_Ang_ZM,PID_T*Dm4310_Spe_ZM)
{
//	GMPitchPid_Ang_ZM->kp = 25.0f;//38
//	GMPitchPid_Ang_ZM->ki = 0.001f;//0.01
//	GMPitchPid_Ang_ZM->kd = 2.0f;//5
//	GMPitchPid_Ang_ZM->poutmax=10000.0f;
//	GMPitchPid_Ang_ZM->ioutmax=100.0f;
//	GMPitchPid_Ang_ZM->doutmax=5000.0f;
//	GMPitchPid_Ang_ZM->outmax=10000.0f;
	GMPitchPid_Ang_ZM->kp = 30.0f;//p   70
	GMPitchPid_Ang_ZM->ki = 0.000f;//i  0.00085
	GMPitchPid_Ang_ZM->kd = 0.0f;//d
	GMPitchPid_Ang_ZM->kf = 0.0f;//1000
	GMPitchPid_Ang_ZM->d_first_enable = 0;//
	GMPitchPid_Ang_ZM->dead=0.1f;//
	GMPitchPid_Ang_ZM->I_Separate_Threshold=0;//
	GMPitchPid_Ang_ZM->i_var_speed_a = 0.0f;//
	GMPitchPid_Ang_ZM->i_var_speed_b = 0.0f;//
	GMPitchPid_Ang_ZM->dt = 0.001f;
	GMPitchPid_Ang_ZM->poutmax=22000.0f;//
	GMPitchPid_Ang_ZM->ioutmax=200.0f;
	GMPitchPid_Ang_ZM->doutmax=12000.0f;
	GMPitchPid_Ang_ZM->foutmax=2000.0f;
	GMPitchPid_Ang_ZM->outmax=16000.0f;
	GMPitchPid_Ang_ZM->lpf_alpha=0.1f;//
	GMPitchPid_Ang_ZM->d_filter.out=0.0f;
	GMPitchPid_Ang_ZM->out_last=0.0f;
	
//	GMPitchPid_Spe_ZM->kp = -26.0f; //-45
//	GMPitchPid_Spe_ZM->ki = 0.0f;//0.05
//	GMPitchPid_Spe_ZM->kd = 3.0f;//3.0
//	GMPitchPid_Spe_ZM->poutmax=10000.0f;
//	GMPitchPid_Spe_ZM->ioutmax=1000.0f;
//	GMPitchPid_Spe_ZM->doutmax=10000.0f;
//	GMPitchPid_Spe_ZM->outmax=10000.0f;
	GMPitchPid_Spe_ZM->kp = 20.0f;//p -40
	GMPitchPid_Spe_ZM->ki = 0.00f;//i -0.05  0
	GMPitchPid_Spe_ZM->kd = 0.0f;//d
	GMPitchPid_Spe_ZM->kf = 0.0f;//1000
	GMPitchPid_Spe_ZM->d_first_enable = 0;//
	GMPitchPid_Spe_ZM->dead=0.1f;//
	GMPitchPid_Spe_ZM->I_Separate_Threshold=0;//
	GMPitchPid_Spe_ZM->i_var_speed_a = 0.0f;//
	GMPitchPid_Spe_ZM->i_var_speed_b = 0.0f;//
	GMPitchPid_Spe_ZM->dt = 0.001f;
	GMPitchPid_Spe_ZM->poutmax=22000.0f;//
	GMPitchPid_Spe_ZM->ioutmax=1200.0f;
	GMPitchPid_Spe_ZM->doutmax=12000.0f;
	GMPitchPid_Spe_ZM->foutmax=2000.0f;
	GMPitchPid_Spe_ZM->outmax=20000.0f;
	GMPitchPid_Spe_ZM->lpf_alpha=0.1f;//
	GMPitchPid_Spe_ZM->d_filter.out=0.0f;
	GMPitchPid_Spe_ZM->out_last=0.0f;
	
//	Dm4310_Ang_ZM->kp = 0.0f;//17
//	Dm4310_Ang_ZM->ki = 0.0f;//0��01
//	Dm4310_Ang_ZM->kd = 0.0f;//5
//	Dm4310_Ang_ZM->poutmax=12000.0f;
//	Dm4310_Ang_ZM->ioutmax=100.0f;
//	Dm4310_Ang_ZM->doutmax=12000.0f;
//	Dm4310_Ang_ZM->outmax=12000.0f;
//	
//	Dm4310_Spe_ZM->kp = 0.0f;  
//	Dm4310_Spe_ZM->ki = 0.01f;
//	Dm4310_Spe_ZM->kd = 0.0f;
//	Dm4310_Spe_ZM->poutmax=30000.0f;
//	Dm4310_Spe_ZM->ioutmax=1000.0f;
//	Dm4310_Spe_ZM->doutmax=10000.0f;
//	Dm4310_Spe_ZM->outmax=30000.0f;
	
}

/*����*/
void FirePID_Init(PID_T *Feedpid_Spe,PID_T*STLpid_Spe,PID_T*STRpid_Spe)
{
	
//	Feedpid_Spe->kp = 20.0f ;//32
//	Feedpid_Spe->ki = 0.0003f ;//0
//	Feedpid_Spe->kd = 3.0f ;//2
//	Feedpid_Spe->poutmax = 16000.0f ;
//	Feedpid_Spe->ioutmax = 2000.0f;
//	Feedpid_Spe->doutmax = 1000.0f ;
//	Feedpid_Spe->outmax  =  16000.0f;
	Feedpid_Spe->kp = 18.0f ;//4.2  18
	Feedpid_Spe->ki = 0.0f ;
	Feedpid_Spe->kd = 0.0f ;//10   2
	Feedpid_Spe->d_first_enable = 0;
	Feedpid_Spe->I_Separate_Threshold=0;//100
	Feedpid_Spe->i_var_speed_a = 0.0f;
	Feedpid_Spe->i_var_speed_b = 0.0f;
	Feedpid_Spe->dt = 0.001f;
	Feedpid_Spe->poutmax = 16000.0f ;
	Feedpid_Spe->ioutmax = 2000.0f;
	Feedpid_Spe->doutmax = 1000.0f ;
	Feedpid_Spe->outmax =  16000.0f;
	Feedpid_Spe->lpf_alpha=0.1f;
	Feedpid_Spe->d_filter.out=0.0f;
	Feedpid_Spe->out_last=0.0f;
	 
//	STLpid_Spe->kp = 55.0f;//66
//	STLpid_Spe->ki = 0.5f;
//	STLpid_Spe->kd = 3.0f;
//	STLpid_Spe->poutmax = 16384.0f ;
//	STLpid_Spe->ioutmax = 9000.0f;
//	STLpid_Spe->doutmax = 10000.0f ;
//	STLpid_Spe->outmax =  16384.0f;
	
	STLpid_Spe->kp = 32.0f;//p  52
	STLpid_Spe->ki = 0.0f;//i  0.33
	STLpid_Spe->kd = 0.0f;//d
	STLpid_Spe->kf = 0.0f;//1000ǰ��
	STLpid_Spe->d_first_enable = 0;//����
	STLpid_Spe->dead=0.1f;//����
	STLpid_Spe->I_Separate_Threshold=0;//���ַ���//4
	STLpid_Spe->i_var_speed_a = 0.0f;//��������
	STLpid_Spe->i_var_speed_b = 0.0f;//��������
	STLpid_Spe->dt = 0.001f;
	STLpid_Spe->poutmax=22000.0f;//������
	STLpid_Spe->ioutmax=200.0f;
	STLpid_Spe->doutmax=12000.0f;
	STLpid_Spe->foutmax=2000.0f;
	STLpid_Spe->outmax=16000.0f;
	STLpid_Spe->lpf_alpha=0.1f;//΢�ֵ�ͨ
	STLpid_Spe->d_filter.out=0.0f;
	STLpid_Spe->out_last=0.0f;
	
	
//	STRpid_Spe->kp = 55.0f;//69
//	STRpid_Spe->ki = 0.5f;//�ұ߻���С��
//	STRpid_Spe->kd = 3.0f;
//	STRpid_Spe->poutmax = 16384.0f ;
//	STRpid_Spe->ioutmax = 9000.0f ;
//	STRpid_Spe->doutmax = 10000.0f ;
//	STRpid_Spe->outmax =  16384.0f;
	
	STRpid_Spe->kp = 32.0f;//p  52
	STRpid_Spe->ki = 0.0f;//i  0.33
	STRpid_Spe->kd = 0.0f;//d
	STRpid_Spe->kf = 0.0f;//1000ǰ��
	STRpid_Spe->d_first_enable = 0;//����
	STRpid_Spe->dead=0.1f;//����
	STRpid_Spe->I_Separate_Threshold=0;//���ַ���//4
	STRpid_Spe->i_var_speed_a = 0.0f;//��������
	STRpid_Spe->i_var_speed_b = 0.0f;//��������
	STRpid_Spe->dt = 0.001f;
	STRpid_Spe->poutmax=22000.0f;//������
	STRpid_Spe->ioutmax=200.0f;
	STRpid_Spe->doutmax=12000.0f;
	STRpid_Spe->foutmax=2000.0f;
	STRpid_Spe->outmax=16000.0f;
	STRpid_Spe->lpf_alpha=0.1f;//΢�ֵ�ͨ
	STRpid_Spe->d_filter.out=0.0f;
	STRpid_Spe->out_last=0.0f;
}

void PID_Init_All(void)
{
  GMPID_Init_Hand(&GMPitchPid_Ang,&GMYawPid_Ang,&GMrollPid_Ang,&GMPitchPid_Spe,&GMYawPid_Spe,&GMrollPid_Spe,&Dm4310_Ang,&Dm4310_Spe);
	GMPID_Init_Zimiao(&GMPitchPid_Ang_ZM,&GMYawPid_Ang_ZM,&GMPitchPid_Spe_ZM,&GMYawPid_Spe_ZM,&Dm4310_Ang_ZM,&Dm4310_Spe_ZM);
	FirePID_Init(&Feedpid_Spe,&STLpid_Spe,&STRpid_Spe);
}
