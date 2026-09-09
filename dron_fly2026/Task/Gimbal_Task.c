#include "rc.h"
#include "motor.h"
#include "main.h"
#include <math.h>
#define TORQUE_DEADZONE 0.05f  
GIMBAL_control gimbal_control = {0};   
float motor1_angle,motor1_angle_set;
extern extKalman_t yaw_RV;
extern int ZM_inter;
extern float ZM_last_yaw;
extern Status_c status_c;
float target_pos;
float yaw_input;
extern float PID_Cal(PID_T *pid, float get, double set);
extern PID_T Dm4310_Ang;
extern PID_T Dm4310_Spe;
extern struct SolveTrajectoryParams st;
float torque_set,pos_angle;
static int status_c_last = -1;
static int status_zimiao_last = -1;
//extern VT13_ctrl_t VT13_ctrl;
int zimiao_flag = 0;                  //
extern dji_motor Roll;
int Pan = 0;
float my_yaw, now_yaw;
float target_yaw = 0.0f; 
uint8_t motor_enabled = 0;
uint8_t motor_enabled_ok = 0;
float diff_xianwei,mech_roll;
float roll_limit_lock;
float ceshi;
void calibrate_motor_zero_point(void)
{
    save_pos_zero(&hcan2, motor[Motor1].id, MIT_MODE);
    osDelay(100);
}

void damiao_handler(void)
{
	  if (vt13_rc.mode_sw==2) return;
    Pan++;
    if (Pan >= 500)
    {
        dm4340_motor_init();
        osDelay(100);
        dm4310_enable(&hcan2, &motor[Motor1]);
//        save_pos_zero(&hcan2, motor[Motor1].id,  MIT_MODE);

    }
}

extern uint8_t imu_ready_flag;
uint32_t hcan1_dead_flag1 = 0,hcan2_dead_flag = 0,hcan1_dead_flag2 = 0;
float roll_error,roll_error_dis;

void Gimbal_Control(void const * argument)
{
    PID_Init_All();
//	 save_pos_zero(&hcan2, motor[Motor1].id,  MIT_MODE);
    osDelay(50);
    target_yaw = imu_angle.yaw_real;
//    gimbal_control.pitch_set = imu_angle.pitch;  //
    while(imu_ready_flag == 0) { osDelay(1); }
  
    for (;;)
    {
			hcan1_dead_flag1 ++;
			hcan1_dead_flag2 ++;
			hcan2_dead_flag ++;
        damiao_handler();
//    save_pos_zero(&hcan2, motor[Motor1].id,  MIT_MODE);
       diff_xianwei=Roll.ecd_angle-Roll.init_angle;  //7; -150  0          -170  3           -185  5
////            			-115  -5
//			// ===================== Roll =====================
//			       mech_roll=diff_xianwei/7.0f;
//             if ((diff_xianwei < (70 )) && (diff_xianwei > (-70)))
//								{
									gimbal_control.roll_set = -1.0f;
//								}
//								else	gimbal_control.roll_set = imu_angle.roll;
								
//							roll_error = gimbal_control.roll_set - imu_angle.roll;
//							if(roll_error>8) 
//								gimbal_control.roll_set += (roll_error-8);
//							else if(roll_error<-8)
//								gimbal_control.roll_set -= (roll_error+8);

			if (vt13_rc.mode_sw==2) // 下电
        {
            PHC_GIMBAL(0,0);
            Set_Shoot_Motor_Current(0, 0, 0, 0);
					  motor[Motor1].ctrl.pos_set=0;
	          motor[Motor1].ctrl.kp_set=0;//4
	          motor[Motor1].ctrl.kd_set=0;//0.4
	          motor[Motor1].ctrl.tor_set=0;
	          motor[Motor1].ctrl.vel_set=0;//1
            dm4310_ctrl_send(&hcan2,&motor[Motor1]);
					   Pan = 0;
					
//					disable_motor_mode(&hcan1, motor[Motor1].id, MIT_MODE);
            osDelay(1);
            continue; 
        }

        if (vt13_rc.mode_sw==0||vt13_rc.mode_sw == 1) // 手瞄模式
        {
            status_c = SHOUMIAO;
        }
        if ((vt13_rc.mouse_right==1||vt13_rc.fn_left==1)&& (vt13_rc.mode_sw==1)) // 自瞄模式
        {
            status_c = ZIMIAO;
        }
        
        if (status_c_last != status_c)
        {
//            target_yaw = imu_angle.yaw_real;
            status_c_last = status_c;
					  zimiao_flag =0;
        }
        
        // ==========================================
        switch (status_c)
        {//
            case SHOUMIAO:
//							 
                gimbal_control.pitch_set -= vt13_rc.mouse_y/500.0f+ vt13_rc.ch2/1000.0f;//
                 yaw_input =vt13_rc.mouse_x/500.0f+ vt13_rc.ch3/1000.0f;//

						if(target_yaw < imu_angle.yaw_real + pos_angle - 55.0f)	target_yaw = (imu_angle.yaw_real-(55.0f-pos_angle));
						else if(target_yaw - imu_angle.yaw_real>pos_angle+55.0f)	target_yaw = imu_angle.yaw_real+pos_angle+55.0f;
						else	target_yaw -= yaw_input;
                break; 
             

            case ZIMIAO:
                
                 if (status_zimiao_last != status_zimiao)
                {
//                    target_yaw = imu_angle.yaw_real;
                    status_zimiao_last = status_zimiao;
                }
                 
                switch (status_zimiao)
                {
                    case MEIMIAODAO: // 0
                         gimbal_control.pitch_set -= vt13_rc.mouse_y/500.0f+ vt13_rc.ch2/1000.0f;
                 yaw_input =vt13_rc.mouse_x/500.0f+ vt13_rc.ch3/1000.0f;
                if(pos_angle > 55.0f && yaw_input < 0.0f)	yaw_input = 0.0f;	//限幅
												if(pos_angle < -55.0f && yaw_input > 0.0f)	yaw_input = 0.0f;	//限幅
													target_yaw -= yaw_input;
												if(pos_angle >55.0f)	target_yaw = imu_angle.yaw_real;
												if(pos_angle <-55.0f)	target_yaw = imu_angle.yaw_real;
													zimiao_flag = 0;
                        break;

                    case MIAODAO: // 1
                        gimbal_control.pitch_set = gimbal_control.pitch_zimiao;
                     target_yaw = gimbal_control.yaw_zimiao;
												if(target_yaw < imu_angle.yaw_real + pos_angle - 55.0f)	target_yaw = (imu_angle.yaw_real-(55.0f-pos_angle));
												if(target_yaw - imu_angle.yaw_real>pos_angle+55.0f)	target_yaw = imu_angle.yaw_real+pos_angle+55.0f;
                        zimiao_flag = 1;
										break;

                    case MIAODAOKAIHUO: // 2
                        gimbal_control.pitch_set = gimbal_control.pitch_zimiao;
                          target_yaw = gimbal_control.yaw_zimiao;
												if(target_yaw < imu_angle.yaw_real + pos_angle - 55.0f)	target_yaw = (imu_angle.yaw_real-(55.0f-pos_angle));
												if(target_yaw - imu_angle.yaw_real>pos_angle+55.0f)	target_yaw = imu_angle.yaw_real+pos_angle+55.0f;
                        zimiao_flag = 1;
										break;


                    default:
                        
                        break;
                }
                break;

             default:
                break;
        }
     
        // ===================== 限位=====================
        if (gimbal_control.pitch_set > 34) gimbal_control.pitch_set = 34;
        else if (gimbal_control.pitch_set < -6) gimbal_control.pitch_set = -6;

         pos_angle=motor[Motor1].para.pos*(180/PI);

			 float m6 = 0.0f;
					 if (vt13_rc.mode_sw!=2 ) // 非急停模式都使能Yaw电机
        {

								// 正常 PID 控制
								
								m6 = PID_Cal(&Dm4310_Ang, imu_angle.yaw_real, target_yaw); // Yaw Angle
								torque_set = PID_Cal(&Dm4310_Spe, imu_angle.gyro_z, m6);   // Yaw Spee
					
           	if(hcan2_dead_flag>500)	torque_set = 0.0f;
					  motor[Motor1].ctrl.tor_set=torque_set;
		    		dm4310_ctrl_send(&hcan2, &motor[Motor1]);
//            
        }
        GMmotor_run(gimbal_control.pitch_set, gimbal_control.roll_set,
                    imu_angle.gyro_y, imu_angle.gyro_x,
                    imu_angle.pitch, imu_angle.roll);

        osDelay(2);
    }
}
