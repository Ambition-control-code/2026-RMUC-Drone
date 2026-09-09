//#include "main.h"
//#include "rc.h"
//#include "uart_bsp.h"
//#include "Get_Referee_Data.h"   // 

//float L_setST, R_setST, Feed_set, Feed_ji = 3500;
//int shoot = 6000;
//extern int zimiao_flag;
//float fric_set_now = 0.0f;

//// 热量控制参数（模拟裁判系统，17mm 弹丸）
//#define HEAT_PER_BULLET           10.0f    // 每发增量 10（裁判系统真实值）
//#define HEAT_SAFE_THRESHOLD       0.8f     // 70%以下全速
//#define HEAT_CAUTION_THRESHOLD    0.9f    // 70%~85% 减速
//#define HEAT_STOP_THRESHOLD       0.95f     // 90%以上停止

//// 模拟热量专用参数
//#define SIM_HEAT_LIMIT            650.0f   // 热量上限 100
//#define COOLING_PER_SEC           60.0f     // 每秒冷却 5 热量（可实测调整，典型值 5~10）
//#define MAX_BULLETS_PER_SEC       10.0f    // 最大射速（发/秒），需实测校准
//#define FEED_MAX_SPEED            3000.0f  // 拨弹设定值上限

//static float ramp_float(float now, float target, float step)
//{
//    if(now < target)
//    {
//        now += step;
//        if(now > target) now = target;
//    }
//    else if(now > target)
//    {
//        now -= step;
//        if(now < target) now = target;
//    }
//    return now;
//}

//static float calculate_feed_speed(float heat_ratio, float max_speed)
//{
//    if(heat_ratio < HEAT_SAFE_THRESHOLD)
//    {
//        return max_speed;
//    }
//    else if(heat_ratio < HEAT_CAUTION_THRESHOLD)
//    {
//        float ratio = 1.0f - (heat_ratio - HEAT_SAFE_THRESHOLD) / (HEAT_CAUTION_THRESHOLD - HEAT_SAFE_THRESHOLD);
//        return max_speed * (0.5f + ratio * 0.5f);
//    }
//    else if(heat_ratio < HEAT_STOP_THRESHOLD)
//    {
//        return max_speed * 0.3f;
//    }
//    else
//    {
//        return 0.0f;
//    }
//}

//void Shoot_Control(void const * argument)
//{
//    static float simulated_heat = 0.0f;

//    for(;;)
//    {
//        if(vt13_rc.mode_sw == 1)
//        {
//            // 摩擦轮速度调节
//            if( vt13_rc.key_Q   == 1) { shoot -= 100; }
//            if( vt13_rc.key_Q   == 1) { shoot += 100; }
//            if(shoot < 5500) shoot = 5500;
//            if(shoot > 6300) shoot = 6300;
//            if(vt13_rc.key_R	== 1) shoot = 6000;

//            fric_set_now = ramp_float(fric_set_now, (float)shoot, 8.0f);
//            L_setST = -fric_set_now;
//            R_setST = -fric_set_now;

//            /* ---------- 模拟热量更新 ---------- */
//            float bullets_per_sec = 0.0f;
//            if(Feed_set > 0.0f)
//            {
//                bullets_per_sec = (Feed_set / FEED_MAX_SPEED) * MAX_BULLETS_PER_SEC;
//            }

//            // 每毫秒热量增量
//            float heat_inc = bullets_per_sec * HEAT_PER_BULLET * 0.001f;
//            float cooling = COOLING_PER_SEC * 0.001f;

//            simulated_heat += heat_inc;
//            simulated_heat -= cooling;

//            if(simulated_heat < 0.0f) simulated_heat = 0.0f;
//            if(simulated_heat > SIM_HEAT_LIMIT) simulated_heat = SIM_HEAT_LIMIT;

//            float heat_ratio = simulated_heat / SIM_HEAT_LIMIT;
//            /* --------------------------------- */

//            // 根据热量比计算本周期拨弹速度
//            if(vt13_rc.wheel>200||vt13_rc.mouse_left==1)
//            {
//                Feed_set = calculate_feed_speed(heat_ratio, 3000.0f);
//            }
//            else
//            {
//                Feed_set = 0;
//            }
//        }
//        else
//        {
//            fric_set_now = ramp_float(fric_set_now, 0.0f, 15.0f);
//            L_setST = -fric_set_now;
//            R_setST = -fric_set_now;
//            Feed_set = 0;
//        }

//        Firemotor_run(L_setST, R_setST, SHOOT_L.re_speed, SHOOT_R.re_speed, FEED.re_speed, Feed_set);
//        osDelay(1);
//    }
//}




#include "main.h"
#include "rc.h"
#include "uart_bsp.h"
#include "Get_Referee_Data.h"
#include "bits_change.h"
#include "stdbool.h"
float L_setST, R_setST, Feed_set, Feed_ji = 3500;
int shoot = 6000;
extern int zimiao_flag;
float fric_set_now = 0.0f;

 
// 热量控制参数
#define HEAT_PER_BULLET    //热量
#define HEAT_SAFE_THRESHOLD       0.7f    // ：70%以下
#define HEAT_CAUTION_THRESHOLD    0.8f   //70%-8%开始减速
#define HEAT_STOP_THRESHOLD       0.85f    // 90%以上停止射击

// 摩擦轮缓坡函数
static float ramp_float(float now, float target, float step)
{
    if(now < target)
    {
        now += step;
        if(now > target) now = target;
    }
    else if(now > target)
    {
        now -= step;
        if(now < target) now = target;
    }
    return now;
}

// 根据热量计算允许的拨弹速度
static float calculate_feed_speed(float heat_ratio, float max_speed)
{
    if(heat_ratio < HEAT_SAFE_THRESHOLD)
    {
        return max_speed;
    }
    else if(heat_ratio < HEAT_CAUTION_THRESHOLD)
    {
        
        float ratio = 1.0f - (heat_ratio - HEAT_SAFE_THRESHOLD) / (HEAT_CAUTION_THRESHOLD - HEAT_SAFE_THRESHOLD);
        return max_speed * (0.5f + ratio * 0.5f);
    }
    else if(heat_ratio < HEAT_STOP_THRESHOLD)
    {
       
        return max_speed * 0.3f;
    }
    else
    {
       
        return 0.0f;
    }
}
  
//      uint16_t jingji_leve= Referee_System_Info.robot_status.robot_level;
void Shoot_Control(void const * argument)
{
    for(;;)
    {
			    uint16_t current_heat = Referee_System_Info.power_heat_data.shooter_17mm_1_barrel_heat;
          uint16_t heat_limit = Referee_System_Info.robot_status.shooter_barrel_heat_limit;
        if(vt13_rc.mode_sw == 1)
        {
            if(vt13_rc.key_Q == 1) { shoot -= 100; }
            if(vt13_rc.key_E == 1) { shoot += 100; }

            if(shoot < 5500) { shoot = 5500; }
            if(shoot > 6300) { shoot = 6300; }
            if(vt13_rc.key_R == 1) { shoot = 6000; }

            fric_set_now = ramp_float(fric_set_now, (float)shoot, 8.0f);

            L_setST = -fric_set_now;
            R_setST = -fric_set_now;

            
            // 手瞄模式
            if((vt13_rc.wheel > 200 ||vt13_rc.mouse_left == 1))
            {
							
				if(current_heat<heat_limit*0.75){
			 Feed_set=3000;
				}

		if(current_heat>heat_limit*0.75&&current_heat <heat_limit*0.85 )
    {
        Feed_set=1500;
    }
    if(current_heat>heat_limit*0.85&&current_heat < heat_limit*0.92)
    {
        Feed_set=800;
    }
    if(current_heat > heat_limit*0.92)
    {
       
       Feed_set=0;
    }
          
            }
            // 自瞄模式
//            else if((zimiao_flag == 1) && ((vt13_rc.mouse_left == 1 && vt13_rc.mouse_right == 1) || vt13_rc.fn_left == 1))
//            {
//                Feed_set =3000;
//            }
            else
            {
                Feed_set = 0;
            }
        }
        else
        {
            fric_set_now = ramp_float(fric_set_now, 0.0f, 15.0f);

            L_setST = -fric_set_now;
            R_setST = -fric_set_now;
            Feed_set = 0;
        }

        Firemotor_run(L_setST, R_setST, SHOOT_L.re_speed, SHOOT_R.re_speed, FEED.re_speed, Feed_set);

        osDelay(1);
    }
}
