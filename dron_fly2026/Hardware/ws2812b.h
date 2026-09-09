#ifndef WS2812B_H
#define WS2812B_H
 
#include "stm32f4xx.h"  
 
#define WS_NUM 20        // ????
#define WS_BIT0 35        // 0?????(?0.48us@72MHz)
#define WS_BIT1 70        // 1?????(?0.97us@72MHz)
 
extern uint32_t ws_rgb_arr[WS_NUM * 24];  // ??GRB?????bit?PWM???
extern TIM_HandleTypeDef htim4;           // 
 
// 
void ws2812b_set_all_color(uint32_t color);
// 
void ws2812b_set_pixel_color(uint8_t index, uint32_t color);
void ws2812b_show(void);
//
void ws2812b_init(void);
 
#endif
