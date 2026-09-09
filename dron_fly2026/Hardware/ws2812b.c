#include "ws2812b.h"
#include "main.h"
uint32_t ws_rgb_arr[WS_NUM * 24] = {0};  // 全局数组:储存每个bit的PWM占空比
 
// RGB转GRB(WS2812B需要GRB??)
static uint32_t rgb_to_grb(uint8_t r, uint8_t g, uint8_t b)
{
    return (g << 16) | (r << 8) | b;
}
 
// 填充单个灯珠的24位数据到数组
static void set_pixel(uint8_t index, uint32_t grb)
{
    if (index >= WS_NUM) return;
    for (uint8_t i = 0; i < 24; i++)
    {
        // 提取每个bit,1——WS_BIT1,0——WS_BIT0
        ws_rgb_arr[index*24 + i] = ((grb << i) & 0x800000) ? WS_BIT1 : WS_BIT0;
    }
}
 
//// ??????:c1=第一个灯的RGB?,c2=第二个灯的RGB?
//void ws2812b_show_color(uint32_t c1, uint32_t c2)
//{
//    // 解析RGB格式(c1:0xRRGGBB)
//    uint8_t r1 = (c1 >> 16) & 0xFF;  // ????
//    uint8_t g1 = (c1 >> 8) & 0xFF;   // ????
//    uint8_t b1 = c1 & 0xFF;          // ????
//    
//    uint8_t r2 = (c2 >> 16) & 0xFF;
//    uint8_t g2 = (c2 >> 8) & 0xFF;
//    uint8_t b2 = c2 & 0xFF;
//    
//    // 转为GRB并填充数组
//    set_pixel(0, rgb_to_grb(r1, g1, b1));
//    set_pixel(1, rgb_to_grb(r2, g2, b2));
//    
//    // 启动DMA，刷新显示
//    HAL_TIM_PWM_Start_DMA(&htim4, TIM_CHANNEL_3, (uint32_t*)ws_rgb_arr, WS_NUM*24);
//}
//设置单个灯珠颜色
void ws2812b_set_pixel_color(uint8_t index, uint32_t color)
{
    if (index >= WS_NUM) return;
    
    uint8_t r = (color >> 16) & 0xFF;  // ??
    uint8_t g = (color >> 8) & 0xFF;   // ??
    uint8_t b = color & 0xFF;          // ??
    
    set_pixel(index, rgb_to_grb(r, g, b));
}
//所有灯珠颜色都变
void ws2812b_set_all_color(uint32_t color)
{
    uint8_t r = (color >> 16) & 0xFF;
    uint8_t g = (color >> 8) & 0xFF;
    uint8_t b = color & 0xFF;
    uint32_t grb = rgb_to_grb(r, g, b);
    
    for (uint8_t i = 0; i < WS_NUM; i++)
    {
        set_pixel(i, grb);
    }
}

void ws2812b_show(void)
{
    HAL_TIM_PWM_Stop_DMA(&htim4, TIM_CHANNEL_3);
    // ????DMA??
    HAL_TIM_PWM_Start_DMA(&htim4, TIM_CHANNEL_3, (uint32_t*)ws_rgb_arr, WS_NUM * 24);
}

 
void ws2812b_init(void)
{
    for (uint16_t i = 0; i < WS_NUM * 24; i++)
    {
        ws_rgb_arr[i] = 0;
    }
    ws2812b_set_all_color(0x0000FF);  // ??: 0x0000FF
    ws2812b_show();
}
// DMA   PWM()
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim4)
    {
        HAL_TIM_PWM_Stop_DMA(&htim4, TIM_CHANNEL_3);
    }
}
