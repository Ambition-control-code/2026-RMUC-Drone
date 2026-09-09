#ifndef __UART_BSP_H__
#define __UART_BSP_H__

#include "ALL.h"
#include "usart.h" 
#include <stdint.h>
extern uint8_t Referee_flag;

void BSP_USART_Init(void);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size);
void deal_vt13(void);

typedef __packed struct
{
  uint8_t sof_1;								//֡ͷ1
  uint8_t sof_2;								//֡ͷ2
  uint64_t ch_0:11;							//��ҡ��ˮƽ�����λ�á���Сֵ364���м�ֵ1024�����ֵ1684��
  uint64_t ch_1:11;							//��ҡ����ֱ�����λ�á���Сֵ364���м�ֵ1024�����ֵ1684��
  uint64_t ch_2:11;							//��ҡ����ֱ�����λ�á���Сֵ364���м�ֵ1024�����ֵ1684��
  uint64_t ch_3:11;							//��ҡ��ˮƽ�����λ�á���Сֵ364���м�ֵ1024�����ֵ1684��
  uint64_t mode_sw:2;						//��λ�л�����λ�á�C:0	N:1	S:2��
  uint64_t pause:1;							//��ͣ�����Ƿ��¡�δ������0��������1��
  uint64_t fn_1:1;							//����Զ��尴���Ƿ��¡�������1��δ������0��
  uint64_t fn_2:1;							//�Ҳ��Զ��尴���Ƿ��¡�������1��δ������0��
  uint64_t wheel:11;						//���Ͻǲ��֡���Сֵ364���м�ֵ1024�����ֵ1684��
  uint64_t trigger:1;						//���Ͻǰ������������1��δ������0��

  int16_t mouse_x;
  int16_t mouse_y;
  int16_t mouse_z;
  uint8_t mouse_left:2;
  uint8_t mouse_right:2;
  uint8_t mouse_middle:2;
  uint8_t without:2;
	uint16_t W:1;
	uint16_t S:1;
	uint16_t A:1;
	uint16_t D:1;
	uint16_t SHIFT:1;
	uint16_t CTRL:1;
	uint16_t Q:1;
	uint16_t E:1;
	uint16_t R:1;
	uint16_t F:1;
	uint16_t G:1;
	uint16_t Z:1;
	uint16_t X:1;
	uint16_t C:1;
	uint16_t V:1;
	uint16_t B:1;
  uint16_t crc16;
}VT13_data;								
// 解算后的 VT13 遥控器数据（范围已映射、死区已处理）
typedef struct
{
    // 摇杆：范围 [-660, 660]，中位为 0
    int16_t ch0;        // 右摇杆水平（左右）
    int16_t ch1;        // 右摇杆竖直（上下）
    int16_t ch2;        // 左摇杆竖直（上下）
    int16_t ch3;        // 左摇杆水平（左右）
    int16_t wheel;      // 拨轮，范围 [-660, 660]

    // 开关与按钮
    uint8_t mode_sw;    // 0:C, 1:N, 2:S
    uint8_t pause;      // 暂停键
    uint8_t fn_left;    // 自定义左键
    uint8_t fn_right;   // 自定义右键
    uint8_t trigger;    // 扳机

    // 鼠标原始数据（范围 -32768 ~ 32767）
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;

    // 鼠标按键（0=未按下，1=按下）
    uint8_t mouse_left;
    uint8_t mouse_right;
    uint8_t mouse_middle;

    // 键盘按键（布尔值）
    uint8_t key_W, key_S, key_A, key_D;
    uint8_t key_Shift, key_Ctrl;
    uint8_t key_Q, key_E, key_R, key_F, key_G;
    uint8_t key_Z, key_X, key_C, key_V, key_B;

    // 数据有效标志
    uint8_t valid;
} VT13_Processed_t;

// 全局变量，供外部使用
extern VT13_Processed_t vt13_rc;
typedef union
{
	VT13_data use_data;
	uint8_t  temp_data[21];
}VT13_USE;
extern VT13_USE VT13_use;

#endif


