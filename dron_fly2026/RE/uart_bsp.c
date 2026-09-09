#include "uart_bsp.h"
#include "rc.h"

extern UART_HandleTypeDef huart6;
#define REFEREE_RXFRAME_LENGTH 256

/**
  * @brief  Starts the multi_buffer DMA Transfer with interrupt enabled.
  * @param  huart      pointer to a UART_HandleTypeDef structure that contains
  *                    the configuration information for the specified USART Stream.  
	*                    ��UART���ָ�룬����UART������Ϣ��
  * @param  SrcAddress pointer to The source memory Buffer address
	*                    ��Դ��ַ��UART���ݼĴ�����ַ����
  * @param  DstAddress pointer to The destination memory Buffer address
	*                    ����һ��Ŀ�껺������ַ��
  * @param  SecondMemAddress pointer to The second memory Buffer address in case of multi buffer Transfer
                             ���ڶ���Ŀ�껺������ַ��  
  * @param  DataLength The length of data to be transferred from source to destination
	                     ��ÿ�������������ݳ��ȣ��ֽ�������
  * @retval none
	* @remember ���µ���������ƺ����봮��ͨѶʹ��DMA�йأ�������ԼĴ�����ֱ��ʹ����ɬ�Ѷ���������ϸѧϰ�������⣬����ǿ��
	* @role     ��δ���ʵ����UART��ͨ���첽�շ�����ʹ��DMA��ֱ���ڴ���ʣ�����˫������յĹ��ܣ���������IDLE�ж�
  */
static void USART_RxDMA_MultiBufferStart(UART_HandleTypeDef *huart, uint32_t *SrcAddress, uint32_t *DstAddress, uint32_t *SecondMemAddress, uint32_t DataLength)
{	
  /* configuare the huart Reception Type TOIDLE */
	//��δ���ʵ����UART��ͨ���첽�շ�����ʹ��DMA��ֱ���ڴ���ʣ�����˫������յĹ��ܣ���������IDLE�ж�
	huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;              //���ý�������

  /* configuare the huart Receicve Size */
	//�����ܽ��ճ���Ϊ����������֮�ͣ�˫������������
   huart->RxXferSize = DataLength * 2;                           //���ý����ܳ���
	
  /* Enable the DMA transfer for the receiver request */
	//��UART���ƼĴ���3(CR3)��ʹ��DMA���չ��ܣ���UART�յ�����ʱ�Զ�����DMA����
  SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);                 //ʹ��DMA��������

  /* Enalbe IDLE interrupt */
	//�������߿����жϣ���UART��·���У������ݣ�ʱ�����ж�
  __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);                     //ʹ��IDLE�ж�

  /* Disable DMA */
	//ѭ������DMAֱ��ȷ��������ֹͣ��ȷ��DMA���ڿ�����״̬��
  do{
      __HAL_DMA_DISABLE(huart->hdmarx);                          //��ȫ����DMA
  }while(huart->hdmarx->Instance->CR & DMA_SxCR_EN);

  /* Configure the source memory Buffer address  */
	//����DMA�����ַ�Ĵ���(PAR)ΪUART���ݼĴ�����ַ��������Դ��
  huart->hdmarx->Instance->PAR = (uint32_t)SrcAddress;           //����DMAԴ��ַ

  /* Configure the destination memory Buffer address */
	//����DMA�ڴ�0��ַ�Ĵ���(M0AR)Ϊ��һ�����ջ�������ַ
  huart->hdmarx->Instance->M0AR = (uint32_t)DstAddress;          //���õ�һ���ڴ滺����

  /* Configure DMA Stream destination address */
	//����DMA�ڴ�1��ַ�Ĵ���(M1AR)Ϊ�ڶ������ջ�������ַ
  huart->hdmarx->Instance->M1AR = (uint32_t)SecondMemAddress;    //���õڶ����ڴ滺����

  /* Configure the length of data to be transferred from source to destination */
	//����DMA���ݴ��������Ĵ���(NDTR)Ϊ��������������
  huart->hdmarx->Instance->NDTR = DataLength;                    //���ô���������

  /* Enable double memory buffer */
	//��DMA���ƼĴ���(CR)��ʹ��˫����ģʽ(Double Buffer Mode)
  SET_BIT(huart->hdmarx->Instance->CR, DMA_SxCR_DBM);            //����˫����ģʽ

  /* Enable DMA */
	//���ʹ��DMA����ʼ���ݴ���
  __HAL_DMA_ENABLE(huart->hdmarx);                               //����DMA����
}

/**
  * @brief  Configures the USART.
  * @role   ����DMA˫�������ĳ�ʼ���������������ʹ������ϸѧϰ��
  * @param  None
  * @retval None
  */
uint8_t Referee_Buf[2][REFEREE_RXFRAME_LENGTH];
uint8_t VT13_Buf[2][21];

void BSP_USART_Init(void)
{
    // USART1 裁判系统
    USART_RxDMA_MultiBufferStart(
        &huart1,
        (uint32_t *)&(huart1.Instance->DR),
        (uint32_t *)Referee_Buf[0],
        (uint32_t *)Referee_Buf[1],
        REFEREE_RXFRAME_LENGTH);

    // USART6 VT13
    USART_RxDMA_MultiBufferStart(
        &huart6,
        (uint32_t *)&(huart6.Instance->DR),
        (uint32_t *)VT13_Buf[0],
        (uint32_t *)VT13_Buf[1],
        136);
}

//------------------------------------------------------------------------------

/**
  * @brief  USER USART6 Reception Event Callback.
  * @param  huart UART handle
  * @param  Size  Number of data available in application reception buffer (indicates a position in
  *               reception buffer until which, data are available)
  * @role   ����USART6�������ݵĻص�����������DMA˫����ģʽʵ�ָ�Ч���ݽ���
  * @teach  USER_USART6_RxHandler ��һ����̬�ص���������DMA�������һ���ȫ�����ʱ�����������ڴ���USART6���յ������ݡ�
            ���Ļ�����DMA˫����ѭ��ģʽ��ͨ�����������������������
  * @retval None
  */
 //VT13_use.use_data;
VT13_USE VT13_use = {0};
static void USER_USART6_RxHandler(UART_HandleTypeDef *huart,uint16_t Size)
{
    if((huart->hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
    {
        __HAL_DMA_DISABLE(huart->hdmarx);

        huart->hdmarx->Instance->CR |= DMA_SxCR_CT;

        if(Size > 10)
        {
						if((VT13_Buf[0][0] == 0xA9 || VT13_Buf[0][1] == 0x53))
						{
							memcpy(VT13_use.temp_data, VT13_Buf[0], 21);
						}
						if((VT13_Buf[0][1] == 0xA9 || VT13_Buf[0][2] == 0x53))
						{
							memcpy(VT13_use.temp_data, VT13_Buf[0], 21);
						}
						deal_vt13();
        }

        __HAL_DMA_SET_COUNTER(huart->hdmarx,272);
    }
    else
    {
        __HAL_DMA_DISABLE(huart->hdmarx);

        huart->hdmarx->Instance->CR &= ~DMA_SxCR_CT;

        if(Size > 10)
        {
						if((VT13_Buf[1][0] == 0xA9 || VT13_Buf[1][1] == 0x53))
						{
							memcpy(VT13_use.temp_data, VT13_Buf[1], 21);
						}
						if((VT13_Buf[1][1] == 0xA9 || VT13_Buf[1][2] == 0x53))
						{
							memcpy(VT13_use.temp_data, VT13_Buf[1], 21);
						}
						deal_vt13();
        }
				

        __HAL_DMA_SET_COUNTER(huart->hdmarx,272);
    }
}
//串口1
static void USER_USART1_RxHandler(UART_HandleTypeDef *huart,uint16_t Size)
{
    if((huart->hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
    {
        __HAL_DMA_DISABLE(huart->hdmarx);

        huart->hdmarx->Instance->CR |= DMA_SxCR_CT;

        if(Size > 5)
        {
            Referee_System_Frame_Update(Referee_Buf[0]);

            __HAL_DMA_SET_COUNTER(
                huart->hdmarx,
                REFEREE_RXFRAME_LENGTH*2);
        }
    }
    else
    {
        __HAL_DMA_DISABLE(huart->hdmarx);

        huart->hdmarx->Instance->CR &= ~DMA_SxCR_CT;

        if(Size > 5)
        {
            Referee_System_Frame_Update(Referee_Buf[1]);
            __HAL_DMA_SET_COUNTER(
                huart->hdmarx,
                REFEREE_RXFRAME_LENGTH*2);
        }
    }
}
/**
  * @brief  Rx Transfer completed callbacks.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
void HAL_UARTEx_RxEventCallback(
    UART_HandleTypeDef *huart,
    uint16_t Size)
{
    if(huart->Instance == USART1)
    {
        USER_USART1_RxHandler(huart, Size);
    }

    if(huart->Instance == USART6)
    {
        USER_USART6_RxHandler(huart, Size);
    }

    huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;

    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

    __HAL_DMA_ENABLE(huart->hdmarx);
}

//------------------------------------------------------------------------------

#include <stdlib.h>  // for abs


VT13_Processed_t vt13_rc = {0};

static int16_t vt13_map_value(uint16_t raw, uint16_t deadzone)
{
    const uint16_t MIN = 364, MID = 1024, MAX = 1684;
    const int16_t RANGE = 660;
    int16_t result;

    if (raw < MIN) raw = MIN;
    if (raw > MAX) raw = MAX;

    if (raw <= MID)
        result = -(int16_t)((MID - raw) * RANGE / (MID - MIN));
    else
        result = (int16_t)((raw - MID) * RANGE / (MAX - MID));

    if (abs(result) < deadzone)
        result = 0;
    return result;
}

/**
 * @brief ?? VT13 ?????(???????)
 */
VT13_data *raw = &VT13_use.use_data;
void deal_vt13(void)
{
 

    vt13_rc.ch0 = vt13_map_value(raw->ch_0, 15);
    vt13_rc.ch1 = vt13_map_value(raw->ch_1, 15);
    vt13_rc.ch2 = vt13_map_value(raw->ch_2, 15);
    vt13_rc.ch3 = vt13_map_value(raw->ch_3, 15);
    vt13_rc.wheel = vt13_map_value(raw->wheel, 10);

    vt13_rc.mode_sw = raw->mode_sw;
    vt13_rc.pause   = raw->pause;
    vt13_rc.fn_left = raw->fn_1;
    vt13_rc.fn_right= raw->fn_2;
    vt13_rc.trigger = raw->trigger;

    // ????
    vt13_rc.mouse_x = raw->mouse_x;
    vt13_rc.mouse_y = raw->mouse_y;
    vt13_rc.mouse_z = raw->mouse_z;
    vt13_rc.mouse_left   = raw->mouse_left;
    vt13_rc.mouse_right  = raw->mouse_right;
    vt13_rc.mouse_middle = raw->mouse_middle;

    // ????(VT13_data ??????,????)
    vt13_rc.key_W     = raw->W;
    vt13_rc.key_S     = raw->S;
    vt13_rc.key_A     = raw->A;
    vt13_rc.key_D     = raw->D;
    vt13_rc.key_Shift = raw->SHIFT;
    vt13_rc.key_Ctrl  = raw->CTRL;
    vt13_rc.key_Q     = raw->Q;
    vt13_rc.key_E     = raw->E;
    vt13_rc.key_R     = raw->R;
    vt13_rc.key_F     = raw->F;
    vt13_rc.key_G     = raw->G;
    vt13_rc.key_Z     = raw->Z;
    vt13_rc.key_X     = raw->X;
    vt13_rc.key_C     = raw->C;
    vt13_rc.key_V     = raw->V;
    vt13_rc.key_B     = raw->B;

    vt13_rc.valid = 1;
}

//------------------------------------------------------------------------------


