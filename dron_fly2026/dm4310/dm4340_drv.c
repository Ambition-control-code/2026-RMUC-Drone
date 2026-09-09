#include "main.h"

int current;
/**
************************************************************************
* @brief:      	dm4310_enable: ÆôÓÃDM4340µç»ú¿ØÖÆÄ£Ê½º¯Êı
* @param[in]:   hcan:    Ö¸ÏòCAN_HandleTypeDef½á¹¹µÄÖ¸Õë
* @param[in]:   motor:   Ö¸Ïòmotor_t½á¹¹µÄÖ¸Õë£¬°üº¬µç»úÏà¹ØĞÅÏ¢ºÍ¿ØÖÆ²ÎÊı
* @retval:     	void
* @details:    	¸ù¾İµç»ú¿ØÖÆÄ£Ê½ÆôÓÃÏàÓ¦µÄÄ£Ê½£¬Í¨¹ıCAN×ÜÏß·¢ËÍÆôÓÃÃüÁî
*               Ö§³ÖµÄ¿ØÖÆÄ£Ê½°üÀ¨Î»ÖÃÄ£Ê½¡¢Î»ÖÃËÙ¶È¿ØÖÆÄ£Ê½ºÍËÙ¶È¿ØÖÆÄ£Ê½
************************************************************************
**/
void dm4310_enable(CAN_HandleTypeDef* hcan, motor_t *motor)
{
	switch(motor->ctrl.mode)
	{
		case 0:
			enable_motor_mode(hcan, motor->id, MIT_MODE);
			break;
		case 1:
			enable_motor_mode(hcan, motor->id, POS_MODE);
			break;
		case 2:
			enable_motor_mode(hcan, motor->id, SPEED_MODE);
			break;
	}	
}

/**
************************************************************************
* @brief:      	dm4310_ctrl_send: ·¢ËÍDM4310µç»ú¿ØÖÆÃüÁîº¯Êı
* @param[in]:   hcan:    Ö¸ÏòCAN_HandleTypeDef½á¹¹µÄÖ¸Õë
* @param[in]:   motor:   Ö¸Ïòmotor_t½á¹¹µÄÖ¸Õë£¬°üº¬µç»úÏà¹ØĞÅÏ¢ºÍ¿ØÖÆ²ÎÊı
* @retval:     	void
* @details:    	¸ù¾İµç»ú¿ØÖÆÄ£Ê½·¢ËÍÏàÓ¦µÄÃüÁîµ½DM4310µç»ú
*               Ö§³ÖµÄ¿ØÖÆÄ£Ê½°üÀ¨Î»ÖÃÄ£Ê½¡¢Î»ÖÃËÙ¶È¿ØÖÆÄ£Ê½ºÍËÙ¶È¿ØÖÆÄ£Ê½
************************************************************************
**/
void dm4310_ctrl_send(CAN_HandleTypeDef* hcan, motor_t *motor)
{
	switch(motor->ctrl.mode)
	{
		case 0:
			mit_ctrl(hcan, motor->id, motor->ctrl.pos_set, motor->ctrl.vel_set, motor->ctrl.kp_set, motor->ctrl.kd_set, motor->ctrl.tor_set);
			break;
		case 1:
			pos_speed_ctrl(hcan, motor->id, motor->ctrl.pos_set, motor->ctrl.vel_set);
			break;
		case 2:
			speed_ctrl(hcan, motor->id, motor->ctrl.vel_set);
			break;
	}	
}






/**
************************************************************************
* @brief:      	mit_ctrl: MITÄ£Ê½ÏÂµÄµç»ú¿ØÖÆº¯Êı
* @param[in]:   hcan:			Ö¸ÏòCAN_HandleTypeDef½á¹¹µÄÖ¸Õë£¬ÓÃÓÚÖ¸¶¨CAN×ÜÏß
* @param[in]:   motor_id:	µç»úID£¬Ö¸¶¨Ä¿±êµç»ú
* @param[in]:   pos:			Î»ÖÃ¸ø¶¨Öµ
* @param[in]:   vel:			ËÙ¶È¸ø¶¨Öµ
* @param[in]:   kp:				Î»ÖÃ±ÈÀıÏµÊı
* @param[in]:   kd:				Î»ÖÃÎ¢·ÖÏµÊı
* @param[in]:   torq:			×ª¾Ø¸ø¶¨Öµ
* @retval:     	void
* @details:    	Í¨¹ıCAN×ÜÏßÏòµç»ú·¢ËÍMITÄ£Ê½ÏÂµÄ¿ØÖÆÖ¡¡£
************************************************************************
**/
//void mit_ctrl(CAN_HandleTypeDef* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq)
//{
//	uint8_t data[8];
//	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
//	uint16_t id = motor_id + MIT_MODE;

//	pos_tmp = float_to_uint(pos,  P_MIN,  P_MAX,  16);
//	vel_tmp = float_to_uint(vel,  V_MIN,  V_MAX,  12);
//	kp_tmp  = float_to_uint(kp,   KP_MIN, KP_MAX, 12);
//	kd_tmp  = float_to_uint(kd,   KD_MIN, KD_MAX, 12);
//	tor_tmp = float_to_uint(torq, T_MIN,  T_MAX,  12);

//	data[0] = (pos_tmp >> 8);
//	data[1] = pos_tmp;
//	data[2] = (vel_tmp >> 4);
//	data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
//	data[4] = kp_tmp;
//	data[5] = (kd_tmp >> 4);
//	data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
//	data[7] = tor_tmp;
//	
//	canx_send_data(hcan, id, data, 8);
//}

void mit_ctrl(CAN_HandleTypeDef* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq)
{
	uint8_t data[8];
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	uint16_t id = motor_id + MIT_MODE;

	pos_tmp = float_to_uint(pos,  P_MIN,  P_MAX,  16);
	vel_tmp = float_to_uint(vel,  V_MIN,  V_MAX,  12);
	kp_tmp  = float_to_uint(kp,   KP_MIN, KP_MAX, 12);
	kd_tmp  = float_to_uint(kd,   KD_MIN, KD_MAX, 12);
	torq = torq >16384 ? 16384 : torq;
	torq = torq <-16384 ? -16384 : torq;
	torq=torq*(2047.0f/16384.0f);
	//×Ö½Ú2047´ú±í·´×ª£¬×î¶à¸ø0~4095£¬Ó³Éäµ½16384Ö»ÊÇÎªÁËºÍ´ó½®Í¬²½//-712
	tor_tmp = (int)torq+2047;
  current=(int)tor_tmp;
	data[0] = (pos_tmp >> 8);
	data[1] = pos_tmp;
	data[2] = (vel_tmp >> 4);
	data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	data[4] = kp_tmp;
	data[5] = (kd_tmp >> 4);
	data[6] = ((kd_tmp&0xF)<<4)|(current>>8);
	data[7] = current;
	
	canx_send_data(hcan, id, data, 8);
}
/**
************************************************************************
* @brief:      	pos_speed_ctrl: Î»ÖÃËÙ¶È¿ØÖÆº¯Êı
* @param[in]:   hcan:			Ö¸ÏòCAN_HandleTypeDef½á¹¹µÄÖ¸Õë£¬ÓÃÓÚÖ¸¶¨CAN×ÜÏß
* @param[in]:   motor_id:	µç»úID£¬Ö¸¶¨Ä¿±êµç»ú
* @param[in]:   vel:			ËÙ¶È¸ø¶¨Öµ
* @retval:     	void
* @details:    	Í¨¹ıCAN×ÜÏßÏòµç»ú·¢ËÍÎ»ÖÃËÙ¶È¿ØÖÆÃüÁî
************************************************************************
**/
void pos_speed_ctrl(CAN_HandleTypeDef* hcan,uint16_t motor_id, float pos, float vel)
{
	uint16_t id;
	uint8_t *pbuf, *vbuf;
	uint8_t data[8];
	
	id = motor_id + POS_MODE;
	pbuf=(uint8_t*)&pos;
	vbuf=(uint8_t*)&vel;
	
	data[0] = *pbuf;
	data[1] = *(pbuf+1);
	data[2] = *(pbuf+2);
	data[3] = *(pbuf+3);

	data[4] = *vbuf;
	data[5] = *(vbuf+1);
	data[6] = *(vbuf+2);
	data[7] = *(vbuf+3);
	
	canx_send_data(hcan, id, data, 8);
}
/**
************************************************************************
* @brief:      	speed_ctrl: ËÙ¶È¿ØÖÆº¯Êı
* @param[in]:   hcan: 		Ö¸ÏòCAN_HandleTypeDef½á¹¹µÄÖ¸Õë£¬ÓÃÓÚÖ¸¶¨CAN×ÜÏß
* @param[in]:   motor_id: µç»úID£¬Ö¸¶¨Ä¿±êµç»ú
* @param[in]:   vel: 			ËÙ¶È¸ø¶¨Öµ
* @retval:     	void
* @details:    	Í¨¹ıCAN×ÜÏßÏòµç»ú·¢ËÍËÙ¶È¿ØÖÆÃüÁî
************************************************************************
**/
void speed_ctrl(CAN_HandleTypeDef* hcan,uint16_t motor_id, float vel)
{
	uint16_t id;
	uint8_t *vbuf;
	uint8_t data[4];
	
	id = motor_id + SPEED_MODE;
	vbuf=(uint8_t*)&vel;
	
	data[0] = *vbuf;
	data[1] = *(vbuf+1);
	data[2] = *(vbuf+2);
	data[3] = *(vbuf+3);
	
	canx_send_data(hcan, id, data, 4);
}


/**
************************************************************************
* @brief:      	dm4310_fbdata: »ñÈ¡DM4310µç»ú·´À¡Êı¾İº¯Êı
* @param[in]:   motor:    Ö¸Ïòmotor_t½á¹¹µÄÖ¸Õë£¬°üº¬µç»úÏà¹ØĞÅÏ¢ºÍ·´À¡Êı¾İ
* @param[in]:   rx_data:  Ö¸Ïò°üº¬·´À¡Êı¾İµÄÊı×éÖ¸Õë
* @retval:     	void
* @details:    	´Ó½ÓÊÕµ½µÄÊı¾İÖĞÌáÈ¡DM4310µç»úµÄ·´À¡ĞÅÏ¢£¬°üÀ¨µç»úID¡¢
*               ×´Ì¬¡¢Î»ÖÃ¡¢ËÙ¶È¡¢Å¤¾ØÒÔ¼°Ïà¹ØÎÂ¶È²ÎÊı
************************************************************************
**/
void dm4310_fbdata(motor_t *motor, uint8_t *rx_data)
{
	motor->para.id = (rx_data[0])&0x0F;
	motor->para.state = (rx_data[0])>>4;
	motor->para.p_int=(rx_data[1]<<8)|rx_data[2];
	motor->para.v_int=(rx_data[3]<<4)|(rx_data[4]>>4);
	motor->para.t_int=((rx_data[4]&0xF)<<8)|rx_data[5];
	motor->para.pos = uint_to_float(motor->para.p_int, P_MIN, P_MAX, 16); // (-12.5,12.5)
	motor->para.vel = uint_to_float(motor->para.v_int, V_MIN, V_MAX, 12); // (-45.0,45.0)
	motor->para.tor = uint_to_float(motor->para.t_int, T_MIN, T_MAX, 12);  // (-18.0,18.0)
	motor->para.Tmos = (float)(rx_data[6]);
	motor->para.Tcoil = (float)(rx_data[7]);
}


/**
************************************************************************
* @brief:      	float_to_uint: ¸¡µãÊı×ª»»ÎªÎŞ·ûºÅÕûÊıº¯Êı
* @param[in]:   x_float:	´ı×ª»»µÄ¸¡µãÊı
* @param[in]:   x_min:		·¶Î§×îĞ¡Öµ
* @param[in]:   x_max:		·¶Î§×î´óÖµ
* @param[in]:   bits: 		Ä¿±êÎŞ·ûºÅÕûÊıµÄÎ»Êı
* @retval:     	ÎŞ·ûºÅÕûÊı½á¹û
* @details:    	½«¸ø¶¨µÄ¸¡µãÊı x ÔÚÖ¸¶¨·¶Î§ [x_min, x_max] ÄÚ½øĞĞÏßĞÔÓ³Éä£¬Ó³Éä½á¹ûÎªÒ»¸öÖ¸¶¨Î»ÊıµÄÎŞ·ûºÅÕûÊı
************************************************************************
**/
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	/* Converts a float to an unsigned int, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}


/**
************************************************************************
* @brief:      	uint_to_float: ÎŞ·ûºÅÕûÊı×ª»»Îª¸¡µãÊıº¯Êı
* @param[in]:   x_int: ´ı×ª»»µÄÎŞ·ûºÅÕûÊı
* @param[in]:   x_min: ·¶Î§×îĞ¡Öµ
* @param[in]:   x_max: ·¶Î§×î´óÖµ
* @param[in]:   bits:  ÎŞ·ûºÅÕûÊıµÄÎ»Êı
* @retval:     	¸¡µãÊı½á¹û
* @details:    	½«¸ø¶¨µÄÎŞ·ûºÅÕûÊı x_int ÔÚÖ¸¶¨·¶Î§ [x_min, x_max] ÄÚ½øĞĞÏßĞÔÓ³Éä£¬Ó³Éä½á¹ûÎªÒ»¸ö¸¡µãÊı
************************************************************************
**/
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

/**
************************************************************************
* @brief:      	enable_motor_mode: ÆôÓÃµç»úÄ£Ê½º¯Êı
* @param[in]:   hcan:     Ö¸ÏòCAN_HandleTypeDef½á¹¹µÄÖ¸Õë
* @param[in]:   motor_id: µç»úID£¬Ö¸¶¨Ä¿±êµç»ú
* @param[in]:   mode_id:  Ä£Ê½ID£¬Ö¸¶¨Òª¿ªÆôµÄÄ£Ê½
* @retval:     	void
* @details:    	Í¨¹ıCAN×ÜÏßÏòÌØ¶¨µç»ú·¢ËÍÆôÓÃÌØ¶¨Ä£Ê½µÄÃüÁî
************************************************************************
**/
void enable_motor_mode(CAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFC;
	
	canx_send_data(hcan, id, data, 8);
}

/**
************************************************************************
* @brief:      	save_pos_zero: ±£´æÎ»ÖÃÁãµãº¯Êı
* @param[in]:   hcan:     Ö¸ÏòCAN_HandleTypeDef½á¹¹µÄÖ¸Õë
* @param[in]:   motor_id: µç»úID£¬Ö¸¶¨Ä¿±êµç»ú
* @param[in]:   mode_id:  Ä£Ê½ID£¬Ö¸¶¨Òª±£´æÎ»ÖÃÁãµãµÄÄ£Ê½
* @retval:     	void
* @details:    	Í¨¹ıCAN×ÜÏßÏòÌØ¶¨µç»ú·¢ËÍ±£´æÎ»ÖÃÁãµãµÄÃüÁî
************************************************************************
**/
void save_pos_zero(CAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFE;
	
	canx_send_data(hcan, id, data, 8);
}
/**
************************************************************************
* @brief:      	disable_motor_mode: Ê§ÄÜÜ(MIT??)
* @param[in]:   hcan:     CAN??
* @param[in]:   motor_id: ??ID
* @param[in]:   mode_id:  ??ID(???MIT_MODE)
* @retval:     	void
* @details:    	???????,?????????,???????
************************************************************************
**/
void disable_motor_mode(CAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id)
{
    uint8_t data[8];
    uint16_t id = motor_id + mode_id;  // ?? MIT_MODE=0x00,? id = motor_id
    
    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFD;  // ????
    
    canx_send_data(hcan, id, data, 8);
}
//Çå³ı´íÎó





