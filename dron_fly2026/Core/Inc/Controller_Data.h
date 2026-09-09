#ifndef __CONTROLLER_DATA_H__
#define __CONTROLLER_DATA_H__

#include "stdint.h"
#include "stdbool.h"

// Custom controller mistake max count
#define Controller_mistake_MAX 50

// Custom controller trigger front value
#define Trigger_Front_Value 1

// Custom robot data structure
typedef struct
{
    uint8_t data[30];
} custom_robot_data_t1;

// Custom data union for controller communication
typedef union
{
    struct
    {
        uint8_t Trigger_Front;
        uint8_t reserved[29];
    } Custom_Data;
    uint8_t RX_custom_data[30];
} custom_data_union;

extern custom_data_union custom_data;

#endif /* __CONTROLLER_DATA_H__ */
