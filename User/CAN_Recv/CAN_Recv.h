//
// Created by DELL on 2023/8/29.
//

#ifndef M3508_PID_CAN_RECV_H
#define M3508_PID_CAN_RECV_H


#include <stdint-gcc.h>

#define CHASSIS_CAN hcan1
#define GIMBAL_CAN hcan2

/* CAN send and receive ID */
typedef enum
{
    CAN_3508_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,
} can_msg_id_e;

//rm motor data
typedef struct
{

    uint16_t    ecd;				//电机转子角度
    int         real_ecd;			//电机转子绝对角度（360）
    int16_t     speed_rpm;			//转速
    int16_t     given_current;		//给定电流
    uint8_t     temperate;			//温度
    int16_t     last_ecd;			//转子上一瞬角度
    int16_t	    offset_angle;	    //补偿角度
    int32_t		round_cnt;		    //累计圈数
    int32_t		total_angle;	    //累计角度

} motor_measure_t;


extern void CAN_cmd_chassis_reset_ID(void);

extern void RM_CAN1_Transmit(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern void RM_CAN2_Transmit(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

extern motor_measure_t  motor_CAN1[12];
extern motor_measure_t  motor_CAN2[12];


extern void can_filter_init(void);



#endif //M3508_PID_CAN_RECV_H
