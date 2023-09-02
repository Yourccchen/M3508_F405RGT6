//
// Created by DELL on 2023/8/29.
//
#include "CAN_Recv.h"
#include "main.h"

int k = 3;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

#define ABS(x)	( (x>0) ? (x) : (-x) )

/**
  * @brief          CAN过滤条件初始化
  * @param          none
  * @retval         none
  */
void can_filter_init(void)
{

    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}


//motor data read
// /* “ \ ” 为续行符，表示下面一行是紧接着当前行，\后不可含有任何符号，包括空格和空行，此处必须使用 */
#define get_motor_measure(ptr, data)    	\
    {                                                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
	if(k)															\
	{ (ptr)->offset_angle =(ptr)->last_ecd = (ptr)->ecd;                                   \
	    k--;}														\
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
	if((ptr)->ecd - (ptr)->last_ecd > 4096)									\
		(ptr)->round_cnt --;												\
	else if ((ptr)->ecd - (ptr)->last_ecd < -4096)							\
		(ptr)->round_cnt ++;												\
		  (ptr)->last_ecd = (ptr)->ecd;																	\
	(ptr)->total_angle = (ptr)->round_cnt * 8192 + ((ptr)->ecd - (ptr)->offset_angle);				\
        (ptr)->real_ecd = (ptr)->total_angle * 19.0/ 8192 ;            \
    }

motor_measure_t motor_CAN1[12];
motor_measure_t motor_CAN2[12];

static CAN_TxHeaderTypeDef  gimbal_tx_message;
static uint8_t              gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];


/**
  * @brief          hal库CAN回调函数,接收电机数据
  * @param          hcan:CAN句柄指针
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    if(&hcan1==hcan)
    {
        switch (rx_header.StdId)
        {
            case CAN_3508_M1_ID:
            case CAN_3508_M2_ID:
            case CAN_3508_M3_ID:
            case CAN_3508_M4_ID:
            {
                static uint8_t i = 0;
                //get motor id
                i = rx_header.StdId - CAN_3508_M1_ID;
                get_motor_measure(&motor_CAN1[i], rx_data);
                break;
            }

            default:
            {
                break;
            }
        }
    }
    else if(&hcan2==hcan)
    {
        switch (rx_header.StdId)
        {
            case CAN_3508_M1_ID:
            case CAN_3508_M2_ID:
            case CAN_3508_M3_ID:
            case CAN_3508_M4_ID:
            {
                static uint8_t i = 0;
                //get motor id
                i = rx_header.StdId - CAN_3508_M1_ID;
                get_motor_measure(&motor_CAN2[i], rx_data);
                break;
            }

            default:
            {
                break;
            }
        }

    }
}

/**
  * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
  * @param[in]      none
  * @retval         none
  */
void CAN_cmd_chassis_reset_ID(void)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x700;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = 0;
    chassis_can_send_data[1] = 0;
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}


/**
  * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
void RM_CAN1_Transmit(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_3508_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}


void RM_CAN2_Transmit(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_3508_ALL_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = motor1 >> 8;
    gimbal_can_send_data[1] = motor1;
    gimbal_can_send_data[2] = motor2 >> 8;
    gimbal_can_send_data[3] = motor2;
    gimbal_can_send_data[4] = motor3 >> 8;
    gimbal_can_send_data[5] = motor3;
    gimbal_can_send_data[6] = motor4 >> 8;
    gimbal_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}