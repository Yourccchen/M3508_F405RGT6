//
// Created by DELL on 2023/8/29.
//

#ifndef M3508_PID_PID_H
#define M3508_PID_PID_H

/* Includes ------------------------------------------------------------------*/


#include "stdint.h"

typedef enum
{

    PID_Position,
    PID_Speed

}PID_ID;

typedef struct _PID_TypeDef
{
    PID_ID id;

    float target;					    //目标值
    float lastNoneZeroTarget;
    float kp;
    float ki;
    float kd;

    float measure;					    //测量值
    float err;					     	//误差
    float last_err;      		        //上次误差

    float pout;
    float iout;
    float dout;

    float output;						//本次输出
    float last_output;			        //上次输出

    float MaxOutput;				    //输出限幅
    float IntegralLimit;		        //积分限幅
    float DeadBand;			            //死区（绝对值）
    float ControlPeriod;		        //控制周期
    float  Max_Err;					    //最大误差

    uint32_t thistime;
    uint32_t lasttime;
    uint8_t dtime;

    void (*f_param_init)(struct _PID_TypeDef *pid,  //PID参数初始化
                         PID_ID id,
                         uint16_t maxOutput,
                         uint16_t integralLimit,
                         float deadband,
                         uint16_t controlPeriod,
                         int16_t max_err,
                         int16_t  target,
                         float kp,
                         float ki,
                         float kd);

    void (*f_pid_reset_kp)(struct _PID_TypeDef *pid, float kp);		//pid参数修改
    void (*f_pid_reset_ki)(struct _PID_TypeDef *pid, float ki);
    void (*f_pid_reset_kd)(struct _PID_TypeDef *pid, float kd);
    void (*f_pid_reset_target)(struct _PID_TypeDef *pid, float target);
    float (*f_cal_pid)(struct _PID_TypeDef *pid, float measure);                    //pid计算
}PID_TypeDef;

void pid_init(PID_TypeDef* pid);
void Pid_Pos(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void Pid_Speed(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void ADRC_Speed(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void Pid_RM(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);


extern PID_TypeDef pid_pos[4];//位置环
extern PID_TypeDef pid_rpm[4];//速度环


#endif //M3508_PID_PID_H
