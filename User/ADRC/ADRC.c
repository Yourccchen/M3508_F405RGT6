//
// Created by DELL on 2023/8/31.
//
#include "ADRC.h"


LADRC_1st *ADRC_Ctrl;
/**
  *@breif      LADRC参数设定
  *@param      LADRC_1st结构体指针，状态反馈带宽wc，状态观测器带宽w0，扰动补偿b0，
  *@retval     none
  */
void setCtrlParm(LADRC_1st *ADRC_Ctrl, double wc, double wo, double b0,double ts)
{
    ADRC_Ctrl->b0 = b0;
    ADRC_Ctrl->wc = wc;
    ADRC_Ctrl->wo = wo;
    ADRC_Ctrl->ts = ts;
//  ADRC_Ctrl->out=0;
//  ADRC_Ctrl->z[0]=0;
//  ADRC_Ctrl->z[1]=0;
}
/**
  *@breif      LADRC输出限幅
  *@param      LADRC_1st结构体指针，输出上限out_up，输出下限out_low
  *@retval     none
  */
void setClampParm(LADRC_1st *ADRC_Ctrl, double out_up, double out_low)
{
    ADRC_Ctrl->out_up = out_up;
    ADRC_Ctrl->out_low = out_low;
}

/**
  *@breif      LADRC输出计算函数
  *@param      LADRC_1st结构体指针，目标值target，反馈值feedback
  *@retval     none
  */
double LADRC_Cal(LADRC_1st *ADRC_Ctrl, double target,double feedback)
{
    // leso
    ADRC_Ctrl->dz[0] = 2*ADRC_Ctrl->wo*(feedback-ADRC_Ctrl->z[0])+ADRC_Ctrl->z[1]+ADRC_Ctrl->b0*ADRC_Ctrl->out;
    ADRC_Ctrl->dz[1] = ADRC_Ctrl->wo*ADRC_Ctrl->wo*(feedback-ADRC_Ctrl->z[0]);

    ADRC_Ctrl->z[0] += ADRC_Ctrl->ts*ADRC_Ctrl->dz[0];
    ADRC_Ctrl->z[1] += ADRC_Ctrl->ts*ADRC_Ctrl->dz[1];

    // controller
    ADRC_Ctrl->out = (ADRC_Ctrl->wc*(target-ADRC_Ctrl->z[0])-ADRC_Ctrl->z[1])/ADRC_Ctrl->b0;

    if (ADRC_Ctrl->out > ADRC_Ctrl->out_up) ADRC_Ctrl->out = ADRC_Ctrl->out_up;
    if (ADRC_Ctrl->out < ADRC_Ctrl->out_low) ADRC_Ctrl->out = ADRC_Ctrl->out_low;

    return  ADRC_Ctrl->out;
}