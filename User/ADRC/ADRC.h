//
// Created by DELL on 2023/8/31.
//
#ifndef M3508_F405RGT6_ADRC_H
#define M3508_F405RGT6_ADRC_H

typedef struct {
    double ts;
    double wc;
    double wo;
    double b0;
    double out_up;
    double out_low;
    double out;

    double z[2];
    double dz[2];
}LADRC_1st;

void setCtrlParm(LADRC_1st *ADRC_Ctrl, double wc, double wo, double b0,double ts);
void setClampParm(LADRC_1st *ADRC_Ctrl, double out_up, double out_low);
double LADRC_Cal(LADRC_1st *ADRC_Ctrl, double target,double feedback);

extern LADRC_1st *ADRC_Ctrl;

#endif //M3508_F405RGT6_ADRC_H
