/********************************************************************
本程序只供学习使用，未经作者许可，不得用于其它任何用途
程序结构参考 安徽师范大学  Lyzhangxiang的EasyHW OS结构设计
datacomm.h
作者：bg8wj
建立日期: 2012.12.23
版本：V1.0

Copyright(C) bg8wj
/********************************************************************/
#ifndef  __DATACOMM_H__
#define  __DATACOMM_H__


/************************************************
PID函数
*************************************************/ 
/*************PID**********************************/
struct PID {
unsigned int Proportion; // 比例常数 Proportional Const
unsigned int Integral; // 积分常数 Integral Const
unsigned int Derivative; // 微分常数 Derivative Const
unsigned int LastError; // Error[-1]
unsigned int PrevError; // Error[-2]
};
xdata struct PID spid; // PID Control Structure

unsigned int rout; // PID Response (Output) 
unsigned int rin; // PID Feedback (Input)
unsigned int temper;
unsigned char shu[3]={0,0,0};
unsigned char counter;
unsigned char set_temper;
bit tp_flag;
unsigned char buff[8];


#endif
