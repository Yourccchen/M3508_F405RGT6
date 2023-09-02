/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "PID.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

uint8_t RxBuffer[1];//串口接收缓冲
uint16_t RxLine = 0;//指令长度
uint8_t DataBuff[200];//指令内容

/* USER CODE END 0 */

UART_HandleTypeDef huart1;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
    HAL_UART_Receive_IT(&huart1,(uint8_t *)RxBuffer,1);//开启串口1中断

  /* USER CODE END USART1_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void usart_printf(const char *fmt,...)
{
    static uint8_t tx_buf[256] = {0};
    static va_list ap;
    static uint16_t len;
    va_start(ap, fmt);
    len = vsprintf((char *)tx_buf, fmt, ap);
    va_end(ap);
    HAL_UART_Transmit(&huart1,tx_buf, len,100);
}

/*
 * 解析出DataBuff的数据
 * 返回解析出的数据
 */
float Get_Data(void)
{
    uint8_t data_Start_Num = 0;  //记录数据位开始的地方
    uint8_t data_End_Num = 0;    //记录数据位结束的地方
    uint8_t data_Num = 0;        // 记录数据位位数
    uint8_t minus_Flag = 0; //判断是不是负数
    float data_return = 0; //解析得到的数据
    for(uint8_t i=0;i<200;i++) //查找等号和感叹号存在的位置
    {
        if(DataBuff[i] == '=') data_Start_Num = i + 1; // +1是直接定位到数据起始位
        if(DataBuff[i] == '!')
        {
            data_End_Num = i - 1;
            break;
        }
    }
    if(DataBuff[data_Start_Num] == '-') //如果是负数
    {
        data_Start_Num += 1; // 后移一位到数据位
        minus_Flag = 1; // 负数flag
    }
    data_Num = data_End_Num - data_Start_Num + 1;
    if(data_Num == 4)//数据共4位
    {
        data_return = (DataBuff[data_Start_Num]-48)  + (DataBuff[data_Start_Num+2]-48)*0.1f +
                      (DataBuff[data_Start_Num+3]-48)*0.01f;
    }
    else if(data_Num == 5)
    {
        data_return = (DataBuff[data_Start_Num]-48)*10 + (DataBuff[data_Start_Num+1]-48) + (DataBuff[data_Start_Num+3]-48)*0.1f +
                      (DataBuff[data_Start_Num+4]-48)*0.01f;
    }
    else if(data_Num == 6)
    {
        data_return = (DataBuff[data_Start_Num]-48)*100 + (DataBuff[data_Start_Num+1]-48)*10 + (DataBuff[data_Start_Num+2]-48) +
                      (DataBuff[data_Start_Num+4]-48)*0.1f + (DataBuff[data_Start_Num+5]-48)*0.01f;
    }
    if(minus_Flag == 1)  data_return = -data_return;
    return data_return;
}

/**
  *@breif      Vofa+参数赋值
  *@param      none
  *@retval     none
  */
void USART_PID_Adjust(uint8_t Motor_n)
{
    float data_Get = Get_Data();
    if(Motor_n == 1)//
    {
        if(DataBuff[0]=='P') // 位置环的P值
        {
            kp_angle=data_Get;
            for(int i = 0;i<4;i++)
            {
                pid_pos[i].kp = kp_angle;
            }
        }
        else if(DataBuff[0]=='I') // 位置环的I值
        {
            ki_angle=data_Get;
            for(int i = 0;i<4;i++)
            {
                pid_pos[i].ki = ki_angle;
            }
        }
        else if(DataBuff[0]=='D') // 位置环的D值
        {
            kd_angle=data_Get;
            for(int i = 0;i<4;i++)
            {
                pid_pos[i].kd = kd_angle;
            }
        }
        else if(DataBuff[0]=='T') // 角度目标值
        {
            target_pos=data_Get;
            for(int i = 0;i<4;i++)
            {
                pid_pos[i].target = target_pos;
            }
        }

        if(DataBuff[0]=='p') // 速度环的P值
        {
            kp_speed=data_Get;
            for(int i = 0;i<4;i++)
            {
                pid_rpm[i].kp = kp_speed;
            }
        }
        else if(DataBuff[0]=='i') // 速度环的I值
        {
            ki_speed=data_Get;
            for(int i = 0;i<4;i++)
            {
                pid_rpm[i].ki = ki_speed;
            }
        }
        else if(DataBuff[0]=='d') // 速度环的D值
        {
            kd_speed=data_Get;
            for(int i = 0;i<4;i++)
            {
                pid_rpm[i].kd = kd_speed;
            }
        }
        else if(DataBuff[0]=='t') // 速度目标值
        {
            target_spd=data_Get;
            for(int i = 0;i<4;i++)
            {
                pid_rpm[i].target = target_spd;
            }
        }
    }
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
    if(UartHandle->Instance==USART1)//如果是串口1
    {
        RxLine++;                      //每接收一个数据，进入回调数据长度+1
        DataBuff[RxLine-1]=RxBuffer[0];  //将接收到的数据保存到缓存数组
        if(RxBuffer[0]==0x21)            //接收结束标志位，可自行设置(!的十六进制为0x21)
        {
            for(int i=0;i<RxLine;i++)

                USART_PID_Adjust(1);//参数赋值


            memset(DataBuff,0,sizeof(DataBuff));  //清空缓存数组
            RxLine=0;  //清空接收长度
        }
        RxBuffer[0]=0;
        HAL_UART_Receive_IT(&huart1, (uint8_t *)RxBuffer, 1); //每接收一个数据就打开一次中断
    }
}
/* USER CODE END 1 */
