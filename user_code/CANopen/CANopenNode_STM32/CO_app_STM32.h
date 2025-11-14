/*
 * CO_app_STM32.h
 *
 *  Created on: Aug 7, 2022
 *      Author: hamed
 */

#ifndef CANOPENSTM32_CO_APP_STM32_H_
#define CANOPENSTM32_CO_APP_STM32_H_

#include "CANopen.h"
#include "main.h"

/* CANHandle : Pass in the CAN Handle to this function and it wil be used for all CAN Communications. It can be FDCan or CAN
 * and CANOpenSTM32 Driver will take of care of handling that
 * HWInitFunction : Pass in the function that initialize the CAN peripheral, usually MX_CAN_Init
 * timerHandle : Pass in the timer that is going to be used for generating 1ms interrupt for tmrThread function,
 * please note that CANOpenSTM32 Library will override HAL_TIM_PeriodElapsedCallback function, if you also need this function
 * in your codes, please take required steps

 */

#ifdef __cplusplus
extern "C" {
#endif


typedef struct {
    uint8_t desiredNodeID;      // 期望分配的CANopen节点ID，调用canopen_app_init后实际分配ID见activeNodeID
    uint8_t activeNodeID;       // 实际分配到的节点ID
    uint16_t baudrate;          // 波特率（CubeMX配置的CAN速率，单位kbit/s）
    TIM_HandleTypeDef* timerHandle; // 用于1ms定时中断的定时器句柄，CANopen库会重载HAL_TIM_PeriodElapsedCallback

#ifdef CO_STM32_FDCAN_Driver
    FDCAN_HandleTypeDef* CANHandle; // FDCAN外设句柄，所有CAN通信使用
#else
    CAN_HandleTypeDef* CANHandle;   // CAN外设句柄，所有CAN通信使用
#endif

    void (*HWInitFunction)();   // CAN外设初始化函数指针，通常为MX_CAN_Init

    uint8_t outStatusLEDGreen;  // 由协议栈更新，用于管理绿色状态指示灯
    uint8_t outStatusLEDRed;    // 由协议栈更新，用于管理红色状态指示灯
    CO_t* canOpenStack;         // 指向CANopen协议栈主结构体

} CANopenNodeSTM32;


// In order to use CANOpenSTM32, you'll have it have a canopenNodeSTM32 structure somewhere in your codes, it is usually residing in CO_app_STM32.c
extern CANopenNodeSTM32* canopenNodeSTM32;


/* This function will initialize the required CANOpen Stack objects, allocate the memory and prepare stack for communication reset*/
int canopen_app_init(CANopenNodeSTM32* canopenSTM32);
/* This function will reset the CAN communication periperhal and also the CANOpen stack variables */
int canopen_app_resetCommunication(void);
/* This function will check the input buffers and any outstanding tasks that are not time critical, this function should be called regurarly from your code (i.e from your while(1))*/
void canopen_app_process(void);
/* Thread function executes in constant intervals, this function can be called from FreeRTOS tasks or Timers ********/
void canopen_app_interrupt(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* CANOPENSTM32_CO_APP_STM32_H_ */
