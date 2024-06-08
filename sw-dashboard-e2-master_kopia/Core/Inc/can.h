/*
 * can.h
 *
 *  Created on: Aug 17, 2022
 *      Author: kpiec
 */

#ifndef __CAN_H_
#define __CAN_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f7xx_hal.h"
#include "timers.h"
#include "car.h"
#include "E2_Frames.h"
#include "gui.h"
#include "main.h"

typedef struct {
	CAN_RxHeaderTypeDef header;
	uint8_t data[8];

} CanRxMessage_t;

typedef struct {
	uint16_t Lights;
	uint16_t Motor;
	uint16_t MotorCtrl;
	uint16_t BMS3;
	uint16_t TPMS;
	uint16_t MotorTemp;
	uint16_t ErrCalback;

} CanTiming_t; // treat as uint64_t and increment all by adding 0x0001000100010001

void CAN_Init(CAN_HandleTypeDef *hcan, TIM_HandleTypeDef *htim);

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan);


#ifdef __cplusplus
}
#endif
#endif /* INC_CAN_H_ */
