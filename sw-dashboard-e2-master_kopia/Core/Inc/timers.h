/*
 * timers.h
 *
 *  Created on: 21 sie 2022
 *      Author: kpiec
 */

#ifndef __TIMERS_H_
#define __TIMERS_H_

#include "stm32f7xx_hal.h"

#define MAX_NUM_CALLBACKS 2

typedef void(*callback_f)(void);

typedef struct{
	callback_f function;
	TIM_TypeDef* timer;
}callbackInfoStruct_t;

typedef struct{
	callbackInfoStruct_t callbacks[MAX_NUM_CALLBACKS];
	uint8_t numCallbacks;
}callbacksStruct_t;

void TIM_InitCallbackList();
uint8_t TIM_RegisterPeriodCallback(TIM_HandleTypeDef *htim, callback_f callback);
void TIM_HandleCallbacks(TIM_HandleTypeDef *htim);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#endif /* INC_TIMERS_H_ */
