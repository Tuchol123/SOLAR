/*
 * timers.c
 *
 *  Created on: 21 sie 2022
 *      Author: kpiec
 */

#include "timers.h"
#include <string.h>

callbacksStruct_t callbacksQ = {};

uint8_t TIM_RegisterPeriodCallback(TIM_HandleTypeDef *htim, callback_f callback){
	if(callbacksQ.numCallbacks < MAX_NUM_CALLBACKS){
		callbacksQ.callbacks[callbacksQ.numCallbacks].timer = htim->Instance;
		callbacksQ.callbacks[callbacksQ.numCallbacks].function = callback;
		callbacksQ.numCallbacks++;
		return 0;
	} else return 1;
}

void TIM_HandleCallbacks(TIM_HandleTypeDef *htim){
	for(uint8_t n=0;n<callbacksQ.numCallbacks;n++){
		if(htim->Instance == callbacksQ.callbacks[n].timer){
			(*callbacksQ.callbacks[n].function)();
			break;
		}
	}
}

void TIM_InitCallbackList(){
	memset(&callbacksQ,0,sizeof(callbacksStruct_t));
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	if (htim->Instance == TIM6) {
		HAL_IncTick();
	}

	TIM_HandleCallbacks(htim);
}

