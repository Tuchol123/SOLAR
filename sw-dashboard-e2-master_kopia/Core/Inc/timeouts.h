/*
 * timeouts.h
 *
 *  Created on: 22 sie 2022
 *      Author: kpiec
 */

#ifndef __TIMEOUTS_H_
#define __TIMEOUTS_H_

typedef struct {
	uint8_t Lights		:1;
	uint8_t Motor		:1;
	uint8_t MotorCtrl	:1;
	uint8_t BMS3		:1;
	uint8_t TPMS		:1;
	uint8_t MotorTemp	:1;
} CanTimeouts_t;

typedef struct {
	uint32_t ErrorCount;
	CanTimeouts_t Timeouts;
	uint8_t ShowError	:1;
	uint8_t ShownErrors;
} CanErrors_t;


#endif /* INC_TIMEOUTS_H_ */
