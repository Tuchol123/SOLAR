/*
 * icons.h
 *
 *  Created on: 24 sie 2022
 *      Author: kpiec
 */

#ifndef __ICONS_H_
#define __ICONS_H_

#include "stm32f7xx_hal.h"

typedef struct{
	uint16_t width;
	uint16_t height;
	uint32_t data[];
}iconARGB_t;

// icons in A4 colour mode (4bits alpha value/px)
// !! x_size divisible by 2 !!
// generate arrays from .bmp files using ikonki/array.py
// works with white backgrounds
typedef struct{
	uint16_t width;
	uint16_t height;
	uint8_t data[];//data size=x_size*y_size/2 bytes
}iconA_t;

extern iconA_t commsIconA;
extern iconA_t cruiseIconA;
extern iconA_t tpmsIconA;

extern iconA_t posLightsIconA;
extern iconA_t lowBeamIconA;

extern iconA_t eagleLogoA;
extern iconA_t eagleTextA;

extern iconA_t EngineTemperatureIconA;
extern iconA_t TireIconA;
extern iconA_t chargingIconA;

#endif /* INC_ICONS_H_ */
