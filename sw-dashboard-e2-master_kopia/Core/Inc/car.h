/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __car_H
#define __car_H
#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f7xx_hal.h"
#include "E2_Frames.h"
#include "timeouts.h"
#include "can.h"
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

#define PI 						3.14159
//#define WHEEL_DIAMETER 			0.567	//[m]
#define WHEEL_CIRCUMFERENCE		1.78	//[m]

#define MIN_TIRE_PRESSURE_FRONT	34	//2.0bar
#define MIN_TIRE_PRESSURE_REAR	34	//2.0bar

/*
	when number of samples is bigger than 0,
	the energy displayed is a rolling energy consumption
	calculated from ENERGY_CALC_SAMPLES past power frames

	when set to 0, the energy displayed will be the energy
	used since boot of the display.
*/
#define ENERGY_CALC_SAMPLES	0 // approx. 500ms spacing -> (ENERGY_CALC_SAMPLES/2)s


typedef struct {
#if ENERGY_CALC_SAMPLES>0
	int16_t energy[ENERGY_CALC_SAMPLES]; //val/4,Joules; with 500ms frame period max power 16384W
	uint8_t head;
#endif
	float lastPower;
	int32_t sum; //val/4,Joules; max 149kWh
} energyCalc_t;

typedef struct {
	lightsFrame_t Lights;
	CanErrors_t Errors;
	energyCalc_t energyCalc;
	uint8_t Speed;	//kmph
	uint8_t CruiseSpeed;	//kmph
	uint8_t SOC;	//%
	float Power;	//W
	int16_t lastLapEnergy;
	uint8_t MotorsOn		:1;
	uint8_t Regen			:2;
	uint8_t BatteryError	:1;
	uint8_t MotorsWarning	:1;
	uint8_t MotorsError		:1;
	uint8_t TirePressureLow	:1;

	motorTemperaturesFrame_t motorTemp;
	tpmsFrame_t TirePressure;
	uint8_t Charger;
	uint16_t motor1Temperature;
	uint16_t motor2Temperature;
} carInfo_t;


void addEnergySample(energyCalc_t* energyCalcStruct, float power, uint16_t time);
int16_t getEnergy(energyCalc_t* energyCalcStruct);

#ifdef __cplusplus
}
#endif
#endif /*__LST_Structs_H */
