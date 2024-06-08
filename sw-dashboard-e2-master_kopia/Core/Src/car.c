/*
 * car.c
 *
 *  Created on: Sep 10, 2022
 *      Author: kpiec
 */

#include "car.h"

/*
	power*time(s)=energy(J=Ws)
	energy(Wh)=3600*energy(J)
*/

void addEnergySample(energyCalc_t* energyCalcStruct, float power, uint16_t time){
	int16_t E = (power + energyCalcStruct->lastPower) * time / 500.0; // value/2*4/1000
	energyCalcStruct->lastPower = power;

#if ENERGY_CALC_SAMPLES>0
	energyCalcStruct->sum += E - energyCalcStruct->energy[energyCalcStruct->head];
	energyCalcStruct->energy[energyCalcStruct->head++] = E;

	if(energyCalcStruct->head == ENERGY_CALC_SAMPLES)
		energyCalcStruct->head = 0;
#else
	energyCalcStruct->sum += E;
#endif
}

int16_t getEnergy(energyCalc_t* energyCalcStruct){
	return (int16_t)(energyCalcStruct->sum/4/3600);
}
