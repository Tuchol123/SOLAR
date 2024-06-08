#include "can.h"

extern carInfo_t carInfo;

TIM_HandleTypeDef* htim_CAN;

CanTiming_t timings;
uint32_t lastPowerTime;

void CAN_SetFilters(CAN_HandleTypeDef *hcan) {
    CAN_FilterTypeDef CANFilter;

    // Konfiguracja pierwszego filtra
    CANFilter.FilterMaskIdLow = LIGHTS_ID << 5;
    CANFilter.FilterMaskIdHigh = MOTOR_ID << 5;
    CANFilter.FilterIdLow = MOTOR_CTRL_ID << 5;
    CANFilter.FilterIdHigh = BMS3_ID << 5;
    CANFilter.FilterFIFOAssignment = CAN_RX_FIFO0;
    CANFilter.FilterBank = 0;
    CANFilter.FilterMode = CAN_FILTERMODE_IDLIST;
    CANFilter.FilterScale = CAN_FILTERSCALE_16BIT;
    CANFilter.FilterActivation = ENABLE;
    HAL_CAN_ConfigFilter(hcan, &CANFilter);

    // Konfiguracja drugiego filtra
    CANFilter.FilterMaskIdLow = TPMS_ID << 5;
    CANFilter.FilterMaskIdHigh = NEW_LAP_ID << 5;
    CANFilter.FilterIdLow = 0;
    CANFilter.FilterIdHigh = 0;
    CANFilter.FilterBank = 1;
    HAL_CAN_ConfigFilter(hcan, &CANFilter);

    // Konfiguracja trzeciego filtra dla MOTOR_TEMP_ID
    CANFilter.FilterMaskIdLow = MOTOR_TEMP_ID << 5;
    CANFilter.FilterMaskIdHigh = 0x0000; // Ustawienie maski na 0 pozwala na filtrowanie pojedynczego identyfikatora
    CANFilter.FilterIdLow = 0x0000;
    CANFilter.FilterIdHigh = 0x0000;
    CANFilter.FilterBank = 2; // Następny dostępny bank filtra
    HAL_CAN_ConfigFilter(hcan, &CANFilter);
}

#define TIME_SHOW_ERROR	5000
void CAN_UpdateTimings(){
	//*((uint64_t*)&timings) += 0x0001000100010001;
    carInfo.Errors.Timeouts.Lights = timings.Lights>LIGHTS_TIMEOUT;
    if(!carInfo.Errors.Timeouts.Lights) timings.Lights++;
	else GUI_ResetErrorCounter();

    carInfo.Errors.Timeouts.Motor = timings.Motor>MOTOR_TIMEOUT;
	if(!carInfo.Errors.Timeouts.Motor) timings.Motor++;
	else GUI_ResetErrorCounter();

	carInfo.Errors.Timeouts.MotorCtrl = timings.MotorCtrl>MOTOR_CTRL_TIMEOUT;
	if(!carInfo.Errors.Timeouts.MotorCtrl) timings.MotorCtrl++;
	else GUI_ResetErrorCounter();

	carInfo.Errors.Timeouts.BMS3 = timings.BMS3>BMS3_TIMEOUT;
	if(!carInfo.Errors.Timeouts.BMS3) timings.BMS3++;
	else GUI_ResetErrorCounter();

	carInfo.Errors.Timeouts.TPMS = timings.TPMS>TPMS_TIMEOUT;
	if(!carInfo.Errors.Timeouts.TPMS) timings.TPMS++;
	else GUI_ResetErrorCounter();

	carInfo.Errors.Timeouts.MotorTemp = timings.MotorTemp > MOTOR_TEMP_TIMEOUT;
	if (!carInfo.Errors.Timeouts.MotorTemp) timings.MotorTemp++;
	else GUI_ResetErrorCounter();

	carInfo.Errors.ShownErrors = *((uint8_t*)&carInfo.Errors.Timeouts);
	if(timings.ErrCalback){
		timings.ErrCalback--;
		carInfo.Errors.ShowError = 1;
	} else
		carInfo.Errors.ShowError = carInfo.Errors.ShownErrors>0;


}

void CAN_Init(CAN_HandleTypeDef *hcan, TIM_HandleTypeDef *htim){
	htim_CAN = htim;
	CAN_SetFilters(hcan);
	HAL_CAN_Start(hcan);
	HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	lastPowerTime = HAL_GetTick();
	memset(&timings,0,sizeof(CanTiming_t));
	TIM_RegisterPeriodCallback(htim, CAN_UpdateTimings);
	HAL_TIM_Base_Start_IT(htim);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	CanRxMessage_t RxMessage;
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxMessage.header, RxMessage.data);

	uint32_t id = 0;
	    if (RxMessage.header.IDE == CAN_ID_EXT) {
	        id = RxMessage.header.ExtId;
	    } else {
	        id = RxMessage.header.StdId;
	    }

	switch(id){
		case LIGHTS_ID:
			if(IGNORE_DLC || RxMessage.header.DLC == LIGHTS_LEN){
				timings.Lights = 0;
				memcpy(&carInfo.Lights,RxMessage.data,sizeof(carInfo.Lights));
				GUI_UpdateIcons();
			}
			break;
		case MOTOR_ID:
			if(IGNORE_DLC || RxMessage.header.DLC == MOTOR_LEN){
				timings.Motor = 0;
				motorFrame_t motorFrame;
				memcpy(&motorFrame,RxMessage.data,sizeof(motorFrame_t));
				carInfo.MotorsWarning = motorFrame.temperatureWarnings > 0;
				carInfo.MotorsError = motorFrame.temperatureErrors > 0;
				GUI_UpdateErrors();
#ifdef WHEEL_DIAMETER
				carInfo.Speed = (motorFrame.RPM * PI * WHEEL_DIAMETER) * 6 / 100;
#endif
#ifdef WHEEL_CIRCUMFERENCE
				carInfo.Speed = (motorFrame.RPM * WHEEL_CIRCUMFERENCE) * 6 / 100; //*6*1600RPM/600RPM
#endif
			}
			break;
		case BMS3_ID:
			if(IGNORE_DLC || RxMessage.header.DLC == BMS3_LEN){
				timings.BMS3 = 0;
				uint16_t time = HAL_GetTick() - lastPowerTime;	//get from timer and reset timer
				lastPowerTime = HAL_GetTick();
				bms3Frame_t bms3Frame;
				memcpy(&bms3Frame,RxMessage.data,sizeof(bms3Frame_t));
				carInfo.SOC = bms3Frame.SOC;
				carInfo.Charger = bms3Frame.ChargerEnabled;
				carInfo.Power = (int16_t)(bms3Frame.VoltageLo | bms3Frame.VoltageHi<<8)/10.0;
				carInfo.Power *= (int16_t)(bms3Frame.CurrentLo | bms3Frame.CurrentHi<<8)/10.0;
				addEnergySample(&carInfo.energyCalc,-carInfo.Power,time);
				carInfo.BatteryError = (bms3Frame.SysState & 0xF8) > 0;
				GUI_UpdateErrors();
				GUI_UpdateConsumption();
			}
			break;
		case MOTOR_CTRL_ID:
			if(IGNORE_DLC || RxMessage.header.DLC == MOTOR_CTRL_LEN){
				timings.MotorCtrl = 0;
				motorControlFrame_t motorControlFrame;
				memcpy(&motorControlFrame,RxMessage.data,sizeof(motorControlFrame_t));
				carInfo.CruiseSpeed = motorControlFrame.DesiredCruiseSpeed;
				if(carInfo.MotorsOn != motorControlFrame.MainSwitch){
					carInfo.MotorsOn = motorControlFrame.MainSwitch;
					GUI_UpdateGear();
				}
				carInfo.Regen = motorControlFrame.Regen >> 6; //2 MSB
				GUI_UpdateRegen();
			}
			break;
		case TPMS_ID:
		    if (IGNORE_DLC || RxMessage.header.DLC == TPMS_LEN) {
		        timings.TPMS = 0;
		        tpmsFrame_t tpmsFrame;
		        memcpy(&tpmsFrame, RxMessage.data, sizeof(tpmsFrame));
		        carInfo.TirePressure = tpmsFrame;
		      if((tpmsFrame.pressureFL && (tpmsFrame.pressureFL < MIN_TIRE_PRESSURE_FRONT)) || (tpmsFrame.pressureFR && (tpmsFrame.pressureFR < MIN_TIRE_PRESSURE_FRONT)))
		         carInfo.TirePressureLow = 1;
		      else if((tpmsFrame.pressureRL && (tpmsFrame.pressureRL < MIN_TIRE_PRESSURE_REAR)) || (tpmsFrame.pressureRR && (tpmsFrame.pressureRR < MIN_TIRE_PRESSURE_REAR)))
		         carInfo.TirePressureLow = 1;
		      else
		         carInfo.TirePressureLow = 0;

		    }
		    break;
		case MOTOR_TEMP_ID: //case that read motor temperature CAN data
		            if (IGNORE_DLC || RxMessage.header.DLC == MOTOR_TEMP_LEN) {
		                timings.MotorTemp = 0;
		                motorTemperaturesFrame_t motorTempFrame;
		                memcpy(&motorTempFrame, RxMessage.data, sizeof(motorTempFrame));
		                carInfo.motor1Temperature = motorTempFrame.leftMotor;
		                carInfo.motor2Temperature = motorTempFrame.rightMotor;

		            }
		            break;
		case NEW_LAP_ID:
				carInfo.lastLapEnergy = getEnergy(&carInfo.energyCalc);
				carInfo.energyCalc.sum = 0;
			break;
	}

}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan){
	carInfo.Errors.ErrorCount++;
	timings.ErrCalback = 300;
}
