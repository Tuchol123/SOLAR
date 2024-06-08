#ifndef __E2_FRAMES_H_
#define __E2_FRAMES_H_

#define IGNORE_DLC	1

#define NEW_LAP_ID		0x11d


#define LIGHTS_ID		0x302
#define LIGHTS_TIMEOUT	120
#define LIGHTS_LEN		2
typedef struct {
	uint8_t Stop		:1;
	uint8_t LowBeam		:1;
	uint8_t HighBeam	:1;
	uint8_t RightInd	:1;
	uint8_t LeftInd		:1;
	uint8_t ParkLights	:1; //pozycyjne
	uint8_t Hazard		:1; //awaryjne
	uint8_t Horn		:1;
	uint8_t Handbrake	:1;
	uint8_t BattError	:1;
	uint8_t EngError	:1;
	uint8_t	DriveMode	:2;
	uint8_t Interior	:1;
	uint8_t CruiseOn	:1; // 0 - false, 1- true
	uint8_t CruiseActive:1;	// 0 - speedControl, 1- powerControl
} lightsFrame_t;

#define BMS3_ID			0x5C3
#define BMS3_TIMEOUT	1050
#define BMS3_LEN		8
typedef struct {
	uint8_t VoltageHi; //*10
	uint8_t VoltageLo; //*10
	uint8_t CurrentHi; //*10
	uint8_t CurrentLo; //*10
	uint8_t SysState;
	uint8_t SOC;
	uint8_t ChargerEnabled;
} bms3Frame_t;

#define BMS7_ID			0x5C7
#define BMS7_TIMEOUT	550
#define BMS7_LEN		7
typedef struct {
	uint16_t MaxVoltage; //*10
	uint16_t MaxCurrent; //*10
	uint16_t BatteryVoltage; //*10
	uint8_t ChargerEnabled;
} bms7Frame_t;

#define ECCM_ID			0x71
#define ECCM_TIMEOUT	550
#define ECCM_LEN		3
typedef struct {
	uint16_t MaxCurrentAC;
	uint8_t AllowCharging;
} ECCMControlFrame_t;

#define CHARGING_ID 0x70
#define CHARGING_LEN 6
typedef struct {
	uint8_t TYPE2State;
	uint8_t ChargeSequence;
	uint16_t CurrentLimitAC;
	uint8_t Flags;
	uint8_t DriveBlock;
} ECCMStatusFrame_t;

#define MOTOR_ID		0x584
#define MOTOR_TIMEOUT	700
#define MOTOR_LEN		3
typedef struct {
	uint16_t RPM;
	uint8_t temperatureErrors	:4;
	uint8_t temperatureWarnings	:4;
} motorFrame_t;

#define MOTOR_TEMP_ID		0x585
#define MOTOR_TEMP_TIMEOUT	1200
#define MOTOR_TEMP_LEN		8
typedef struct {
	uint16_t rightMotor;
	uint16_t leftMotor;
	uint16_t rightMotorDriver;
	uint16_t leftMotorDriver;
} motorTemperaturesFrame_t;

#define MOTOR_CTRL_ID		0x141
#define MOTOR_CTRL_TIMEOUT	120
#define MOTOR_CTRL_LEN		5
typedef struct {
	uint8_t Throttle;
	uint8_t MainSwitch	:1;
	uint8_t Direction	:1;
	uint8_t ECOPower	:1;
	uint8_t BatEngError	:1;
	uint8_t Fill		:4; //empty
	uint8_t Regen;
	uint8_t CruiseThrottle;
	uint8_t DesiredCruiseSpeed;
} motorControlFrame_t;

#define TPMS_ID			0x7b
#define TPMS_TIMEOUT	1200
#define TPMS_LEN		8
typedef struct {
	uint8_t dummy1;
	uint8_t pressureFL;
	uint8_t dummy2;
	uint8_t pressureFR;
	uint8_t dummy3;
	uint8_t pressureRL;
	uint8_t dummy4;
	uint8_t pressureRR;
} tpmsFrame_t;

//frame in big endian
#define NGL5_CTRL_ID	0x618
#define NGL5_LEN		8
typedef struct {
	uint8_t Control; //0x80 = ON
	uint16_t MaxCurrentAC; //*10
	uint16_t MaxVoltage; //*10
	uint16_t MaxCurrent; //*10
	uint8_t fill;
} __attribute__((packed)) NGL5_ControlFrame_t;

//frame in big endian
#define OVARTECH_CTRL_ID	0x1806E5F4
#define OVARTECH_LEN		8
typedef struct {
	uint16_t MaxVoltage; //*10
	uint16_t MaxCurrent; //*10
	uint8_t Control; 	 //0 = ON
	uint8_t fill[3];
} OVARTECH_ControlFrame_t;


#endif /* INC_E2_FRAMES_H_ */
