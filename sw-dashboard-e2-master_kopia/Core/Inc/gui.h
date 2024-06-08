/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GUI_H_
#define __GUI_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f7xx_hal.h"
#include "timers.h"
#include "icons.h"
#include <car.h>
#include "../Utilities/BSP/stm32746g_discovery_lcd.h"
#include "../Utilities/BSP/stm32746g_discovery_ts.h"
#include "math.h"

#define POWER_SMOOTHING_GAIN	0.3
#define SEG_LENGTH	60
#define SEG_WIDTH	11
#define DIM_SPEED 0x22FFFFFF
#define SPEED_X	150
#define SPEED_Y	60

#define R_IND_X		365
#define R_IND_Y		40

#define L_IND_X		115
#define L_IND_Y		40

#define MAX_ABS_GRADIENT_POWER	10000
#define POWER_GRADIENT_WIDTH	200
#define POWER_GRADIENT_HEIGHT	12
#define DIFF_SIGN(a,b)	( (a^b) < 0 )

#define LCD_COLOR_LIME	0xFFB5E61D
#define LCD_COLOR_PEACH	0xFFFFA206

typedef enum {UP,DOWN,LEFT,RIGHT} dir_e;
extern LTDC_HandleTypeDef  hLtdcHandler;
extern DMA2D_HandleTypeDef hDma2dHandler;
extern uint32_t ActiveLayer;
extern CanErrors_t GUI_CanErrors;

void GUI_DrawIconA(uint16_t x, uint16_t y, iconA_t* icon, uint32_t colour);

void GUI_Init(uint8_t refreshRate,TIM_HandleTypeDef* htim);
void GUI_Clear();
void GUI_IconsCheck();
void GUI_UpdateIcons();
void GUI_UpdateErrors();
void GUI_UpdateRegen();
void GUI_UpdateGear();
void GUI_UpdateConsumption();
void GUI_ResetErrorCounter();
void GUI_Refresh();

void GUI_UpdateMotorTemperatures();
void GUI_UpdateTirePressure();
void GUI_DisplayTemperature(uint16_t x, uint16_t y,uint16_t Temperature, uint32_t colour);
void GUI_DisplayPressure(uint16_t x, uint16_t y,uint16_t Pressure, uint32_t colour);
void GUI_SpeedScreen();
void GUI_EnergyScreen();
void DrawCurrentScreen();
void HandleButtons();
int ReadKeyPress();
void handleButtonPress(uint8_t button);



#endif
