#include "gui.h"
#include <stdbool.h>


extern carInfo_t carInfo;

TIM_HandleTypeDef* htim_GUI;

lightsFrame_t prevLights;

uint8_t frameCount,frameRate,canErrorFrames;
float shownPower;

// draw a generated icon at (x,y) with specified colour
void GUI_DrawIconA(uint16_t x, uint16_t y, iconA_t* icon, uint32_t colour){
	uint32_t address;
	/* Set the address */
	address = hLtdcHandler.LayerCfg[ActiveLayer].FBStartAdress + (((BSP_LCD_GetXSize()*y) + x)*(4));

	uint8_t* pbmp = (uint8_t*)&(icon->data);

	hDma2dHandler.Init.Mode         = DMA2D_M2M_PFC;
	hDma2dHandler.Init.ColorMode    = DMA2D_ARGB8888;
	hDma2dHandler.Init.OutputOffset = (BSP_LCD_GetXSize() - icon->width);

	hDma2dHandler.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
	hDma2dHandler.LayerCfg[1].InputAlpha = colour;
	hDma2dHandler.LayerCfg[1].InputColorMode = DMA2D_INPUT_A4;
	hDma2dHandler.LayerCfg[1].InputOffset = 0;

	hDma2dHandler.Instance = DMA2D;

	if(HAL_DMA2D_Init(&hDma2dHandler) == HAL_OK){
		if(HAL_DMA2D_ConfigLayer(&hDma2dHandler, 1) == HAL_OK){
			if (HAL_DMA2D_Start(&hDma2dHandler, (uint32_t)pbmp, (uint32_t)address, icon->width, icon->height) == HAL_OK){
				/* Polling For DMA transfer */
				HAL_DMA2D_PollForTransfer(&hDma2dHandler, 10);
			}
		}
	}

}

void GUI_DrawRectGradient(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint32_t colour, dir_e dir){

	uint32_t address = hLtdcHandler.LayerCfg[ActiveLayer].FBStartAdress + (((BSP_LCD_GetXSize()*y) + x)*(4));
	uint32_t mask;
	colour &= 0xFFFFFF;
	hDma2dHandler.Init.Mode 		= DMA2D_R2M;
    hDma2dHandler.Init.ColorMode 	= DMA2D_ARGB8888;
    hDma2dHandler.Init.OutputOffset = BSP_LCD_GetXSize()-width;

    hDma2dHandler.Instance = DMA2D;
    if(HAL_DMA2D_Init(&hDma2dHandler) == HAL_OK){
		if(HAL_DMA2D_ConfigLayer(&hDma2dHandler, 1) == HAL_OK){
			switch(dir){
			case UP:
				for(uint16_t h = 0;h < height; h++){
                    mask = (0xFF * h / (height-1)) << 24;
//					mask = (0x11|(0xEE * h / (height-1))) << 24;
					if (HAL_DMA2D_Start(&hDma2dHandler, colour|mask, address, width, 1) == HAL_OK){
				        HAL_DMA2D_PollForTransfer(&hDma2dHandler, 10);
					}
					address += BSP_LCD_GetXSize()*4;
				}
				break;
			case DOWN:
				for(uint16_t h = 0;h < height; h++){
					mask = (0xFF * (height-1-h) / (height-1)) << 24;
//					mask = (0x11|(0xEE * (height-1-h) / (height-1))) << 24;
					if (HAL_DMA2D_Start(&hDma2dHandler, colour|mask, address, width, 1) == HAL_OK){
				        HAL_DMA2D_PollForTransfer(&hDma2dHandler, 10);
					}
					address += BSP_LCD_GetXSize()*4;
				}
				break;
			default:
				//only up/down supported
				break;
			}
		}
    }
}

void GUI_DrawTriagGradient(uint16_t x, uint16_t y, uint16_t size, uint32_t colour, dir_e dir){
	uint32_t mask;
	colour &= 0xFFFFFF;
	size--;
	for(uint16_t a = 0;a <= size; a++){
		mask = (0xFF * (size-a) / (size)) << 24;
		for(uint16_t b = 0; b <= a ; b++){
			if(dir == LEFT)
				BSP_LCD_DrawPixel(x+b, y-a+b, colour|mask);
			else if(dir == RIGHT)
				BSP_LCD_DrawPixel(x-b, y-a+b, colour|mask);
			else
				break;
		}
	}
}

void LCD_Config(void){
	/* LCD Initialization */
	BSP_LCD_Init();

	/* LCD Initialization */
	BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS);

	/* Enable the LCD */
	BSP_LCD_DisplayOn();

	/* Select the LCD Background Layer  */
	BSP_LCD_SelectLayer(0);

	/* Clear the Background Layer */
	BSP_LCD_Clear(LCD_COLOR_TRANSPARENT);

	/* Configure the transparency for background */
	BSP_LCD_SetTransparency(0, 255);
}

//setup timer to trigger at specified frequency
void GUI_TimerConfig(uint16_t timerFreq){
	uint32_t reload = timerFreq/frameRate-1;
	htim_GUI->Instance->ARR = reload;
	HAL_TIM_Base_Start_IT(htim_GUI);
}

void GUI_Clear(void){
	BSP_LCD_Clear(LCD_COLOR_TRANSPARENT);
	BSP_LCD_SetBackColor(LCD_COLOR_TRANSPARENT);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_SetFont(&Font24);
}

void GUI_Init(uint8_t refreshRate,TIM_HandleTypeDef* htim){
	htim_GUI = htim;
	frameRate = refreshRate;
	LCD_Config();

	// make the screen look fancy with the loading screen
	GUI_DrawIconA(122, 10, &eagleLogoA, 0xFFC00000);
	GUI_DrawIconA(20, 225, &eagleTextA, LCD_COLOR_WHITE);
	HAL_Delay(1000);

	// display all icons
	GUI_Clear();
	GUI_IconsCheck();
	HAL_Delay(500);
	GUI_Clear();

	// setup the timer
	GUI_TimerConfig(10000);
	// add render callback to the timer
	TIM_RegisterPeriodCallback(htim, GUI_Refresh);
	*(uint16_t*)&prevLights = ~(*(uint16_t*)&carInfo.Lights);
	GUI_UpdateIcons();
	GUI_UpdateRegen();
	GUI_UpdateGear();
}

void GUI_DrawBattery(uint16_t x, uint16_t y, uint8_t procent){
	char text[8];
	uint32_t colour = (procent>20 ? LCD_COLOR_GREEN : (procent>5 ? LCD_COLOR_ORANGE : LCD_COLOR_RED));
	uint16_t notFill = (100-procent)*3/5;

	BSP_LCD_SetTextColor(colour);
	BSP_LCD_DrawRect(x, y+9, 32, 62);
	BSP_LCD_FillRect(x+11, y, 10, 10);
	BSP_LCD_FillRect(x+1, y+10+notFill, 31, 61-notFill);
	sprintf(text, "%d%%", procent);
	BSP_LCD_DisplayStringAt(x+35, y+35, (uint8_t*)text, LEFT_MODE);

	BSP_LCD_SetTextColor(LCD_COLOR_TRANSPARENT);
	BSP_LCD_FillRect(x+1, y+10, 31, notFill);
}

void ecoGear(int x, int y){
	BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
	BSP_LCD_FillRect(x, y, 6, 50);
	BSP_LCD_FillRect(x, y, 25, 6);
	BSP_LCD_FillRect(x, y + 20, 25, 6);
	BSP_LCD_FillRect(x, y + 44, 25, 6);
}

void parkingGear(int x, int y){
	BSP_LCD_SetTextColor(LCD_COLOR_RED);
	BSP_LCD_FillRect(x, y, 6, 50);
	BSP_LCD_FillRect(x, y, 25, 6);
	BSP_LCD_FillRect(x + 23, y + 3, 6, 20);
	BSP_LCD_FillRect(x, y + 20, 25, 6);
}

void driveGear(int x, int y){
	BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
	BSP_LCD_FillRect(x, y, 6, 50);
	BSP_LCD_FillRect(x, y, 25, 6);
	BSP_LCD_FillRect(x + 23, y + 3, 6, 45);
	BSP_LCD_FillRect(x, y + 44, 25, 6);
}

void neutralGear(int x, int y){
	BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
	BSP_LCD_FillRect(x, y, 6, 50);
	BSP_LCD_FillRect(x + 20, y, 6, 50);
	BSP_LCD_DrawLine(x, y, x+20, y+50);
	BSP_LCD_DrawLine(x+1, y, x+20, y+50);
	BSP_LCD_DrawLine(x+2, y, x+21, y+50);
	BSP_LCD_DrawLine(x+3, y, x+22, y+50);
	BSP_LCD_DrawLine(x+4, y, x+23, y+50);
	BSP_LCD_DrawLine(x+5, y, x+24, y+50);
	BSP_LCD_DrawLine(x+6, y, x+25, y+50);
}

void reverseGear(int x, int y){
	BSP_LCD_SetTextColor(LCD_COLOR_ORANGE);
	BSP_LCD_FillRect(x, y, 6, 50);
	BSP_LCD_FillRect(x, y, 25, 6);
	BSP_LCD_FillRect(x + 23, y + 3, 6, 20);
	BSP_LCD_FillRect(x, y + 20, 25, 6);
	BSP_LCD_DrawLine(x + 10, y + 26, x + 29, y + 49);
	BSP_LCD_DrawLine(x + 10, y + 25, x + 29, y + 48);
	BSP_LCD_DrawLine(x + 10, y + 24, x + 29, y + 47);
	BSP_LCD_DrawLine(x + 10, y + 23, x + 29, y + 46);
	BSP_LCD_DrawLine(x + 9, y + 26, x + 28, y + 49);
	BSP_LCD_DrawLine(x + 8, y + 26, x + 27, y + 49);
	BSP_LCD_DrawLine(x + 7, y + 26, x + 26, y + 49);
	BSP_LCD_DrawLine(x + 6, y + 26, x + 25, y + 49);
}

void battError(int kolor){
	int x = 200, y = 27;
	BSP_LCD_SetTextColor(kolor);
	BSP_LCD_DrawRect(x, y, 35, 20);
	BSP_LCD_DrawRect(x + 1, y + 1, 35 - 2, 20 - 2);
	BSP_LCD_DrawRect(x + 2, y + 2, 35 - 4, 20 - 4);
	BSP_LCD_FillRect(x + 6, y - 3, 4, 3);
	BSP_LCD_FillRect(x + 25, y - 3, 4, 3);
	BSP_LCD_FillRect(x + 6, y + 6, 4, 2);
	BSP_LCD_FillRect(x + 24, y + 6, 6, 2);
	BSP_LCD_FillRect(x + 26, y + 4, 2, 6);

}

void engError(int kolor){
	int x = 260, y = 25;
	BSP_LCD_SetTextColor(kolor);
	BSP_LCD_DrawRect(x + 5, y, 33, 22);
	BSP_LCD_DrawRect(x + 6, y + 1, 35 - 2 - 2, 22 - 2);
	BSP_LCD_DrawRect(x + 7, y + 2, 35 - 4 - 2, 22 - 4);
	BSP_LCD_DrawLine(x + 14, y + 7, x + 29, y + 7);
	BSP_LCD_DrawLine(x + 14, y + 11, x + 29, y + 11);
	BSP_LCD_DrawLine(x + 14, y + 15, x + 29, y + 15);
	BSP_LCD_FillRect(x + 2, y + 9, 4, 3);
	BSP_LCD_FillRect(x, y + 4, 2, 14);
}

// draws communication icon with specified colour
void GUI_DrawComm(uint32_t colour){
	GUI_DrawIconA(425, 95, &commsIconA, colour);
}

// draws cruise control icon and set speed with specified colour
void GUI_DrawCruise(uint32_t colour){
	GUI_DrawIconA(380, 85, &cruiseIconA, colour);
	char text[8];
	BSP_LCD_SetFont(&Font16);
	BSP_LCD_SetTextColor(colour);
	sprintf(text, "%02d", carInfo.CruiseSpeed);
	BSP_LCD_DisplayStringAt(395, 115, (uint8_t*)text, LEFT_MODE);
	BSP_LCD_SetFont(&Font24);
}

// draws TPMS icon with specified colour
void GUI_DrawTPMS(uint32_t colour){
	GUI_DrawIconA(425, 55, &tpmsIconA, colour);
}

// calculate power value to be shown based on displayed and current values
float GUI_SmoothPower(float carPower){
	return shownPower + POWER_SMOOTHING_GAIN * (carPower - shownPower);
}

void GUI_DisplayPower(float Power, uint32_t colour){
	char text[16];
	BSP_LCD_SetTextColor(colour);
	sprintf(text, " %4.2f kW ", Power/1000.0);
	BSP_LCD_DisplayStringAt(0, 230, (uint8_t*)text, CENTER_MODE);
}

static int16_t last_draw_width;
void GUI_UpdatePowerGradient(float Power){
	int16_t draw_width = (int16_t)(Power/MAX_ABS_GRADIENT_POWER*POWER_GRADIENT_WIDTH);
	uint16_t start = BSP_LCD_GetXSize() / 2;
	if(draw_width != last_draw_width){
		int16_t diff = draw_width-last_draw_width;
		if((diff>0)&&(last_draw_width<=0)){
		    BSP_LCD_FillRectColour(start+last_draw_width-POWER_GRADIENT_HEIGHT, BSP_LCD_GetYSize()-POWER_GRADIENT_HEIGHT, abs(diff)+2*POWER_GRADIENT_HEIGHT, POWER_GRADIENT_HEIGHT, LCD_COLOR_TRANSPARENT);
		} else if((diff<0)&&(last_draw_width>=0)){
		    BSP_LCD_FillRectColour(start+draw_width-POWER_GRADIENT_HEIGHT, BSP_LCD_GetYSize()-POWER_GRADIENT_HEIGHT, abs(diff)+2*POWER_GRADIENT_HEIGHT, POWER_GRADIENT_HEIGHT, LCD_COLOR_TRANSPARENT);
		}
		if(draw_width < 0){
			GUI_DrawRectGradient(start+draw_width, BSP_LCD_GetYSize()-POWER_GRADIENT_HEIGHT, -draw_width, POWER_GRADIENT_HEIGHT, LCD_COLOR_GREEN, UP);
			GUI_DrawTriagGradient(start+draw_width, BSP_LCD_GetYSize(), POWER_GRADIENT_HEIGHT, LCD_COLOR_GREEN, RIGHT);
		} else{
	        GUI_DrawRectGradient(start, BSP_LCD_GetYSize()-POWER_GRADIENT_HEIGHT, draw_width, POWER_GRADIENT_HEIGHT, LCD_COLOR_BLUE, UP);
	        GUI_DrawTriagGradient(start+draw_width, BSP_LCD_GetYSize(), POWER_GRADIENT_HEIGHT, LCD_COLOR_BLUE, LEFT);
		}
	}
	last_draw_width = draw_width;
}

// draw energy counter
void GUI_DisplayEnergy(uint16_t x, uint16_t y, int16_t pastEnergy, int16_t Energy){
	char text[15];
	BSP_LCD_SetFont(&Font20);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	sprintf(text, "%dWh  ", pastEnergy);
	BSP_LCD_DisplayStringAt(x, y, (uint8_t*)text, LEFT_MODE);
	sprintf(text, "  %dWh", Energy);
	BSP_LCD_SetTextColor(Energy>pastEnergy?LCD_COLOR_RED:LCD_COLOR_GREEN);
	BSP_LCD_DisplayStringAt(x, y, (uint8_t*)text, RIGHT_MODE);
	BSP_LCD_SetFont(&Font24);
}

void GUI_UpdateConsumption(){
	GUI_DisplayEnergy(20, 230, carInfo.lastLapEnergy, getEnergy(&carInfo.energyCalc));
}

void GUI_DrawRegenBars(uint16_t x, uint16_t y, uint8_t val, uint32_t colour){// +6/+4
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_DrawRect(x, y, 24, 45);

	BSP_LCD_SetTextColor(colour);

	BSP_LCD_FillRectColour(x+3, y+3, 19, 12, val>2?colour:LCD_COLOR_TRANSPARENT);
	BSP_LCD_FillRectColour(x+3, y+17, 19, 12, val>1?colour:LCD_COLOR_TRANSPARENT);
	BSP_LCD_FillRectColour(x+3, y+31, 19, 12, val>0?colour:LCD_COLOR_TRANSPARENT);
}

void GUI_UpdateRegen(){
	GUI_DrawRegenBars(425, 153, carInfo.Regen, LCD_COLOR_LIME);
	//GUI_DrawIconA(423, 154, &regenIconA, LCD_COLOR_GREEN);
}

void GUI_PositionLights(uint32_t colour){
	GUI_DrawIconA(30, 25, &posLightsIconA, colour);
}

void GUI_DrawLowBeam(uint32_t colour){
	GUI_DrawIconA(30, 65, &lowBeamIconA, colour);
}

void rightIndicator(uint32_t colour){
	//do poprawy
	BSP_LCD_SetTextColor(colour);
	for (int i = 0; i < 18; i++){
		for (int j = 0; j < i; j++){
			BSP_LCD_DrawPixel(R_IND_X - i, R_IND_Y + j, colour);
			BSP_LCD_DrawPixel(R_IND_X - i, R_IND_Y - j, colour);
		}
	}
	BSP_LCD_FillRect(R_IND_X - 27, R_IND_Y - 8, 10, 19);
}

void leftIndicator(uint32_t colour){
	// do poprawy
	BSP_LCD_SetTextColor(colour);
	for (int i = 0; i < 18; i++){
		for (int j = 0; j < i; j++){
			BSP_LCD_DrawPixel(L_IND_X + i, L_IND_Y + j, colour);
			BSP_LCD_DrawPixel(L_IND_X + i, L_IND_Y - j, colour);
		}
	}
	BSP_LCD_FillRect(L_IND_X + 18, L_IND_Y - 8, 10, 19);
}

// helper function drawing a vertical segment
void V_seg(uint16_t x, uint16_t y, uint32_t colour){
	BSP_LCD_SetTextColor(colour);

	BSP_LCD_DrawVLine(x, y, SEG_LENGTH);
	for(uint8_t n=1;n<SEG_WIDTH/2;n++){
		BSP_LCD_DrawVLine(x + n, y + n, SEG_LENGTH-2*n);
		BSP_LCD_DrawVLine(x - n, y + n, SEG_LENGTH-2*n);
	}
}

// helper function drawing a horizontal segment
void H_seg(uint16_t x, uint16_t y, uint32_t colour){
	BSP_LCD_SetTextColor(colour);

	BSP_LCD_DrawHLine(x, y, SEG_LENGTH);
	for(uint8_t n=1;n<SEG_WIDTH/2;n++){
		BSP_LCD_DrawHLine(x + n, y + n, SEG_LENGTH-2*n);
		BSP_LCD_DrawHLine(x + n, y - n, SEG_LENGTH-2*n);
	}
}

// specific segments values encoded for each number
const uint8_t SEG_DECODED[] = {0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F};

// draw a digit in a segmented display
void GUI_DrawSegDigit(uint16_t x, uint16_t y, uint8_t n, uint32_t colour){
	uint8_t mapped = (n>9 ? 0 : SEG_DECODED[n]);

	x += SEG_WIDTH/2; y += SEG_WIDTH/2;

	V_seg(x + SEG_LENGTH + 4, y + 3, mapped&0x02?colour:(colour&DIM_SPEED));	//b
	V_seg(x + 1, y + 3, mapped&0x20?colour:(colour&DIM_SPEED));	//f
	V_seg(x + SEG_LENGTH + 4, y + SEG_LENGTH + 6, mapped&0x04?colour:(colour&DIM_SPEED));	//c
	V_seg(x + 1, y + SEG_LENGTH + 6, mapped&0x10?colour:(colour&DIM_SPEED));	//e

	H_seg(x + 3, y + 1, mapped&0x01?colour:(colour&DIM_SPEED));	//a
	H_seg(x + 3, y + SEG_LENGTH + 4, mapped&0x40?colour:(colour&DIM_SPEED));	//g
	H_seg(x + 3, y + SEG_LENGTH*2 + 7, mapped&0x08?colour:(colour&DIM_SPEED));	//d

}

void GUI_DrawSpeed(uint16_t x, uint16_t y, uint8_t speed, uint32_t colour){	//up to 199

	uint8_t ns = speed / 100;
	uint8_t nd = speed / 10 - ns * 10;
	uint8_t nj = speed % 10;

	GUI_DrawSegDigit(x + SEG_WIDTH*2, y, nd, colour);
	GUI_DrawSegDigit(x + SEG_WIDTH*4 + SEG_LENGTH, y, nj, colour);

	x += SEG_WIDTH/2; y += SEG_WIDTH/2;
	if(carInfo.Speed<100) colour &= DIM_SPEED;
	V_seg(x + 1, y + 3, colour);
	V_seg(x + 1, y + SEG_LENGTH + 6, colour);
}

// draw all available icons
void GUI_IconsCheck(){
	GUI_PositionLights(LCD_COLOR_YELLOW);
	GUI_DrawLowBeam(LCD_COLOR_LIME);
	battError(LCD_COLOR_RED);
	engError(LCD_COLOR_RED);
	GUI_DrawCruise(LCD_COLOR_LIME);
	neutralGear(385, 150);
	GUI_DrawRegenBars(425, 153, 2, LCD_COLOR_LIME);
	leftIndicator(0xff00e000);
	rightIndicator(0xff00e000);
	GUI_DrawBattery(20, 130, 60);
	GUI_DisplayEnergy(20, 230, 115, 760);
	GUI_DisplayPower(4200, LCD_COLOR_WHITE);
	GUI_DrawSpeed(SPEED_X, SPEED_Y, 69, LCD_COLOR_WHITE);
	GUI_DrawComm(LCD_COLOR_GREEN);
	GUI_DrawTPMS(LCD_COLOR_PEACH);
}

void GUI_UpdateIcons(){
	lightsFrame_t change;
	*(uint16_t*)&change = (*(uint16_t*)&prevLights)^(*(uint16_t*)&carInfo.Lights);
	prevLights = carInfo.Lights;

	if(change.Stop){
		if (carInfo.Lights.Stop){
			GUI_DrawRectGradient(0, 0, 480, 16, LCD_COLOR_RED, DOWN);
		} else{
			BSP_LCD_SetTextColor(LCD_COLOR_TRANSPARENT);
			BSP_LCD_FillRect(0, 0, 480, 16);
		}
	}

	if(change.ParkLights){
		GUI_PositionLights(carInfo.Lights.ParkLights ? LCD_COLOR_YELLOW : LCD_COLOR_TRANSPARENT);
	}

	if(change.LowBeam){
		GUI_DrawLowBeam(carInfo.Lights.LowBeam ? LCD_COLOR_LIME : LCD_COLOR_TRANSPARENT);
	}

	//no high beams in the car :)
//	if(change.HighBeam){
//		if (carInfo.Lights.HighBeam){
//			highBeam(85, 80);
//		} else {
//			BSP_LCD_SetTextColor(LCD_COLOR_TRANSPARENT);
//			BSP_LCD_FillRect(85 - 21, 80, 50, 50);
//		}
//	}

//	not updated from lights frame anymore
//	if(change.BattError){
//		battError(carInfo.Lights.BattError ? LCD_COLOR_RED : LCD_COLOR_TRANSPARENT);
//	}
//
//	if(change.EngError){
//		engError(carInfo.Lights.EngError ? LCD_COLOR_RED : LCD_COLOR_TRANSPARENT);
//	}

	if(change.CruiseOn || change.CruiseActive){
		GUI_DrawCruise(carInfo.Lights.CruiseOn ? (carInfo.Lights.CruiseActive ? LCD_COLOR_GREEN : LCD_COLOR_ORANGE) : LCD_COLOR_TRANSPARENT);
	}

	if(change.LeftInd || change.RightInd || change.Hazard)
		frameCount = 0;

	if(change.DriveMode)
		GUI_UpdateGear();
}

void GUI_UpdateErrors(){
	battError(carInfo.BatteryError ? LCD_COLOR_RED : LCD_COLOR_TRANSPARENT);
	engError(carInfo.MotorsError ? LCD_COLOR_RED : carInfo.MotorsWarning ? LCD_COLOR_ORANGE : LCD_COLOR_TRANSPARENT);
}

void GUI_UpdateGear(){
	BSP_LCD_SetTextColor(LCD_COLOR_TRANSPARENT);
	BSP_LCD_FillRect(385, 150, 30, 52);
	if(carInfo.MotorsOn){
		switch(carInfo.Lights.DriveMode){
		case 0:
			reverseGear(385, 150);
			break;
		case 1:
			neutralGear(385, 150);
			break;
		case 2:
			ecoGear(385, 150);
			break;
		case 3:
			driveGear(385, 150);
			break;
		}
	} else{
		parkingGear(385, 150);
	}
}

void GUI_DrawCharging(uint32_t colour){
	GUI_DrawIconA(160, 100, &chargingIconA, colour);
}

void GUI_EnergyScreen(){ ////function that draws energy screen
    GUI_DrawBattery(250, 100, carInfo.SOC);
    GUI_DrawCharging(carInfo.Charger?LCD_COLOR_GREEN:LCD_COLOR_RED);
}

void GUI_SpeedScreen(){ //function that draws speed screen
	shownPower = GUI_SmoothPower(-carInfo.Power);
	GUI_DisplayPower(shownPower, LCD_COLOR_WHITE);
	GUI_UpdatePowerGradient(shownPower);
	if(carInfo.MotorsOn)
		GUI_DrawSpeed(SPEED_X, SPEED_Y, carInfo.Speed, LCD_COLOR_WHITE);
	else
		GUI_DrawSpeed(SPEED_X, SPEED_Y, 188, LCD_COLOR_WHITE&DIM_SPEED);

}
void GUI_DisplayPressure(uint16_t x, uint16_t y,uint16_t Pressure, uint32_t colour){
	char text[16];
	BSP_LCD_SetFont(&Font20);
	BSP_LCD_SetTextColor(colour);
	//float pressure_PSI = (float)Pressure / 100.0f;
	sprintf(text, "%d kPa", Pressure);
	BSP_LCD_DisplayStringAt(x, y, (uint8_t*)text, CENTER_MODE);
	BSP_LCD_SetFont(&Font20);
}

void GUI_UpdateTirePressure(){ //function that draws tire pressure screen
	GUI_DrawIconA(200, 60, &TireIconA, LCD_COLOR_WHITE);
	GUI_DisplayPressure(-75,70,carInfo.TirePressure.pressureFL,LCD_COLOR_WHITE);
	GUI_DisplayPressure(75,70,carInfo.TirePressure.pressureFR,LCD_COLOR_WHITE);
	GUI_DisplayPressure(-75,180,carInfo.TirePressure.pressureRL,LCD_COLOR_WHITE);
	GUI_DisplayPressure(75,180,carInfo.TirePressure.pressureRR,LCD_COLOR_WHITE);
}



void GUI_DisplayTemperature(uint16_t x, uint16_t y,uint16_t Temperature, uint32_t colour){
	char text[16];
	BSP_LCD_SetFont(&Font20);
	BSP_LCD_SetTextColor(colour);
	//uint16_t temperature = uint16_t Temperature / 100.0f;
	sprintf(text, "%d C", Temperature);
	BSP_LCD_DisplayStringAt(x, y, (uint8_t*)text, CENTER_MODE);
	BSP_LCD_SetFont(&Font20);
}

void GUI_UpdateMotorTemperatures() { //function that draws motor temperature screen
	GUI_DrawIconA(190, 60, &EngineTemperatureIconA, LCD_COLOR_WHITE);
	GUI_DisplayTemperature(-70, 150, carInfo.motor1Temperature, LCD_COLOR_WHITE);
	GUI_DisplayTemperature(70, 150, carInfo.motor2Temperature, LCD_COLOR_WHITE);
}


int ReadKeyPress() { //function that reads which button is pressed
    if (HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_6) == GPIO_PIN_SET) {
        return 1;  // Button 1 is pressed
    } else if (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_7) == GPIO_PIN_SET) {
        return 2;  // Button 2 is pressed
    } else {
        return 0;  // No button is pressed
    }
}

int currentScreen = 0;
int totalScreens = 3;

void handleButtonPress(uint8_t button) { //function that reads subsequent or previous screens
  if (button == 1) { // Forward button
    if (currentScreen < totalScreens - 1) {
      currentScreen++;
    }
  } else if (button == 2) { // Backward button
    if (currentScreen > 0) {
      currentScreen--;
    }
  }
}

typedef enum { //screen structure
	SPEED_SCREEN,
	ENERGY_SCREEN,
    TEMPERATURE_SCREEN,
    TPMS_SCREEN
} Screen_t;

void HandleButtons() { //the main function that operates the displayed screens
    static bool prevButtonState = false;
    static bool prevButton2State = false;

    int currentButtonState = ReadKeyPress(); // read current screen

    //Check whether the button 1 state has changed
    if (currentButtonState == 1 && !prevButtonState) {
    	BSP_LCD_FillRectColour(110, 60, 260, 170, LCD_COLOR_TRANSPARENT);
        // Next screen
        switch (currentScreen) {
            case ENERGY_SCREEN:
                currentScreen = SPEED_SCREEN;
                break;
            case SPEED_SCREEN:
                currentScreen = TEMPERATURE_SCREEN;
                break;
            case TEMPERATURE_SCREEN:
                currentScreen = TPMS_SCREEN;
                break;
            case TPMS_SCREEN:
                currentScreen = ENERGY_SCREEN;
                break;
            default:
                // Place to add new screens
                break;
        }
    }

    //Save current button 1 state
    prevButtonState = currentButtonState == 1;

    //Check whether the button 2 state has changed
    if (currentButtonState == 2 && !prevButton2State) {
    	BSP_LCD_FillRectColour(110, 60, 260, 170, LCD_COLOR_TRANSPARENT);
        // Previous screen
        switch (currentScreen) {
            case ENERGY_SCREEN:
                currentScreen = TPMS_SCREEN;
                break;
            case SPEED_SCREEN:
                currentScreen = ENERGY_SCREEN;
                break;
            case TEMPERATURE_SCREEN:
                currentScreen = SPEED_SCREEN;
                break;
            case TPMS_SCREEN:
                currentScreen = TEMPERATURE_SCREEN;
                break;
            default:
            	// Place to add new screens
                break;
        }
    }
    //Save current button 2 state
    prevButton2State = currentButtonState == 2;
}
void DrawCurrentScreen() { //function that draw current screen
	switch(currentScreen){
	case TPMS_SCREEN:
		GUI_UpdateTirePressure(LCD_COLOR_WHITE);
		break;
	case TEMPERATURE_SCREEN:
		GUI_UpdateMotorTemperatures(LCD_COLOR_WHITE);
		break;
	case SPEED_SCREEN:
		GUI_SpeedScreen(LCD_COLOR_WHITE);
		break;
	case ENERGY_SCREEN:
		GUI_EnergyScreen();
		break;
	}
}


// main render function
void GUI_Refresh(){
	if ((carInfo.Lights.LeftInd || carInfo.Lights.Hazard) && (frameCount <= frameRate/2)){
		leftIndicator(0xff00e000);
	} else {
		leftIndicator(LCD_COLOR_TRANSPARENT);
	}

	if ((carInfo.Lights.RightInd || carInfo.Lights.Hazard) && (frameCount <= frameRate/2)){
		rightIndicator(0xff00e000);
	} else {
		rightIndicator(LCD_COLOR_TRANSPARENT);
	}

	HandleButtons();
	DrawCurrentScreen();


	GUI_DrawComm(carInfo.Errors.ShowError?LCD_COLOR_RED:LCD_COLOR_GREEN);

	if(carInfo.Errors .ShowError){
		char text[5];
		sprintf(text, "%02X", carInfo.Errors.ShownErrors);
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		uint16_t posx=432,posy=128;
		BSP_LCD_SetFont(&Font20);
		BSP_LCD_DisplayStringAt(posx, posy, (uint8_t*)text, LEFT_MODE);
		BSP_LCD_SetFont(&Font24);
	} else{
		if(canErrorFrames) canErrorFrames--;
		else BSP_LCD_FillRectColour(425, 125, 40, 20, LCD_COLOR_TRANSPARENT);
	}

	GUI_DrawTPMS(carInfo.TirePressureLow?LCD_COLOR_PEACH:LCD_COLOR_TRANSPARENT);

	if(++frameCount>frameRate) frameCount = 0;
}

void GUI_ResetErrorCounter(){
	canErrorFrames = frameRate*5;

}
