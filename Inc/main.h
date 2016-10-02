/*
 * main.h
 *
 *  Created on: 30. Juli 2016
 *      Author: Mark
 */

#ifndef MAIN_H_
#define MAIN_H_
/* Includes ------------------------------------------------------------------*/


/*HAL*/
#include "stm32f7xx_hal.h"
/*CMSIS*/
//#include "stm32f745xx.h"
/*USB*/
#include "usb_device.h"
/*BSP*/
#include "stm32746g_discovery_lcd.h"
#include "stm32746g_discovery.h"
#include "stm32746g_discovery_sdram.h"
#include "stm32746g_discovery_ts.h"
/*STemWin*/
#include "GUI.h"
#include "WM.h"
#include "DIALOG.h"
#include "GRAPH.h"
WM_HWIN CreateWindow(void);
WM_HWIN CreateGraph(void);
/*Variables*/
UART_HandleTypeDef huart6; //for debug

#endif /* MAIN_H_ */
