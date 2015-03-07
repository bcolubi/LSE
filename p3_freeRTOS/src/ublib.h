/*
 * ublib.h
 *
 *  Created on: Mar 7, 2015
 *      Author: bcolubi
 */

#ifndef UBLIB_H_
#define UBLIB_H_

#include "stm32f4_discovery.h"

void Delay(__IO uint32_t nCount);
void delay(uint32_t ms); //not used yet. Need to modify.
void init();
void loop();
void initLeds();
void initMisc();
void conv_pin_reg();
void playLED();

#endif /* UBLIB_H_ */
