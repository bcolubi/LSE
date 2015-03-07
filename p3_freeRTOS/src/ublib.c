/*
 * ublib.c
 *
 *  Created on: Mar 7, 2015
 *      Author: bcolubi
 */
#include "stm32f4_discovery.h"

static uint16_t i;
static uint16_t pin_reg, pin_value, pin_conv;

void initLeds() {

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef gpio;
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin =  GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;

    GPIO_Init(GPIOB, &gpio);

}

void initMisc(){
	pin_value = 0;
	pin_conv = 0;
	pin_reg = 3;	//simulate value from ISR or UART

	GPIO_WriteBit(GPIOB,GPIO_Pin_3, 1); //0x000008, 8
	GPIO_WriteBit(GPIOB,GPIO_Pin_4, 1); //0x000010, 16
	GPIO_WriteBit(GPIOB,GPIO_Pin_6, 1); //0x000020, 64
	GPIO_WriteBit(GPIOB,GPIO_Pin_7, 1); //0x000040, 128

}

void playLED(){
	conv_pin_reg();
	pin_value = !GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_3);
	GPIO_WriteBit(GPIOB,pin_conv, pin_value);
    Delay(0xFFFFFF); //1000ms
}

/*
void delay(uint32_t ms) {
    ms *= 3360;
    while(ms--) {
        __NOP();
    }
}
*/

void conv_pin_reg(){
	pin_conv = 1;
	for(i=0;i<pin_reg;i++){
		pin_conv*=2;
	}
}

void Delay(__IO uint32_t nCount){
  while(nCount--){
  }
}

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t* file, uint32_t line){
  while (1){
  }
}
#endif



