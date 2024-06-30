//Driver for TM1637 LED driver by Nanoslavic: https://www.youtube.com/watch?v=aSUkUThnnHQ

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_rcc.h"
#include "tm1637_lib.h"

//GPIO pins for control TM1637
//CLK pin
#define TM1637_CLK_PORT		GPIOB
#define TM1637_CLK_PIN		GPIO_PIN_9

//DIO pin
#define TM1637_DIO_PORT		GPIOB
#define TM1637_DIO_PIN		GPIO_PIN_8

//Functions headers
void tm1637_SetBrightness(uint8_t brightness);
void tm1637_DisplayUpdate(uint8_t d0,uint8_t d1,uint8_t d2,uint8_t d3);
void tm1637_ShowNumber(uint16_t num);
uint8_t tm1637_NumberToSegments(uint8_t n);

void tm1637_Start(void);
void tm1637_Stop(void);
uint8_t tm1637_ReadACK(void);
void tm1637_WriteByte(uint8_t data);
void SomeDelay(uint32_t i);
void tm1637_CLK_SetHigh(void);
void tm1637_CLK_SetLow(void);
void tm1637_DIO_SetHigh(void);
void tm1637_DIO_SetLow(void);

//=================================================
//Display control functions

// Brightness values: 0 - 8
void tm1637_SetBrightness(uint8_t brightness){
	tm1637_Start();
	tm1637_WriteByte(0x87 + brightness);
	tm1637_ReadACK();
	tm1637_Stop();
}

//Send segments data into display
//d0 - ***8, d1 - **8*, d2 - *8**, d3 - 8***
void tm1637_DisplayUpdate(uint8_t d0,uint8_t d1,uint8_t d2,uint8_t d3){

	tm1637_Start();
	tm1637_WriteByte(0x40);//Memory write command
    tm1637_ReadACK();
    tm1637_Stop();

    tm1637_Start();
    tm1637_WriteByte(0xc0);//Start address
    tm1637_ReadACK();

	tm1637_WriteByte(d0);//Data
    tm1637_ReadACK();
	tm1637_WriteByte(d1);//Data
    tm1637_ReadACK();
	tm1637_WriteByte(d2);//Data
    tm1637_ReadACK();
	tm1637_WriteByte(d3);//Data
    tm1637_ReadACK();

    tm1637_Stop();
}

//Convert number to 7-segment code
uint8_t tm1637_NumberToSegments(uint8_t n){
	if (n == 0) return 0x3F;//0
	if (n == 1) return 0x06;//1
	if (n == 2) return 0x5B;//2
	if (n == 3) return 0x4F;//3
	if (n == 4) return 0x66;//4
	if (n == 5) return 0x6D;//5
	if (n == 6) return 0x7D;//6
	if (n == 7) return 0x07;//7
	if (n == 8) return 0x7F;//8
	if (n == 9) return 0x6F;//9
	if (n == 10) return 0x77;//A
	if (n == 11) return 0x7C;//B
	if (n == 12) return 0x39;//C
	if (n == 13) return 0x5E;//D
	if (n == 14) return 0x79;//E
	if (n == 15) return 0x71;//F
	if (n == 16) return 0x40;//-
	if (n == 17) return 0x77;//A
	if (n == 18) return 0x3D;//G
	if (n == 19) return 0x76;//H
	if (n == 20) return 0x3C;//J
	if (n == 21) return 0x73;//P
	if (n == 22) return 0x38;//L
	if (n == 23) return 0x6D;//S
	if (n == 24) return 0x3E;//U
	if (n == 25) return 0x6E;//Y
	return 0x00;
}

//Send number into display; BMS a bms se modificaron.
void tm1637_ShowNumber(uint16_t num){
	uint8_t dg0,dg1,dg2,dg3;
	dg0 = tm1637_NumberToSegments((uint8_t)(num / 1000));
	num = num % 1000;
	dg1 = tm1637_NumberToSegments((uint8_t)(num / 100));
	num = num % 100;
	dg2 = tm1637_NumberToSegments((uint8_t)(num / 10));
	num = num % 10;
	dg3 = tm1637_NumberToSegments((uint8_t)num);
	tm1637_DisplayUpdate(dg0,dg1,dg2,dg3);
}

//=================================================
//Protocol functions

//Start transfer signal
void tm1637_Start(void){
	tm1637_CLK_SetHigh();
	tm1637_DIO_SetHigh();
	SomeDelay(5);
	tm1637_DIO_SetLow();
}

//Stop transfer signal
void tm1637_Stop(void){
	tm1637_CLK_SetLow();
	SomeDelay(5);
	tm1637_DIO_SetLow();
    SomeDelay(5);
    tm1637_CLK_SetHigh();
    SomeDelay(5);
    tm1637_DIO_SetHigh();
}

//Read ack signal
uint8_t tm1637_ReadACK(void)
{
	tm1637_CLK_SetLow();
	tm1637_DIO_SetHigh();
	SomeDelay(7);
    uint8_t d = HAL_GPIO_ReadPin(TM1637_DIO_PORT, TM1637_DIO_PIN);
	tm1637_CLK_SetHigh();
	SomeDelay(5);
    tm1637_CLK_SetLow();
    return d;
}

//Send command or data into display
void tm1637_WriteByte(uint8_t data){
    for (uint8_t i = 0; i < 8; i++) {
    	tm1637_CLK_SetLow();
        if (data & 0x01) {
        	tm1637_DIO_SetHigh();
        } else {
        	tm1637_DIO_SetLow();
        };
        SomeDelay(7);
        data >>= 1;
        tm1637_CLK_SetHigh();
        SomeDelay(7);
    };
}

//=================================================
//We need some delay for protocol

//Delay approximately (0.95*i + 0.6) us on 72Mhz & optimization for size "-Os"
void SomeDelay(uint32_t i){
    while (i > 0) {
    	for (uint32_t j = 0; j < 10; j++) {
    		__NOP();
    	};
    	i--;
    };
}

//=================================================
//Pins control functions

//Set 1 on CLK pin
void tm1637_CLK_SetHigh(void){
	HAL_GPIO_WritePin(TM1637_CLK_PORT, TM1637_CLK_PIN, GPIO_PIN_SET);//CLK
}

//Set 0 on CLK pin
void tm1637_CLK_SetLow(void){
	HAL_GPIO_WritePin(TM1637_CLK_PORT, TM1637_CLK_PIN, GPIO_PIN_RESET);//CLK
}

//Set 1 on DIO pin
void tm1637_DIO_SetHigh(void){
	HAL_GPIO_WritePin(TM1637_DIO_PORT, TM1637_DIO_PIN, GPIO_PIN_SET);//DIO
}

//Set 0 on DIO pin
void tm1637_DIO_SetLow(void){
	HAL_GPIO_WritePin(TM1637_DIO_PORT, TM1637_DIO_PIN, GPIO_PIN_RESET);//DIO
}
