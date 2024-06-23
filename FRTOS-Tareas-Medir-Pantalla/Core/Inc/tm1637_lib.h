//Driver for TM1637 LED driver by Nanoslavic

#ifndef INC_TM1637_LIB_H_
#define INC_TM1637_LIB_H_

void tm1637_SetBrightness(uint8_t brightness);
void tm1637_DisplayUpdate(uint8_t d0,uint8_t d1,uint8_t d2,uint8_t d3);
void tm1637_ShowNumber(uint16_t num);
uint8_t tm1637_NumberToSegments(uint8_t n);

#endif /* INC_TM1637_LIB_H_ */
