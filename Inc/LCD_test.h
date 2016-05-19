#ifndef LCD_TEST_H
#define LCD_TEST_H

#include <stdint.h>
#include <string.h>
#include "stm32l0xx_hal.h"
#include "flir_lcd.h"

extern uint8_t test,test1;
extern uint16_t testBuf1[640];

extern SPI_HandleTypeDef hspi1;

#endif
