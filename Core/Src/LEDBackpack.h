/*
 * LEDBackpack.h
 *
 *  Created on: Apr 8, 2021
 *      Author: czq
 */

#ifndef SRC_LEDBACKPACK_H_
#define SRC_LEDBACKPACK_H_

#include "stm32l0xx_hal.h"
#include <stdbool.h>
#include "main.h"

#define i2c_addr 0x70<<1

#define HT16K33_BLINK_CMD 0x80       ///< I2C register for BLINK setting
#define HT16K33_BLINK_DISPLAYON 0x01 ///< I2C value for steady on
#define HT16K33_BLINK_OFF 0          ///< I2C value for steady off
#define HT16K33_BLINK_2HZ 1          ///< I2C value for 2 Hz blink
#define HT16K33_BLINK_1HZ 2          ///< I2C value for 1 Hz blink
#define HT16K33_BLINK_HALFHZ 3       ///< I2C value for 0.5 Hz blink
#define HT16K33_CMD_BRIGHTNESS 0xE0 ///< I2C register for BRIGHTNESS setting

uint16_t displaybuffer[4];

void setBrightness(uint8_t b);

void blinkRate(uint8_t b);

void HKbegin(void);

void writeDisplay(void);

void clear(void);

void writeDigitRaw(uint8_t n, uint16_t bitmask);

void writeDigitAscii(uint8_t n, uint8_t a);

#endif /* SRC_LEDBACKPACK_H_ */

