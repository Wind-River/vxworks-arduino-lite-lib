/* 
 * Copyright (c) 2016, Wind River Systems, Inc. 
 * 
 * Redistribution and use in source and binary forms, with or without modification, are 
 * permitted provided that the following conditions are met: 
 * 
 * 1) Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer. 
 * 
 * 2) Redistributions in binary form must reproduce the above copyright notice, 
 * this list of conditions and the following disclaimer in the documentation and/or 
 * other materials provided with the distribution. 
 * 
 * 3) Neither the name of Wind River Systems nor the names of its contributors may be 
 * used to endorse or promote products derived from this software without specific 
 * prior written permission. 
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE 
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
 */

/*
 * DESCRIPTION
 *  - arduino.h
 * Arduino API source header.
 */

#ifndef _ARDUINO_HPP_
#define _ARDUINO_HPP_

#include <vxWorks.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <ctype.h>        /* character functions */

#include <stdlib.h>

#include "wiring-lite/vxWorks/common/serial/serial.h"
#include "wiring-lite/vxWorks/intel-galileo-gen2/wiring-lite.h"

/* The following includes would normally be placed in the Arduino sketch by 
 * default. Uncomment the following to make this unnecessary.
 * #include <Wire.h>
 * #include <rgb_lcd.h>
*/

/*
 * Arduino pin number definitions
 */
#define IO0		0
#define IO1		1
#define IO2		2
#define IO3		3
#define IO4		4
#define IO5		5
#define IO6		6
#define IO7		7
#define IO8		8
#define IO9		9
#define IO10	10
#define IO11	11
#define IO12	12
#define IO13	13
#define IO14	14
#define IO15	15
#define IO16	16
#define IO17	17
#define IO18	18
#define IO19	19
#define A0		14
#define A1		15
#define A2		16
#define A3		17
#define A4		18
#define A5		19
#define	LED_BUILTIN		13

#define BIN   2
#define OCT   8
#define DEC   10
#define HEX   16

/* Interrupt modes */
#define	LOW		0x0
#define	HIGH	0x1
#define	RISING	0x02
#define	FALLING	0x04
#define	CHANGE	0x08

/* Digital i/o pin modes - used when calling pinMode and by osPinMode
 * at a lower level
 */
#define INPUT           0x01
#define	INPUT_PULLUP	  0x02
#define	INPUT_PULLDOWN	0x03
#define INPUT_FAST		  0x04
#define OUTPUT          0x05
#define	OUTPUT_FAST		  0x06

/*
 * This is our list of potential pin modes/states. These are used when
 * calling the pinMux function to set a pin to a specific mode.
 */
#define MUX_SET     0x80
#define MUX_GPIO    (MUX_SET | 0x10)
#define MUX_UART    (MUX_SET | 0x20)
#define MUX_PWM     (MUX_SET | 0x30)
#define MUX_SPI     (MUX_SET | 0x40)
#define MUX_ADC     (MUX_SET | 0x50)
#define MUX_I2C     (MUX_SET | 0x60)

/*
 * These are used in the delay and timer routines
 */
#define MS_PER_SEC	1000
#define US_PER_SEC	1000000

#define	DEFAULT	0

/*
#define true 0x1
#define false 0x0
*/

#define ANALOG_READ_RESOLUTION_MIN    8
#define ANALOG_READ_RESOLUTION_MAX    12
#define ANALOG_WRITE_RESOLUTION_MIN   8
#define ANALOG_WRITE_RESOLUTION_MAX   12

/*
 * Used when spawning tasks. The default task priority is used for the
 * Arduino task. The interrupt monitor task uses this valie minus 1
 */
#define DEFAULT_TASK_PRIORITY 60
#define DEFAULT_STACK_SIZE    2048

/* modulo range applied to random numbers from rand() function. */
#define MODULO_RAND           65536
#define MAX_DEFAULT_RAND      -999

void startArduino(void);
void runArduino(void);
void stopArduino(void);

/* Time */
unsigned long millis(void);
unsigned long micros(void);
void delay(unsigned long ms);
void delayMicroseconds(unsigned int us);

/* Random Numbers */
void randomSeed(uint32_t seed);

/* Advanced I/O */
void tone(uint8_t pin, unsigned int frequency, unsigned long duration); /* XXX: Add duration default of 0 */
void noTone(uint8_t pin);
void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t value);
uint8_t shiftIn(uint8_t dataPin, uint8_t clockPing, uint8_t bitOrder);
unsigned long pulseIn(uint8_t pin, uint8_t value, unsigned long timeout); /* XXX: Add timeout default of 1000000L */

/* Math */
uint32_t map(uint32_t value, uint32_t fromLow, uint32_t fromHigh, \
             uint32_t toLow,uint32_t toHigh);

/* Interrupts */
void attachInterrupt(uint32_t pin, void (*callback)(void), uint32_t mode);
void detachInterrupt(uint32_t pin);
void interrupts(void);
void noInterrupts(void);

#ifdef __cplusplus
#define min(x,y)	((x)<(y)?(x):(y))
#define max(x,y)	((x)>(y)?(x):(y))
#endif  /* __cplusplus */
#define abs(x)		((x)>0?(x):-(x))
#define constrain(x,a,b)	((x)<(a)?(a):((x)>(b)?(b):(x)))
#define	sq(x)	((x)*(x))

/* Characters */
#define isAlphaNumeric(x) (isalnum(x) ? TRUE : FALSE)     /* isAlphaNumeric() */
#define isAlpha(x)        (isalpha(x) ? TRUE : FALSE)     /* isAlpha() */
#define isAscii(x)        (isascii(x) ? TRUE : FALSE)     /* isAscii() */
#define isWhitespace(x)   (isblank(x) ? TRUE : FALSE)     /* isWhitespace() */
#define isControl(x)      (iscntrl(x) ? TRUE : FALSE)     /* isControl() */
#define isDigit(x)        (isdigit(x) ? TRUE : FALSE)     /* isDigit() */
#define isGraph(x)        (isgraph(x) ? TRUE : FALSE)     /* isGraph() */
#define isLowerCase(x)    (islower(x) ? TRUE : FALSE)     /* isLowerCase() */
#define isPrintable(x)    (isprint(x) ? TRUE : FALSE)     /* isPrintable() */
#define isPunct(x)        (ispunct(x) ? TRUE : FALSE)     /* isPunct() */
#define isSpace(x)        (isspace(x) ? TRUE : FALSE)     /* isSpace() */
#define isUpperCase(x)    (isupper(x) ? TRUE : FALSE)     /* isUpperCase() */
#define isHexadecimalDigit(x)   (isxdigit(x) ? TRUE : FALSE)  /* isHexadecimalDigit() */

/* Bits and Bytes */
#define lowByte(x)		((uint8_t) ((x) & 0xff))
#define highByte(x)		((uint8_t) ((x) >> 8))
#define bitRead(x,n)	(((x) >> (n)) & 0x01)
#define bitWrite(x,n,b)	(b ? bitSet(x,n) : bitClear(x,n))
#define bitClear(x,n)	((x) &= ~(1UL << (n)))
#define bitSet(x,n)		((x) |= (1UL << (bit)))
#define bit(n)			(1UL << (b))

typedef unsigned char byte;

/* Communication */
/* USB */

extern void setup(void);
extern void loop(void);

extern int pwmPeriod;
extern bool arduino_running;

#ifdef __cplusplus

long random(long min, long max = MAX_DEFAULT_RAND);

#endif /*  __cplusplus */

#endif /* _ARDUINO_HPP_ */
