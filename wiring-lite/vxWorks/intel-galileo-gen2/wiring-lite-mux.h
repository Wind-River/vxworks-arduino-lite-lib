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

#ifndef __INC_VX_GALILEO_MUX_H__
#define __INC_VX_GALILEO_MUX_H__

#include "wiring-lite.h"

/*
 * This enum defines the modes that each pins can support. Values defined
 * here are used to reference members of the pinMuxFs structure below. Any new
 * addition to this enum must be mirrored, and in the same order as, in the
 * pinMuxFs structure !
 */
enum pin_tag  {
    IO0_GPIO, IO0_UART,
    IO1_GPIO, IO1_UART,
    IO2_GPIO, IO2_UART,
    IO3_GPIO, IO3_UART,  IO3_PWM,
    IO4_GPIO,
    IO5_GPIO,            IO5_PWM,
    IO6_GPIO,            IO6_PWM,
    IO7_GPIO,
    IO8_GPIO,
    IO9_GPIO,            IO9_PWM,
    IO10_GPIO,          IO10_PWM,
    IO11_GPIO,          IO11_PWM, IO11_SPI,
    IO12_GPIO,                    IO12_SPI,
    IO13_GPIO,                    IO13_SPI,
    IO14_GPIO,                              IO14_ADC,
    IO15_GPIO,                              IO15_ADC,
    IO16_GPIO,                              IO16_ADC,
    IO17_GPIO,                              IO17_ADC,
    IO18_GPIO,                              IO18_ADC, IO18_I2C,
    IO19_GPIO,                              IO19_ADC, IO19_I2C,
};  /* pintags; */

/*
 * This structure contains details of how the two mux pins are setup to
 * establish a specific pin in a specific state. These entries must be in
 * the same order as the pintags enum.
 */
static struct {
	UINT8  setupRef;
	UINT16 enablePin1;
	UINT16 enablePin2;
} pinMuxFs[] =
{
	{  IO0_GPIO,    0xff,    0xff},   /*  IO0 gpio/11 */
	{  IO0_UART,    0xff,    0xff},   /*  IO0 uart */
	{  IO1_GPIO, 45 | LO,    0xff},   /*  IO1 gpio/12 */
	{  IO1_UART, 45 | HI,    0xff},   /*  IO1 uart */
	{  IO2_GPIO, 77 | LO,    0xff},   /*  IO2 gpio/13 */
	{  IO2_UART, 77 | HI,    0xff},   /*  IO2 uart */
	{  IO3_GPIO, 76 | LO, 64 | LO},   /*  IO3 gpio/14 */
	{  IO3_UART, 76 | HI, 64 | DE},   /*  IO3 uart */
	{  IO3_PWM,  76 | LO, 64 | HI},   /*  IO3 pwm/1 */
	{  IO4_GPIO,    0xff,    0xff},   /*  IO4 gpio/6 */
	{  IO5_GPIO, 66 | LO,    0xff},   /*  IO5 gpio/0 */
	{  IO5_PWM,  66 | HI,    0xff},   /*  IO5 pwm/3 */
	{  IO6_GPIO, 68 | LO,    0xff},   /*  IO6 gpio/1 */
	{  IO6_PWM,  68 | HI,    0xff},   /*  IO6 pwm/5 */
	{  IO7_GPIO,    0xff,    0xff},   /*  IO7 gpio/38 */
	{  IO8_GPIO,    0xff,    0xff},   /*  IO8 gpio/40 */
	{  IO9_GPIO, 70 | LO,    0xff},   /*  IO9 gpio/4 */
	{  IO9_PWM,  70 | HI,    0xff},   /*  IO9 pwm/7 */
	{ IO10_GPIO, 74 | LO,    0xff},   /* IO10 gpio/10 */
	{ IO10_PWM,  74 | HI,    0xff},   /* IO10 pwm/11 */
	{ IO11_GPIO, 44 | LO, 72 | LO},   /* IO11 gpio/5 */
	{ IO11_PWM,  44 | HI, 72 | LO},   /* IO11 pwm/9 */
	{ IO11_SPI,  44 | DE, 72 | HI},   /* IO11 spi */
	{ IO12_GPIO,    0xff,    0xff},   /* IO12 gpio/15 */
	{ IO12_SPI,     0xff,    0xff},   /* IO12 spi */
	{ IO13_GPIO, 46 | LO,    0xff},   /* IO13 gpio/7 */
	{ IO13_SPI,  46 | HI,    0xff},   /* IO13 spi */
	{ IO14_GPIO,    0xff,    0xff},   /* IO14 gpio/48 */
	{ IO14_ADC,     0xff,    0xff},   /* IO14 adc/0 */
	{ IO15_GPIO,    0xff,    0xff},   /* IO15 gpio/50 */
	{ IO15_ADC,     0xff,    0xff},   /* IO15 adc/1 */
	{ IO16_GPIO,    0xff,    0xff},   /* IO16 gpio/52 */
	{ IO16_ADC,     0xff,    0xff},   /* IO16 adc/2 */
	{ IO17_GPIO,    0xff,    0xff},   /* IO17 gpio/54 */
	{ IO17_ADC,     0xff,    0xff},   /* IO17 adc/3 */
	{ IO18_GPIO, 60 | HI, 78 | HI},   /* IO18 gpio/56 */
	{ IO18_ADC,  60 | HI, 78 | LO},   /* IO18 adc/4 */
	{ IO18_I2C,  60 | LO, 78 | DE},   /* IO18 i2c/sda */
	{ IO19_GPIO, 60 | HI, 79 | HI},   /* IO19 gpio/58 */
	{ IO19_ADC,  60 | HI, 79 | LO},   /* IO19 adc/5 */
	{ IO19_I2C,  60 | LO, 79 | DE},   /* IO19 i2c/scl */
};

/*
 * This structure includes the information needed to setup specific IO pins.
 * The columns called setupXxxx specify a reference to the list above which,
 * in turn defines how the mux pins need to be set for that specific setup.
 */
static struct pintablefs {
	UINT8  gpioPin;
	UINT8  directionPin;
	UINT8  pullupPin;
	UINT8  pwmPin;
	int    setupGpio;
	int    setupUart;
	int    setupPwm;
	int    setupSpi;
	int    setupAdc;
	int    setupI2c;
	UINT16 pinStatus;
} pinTableFs[NUM_DIGITAL_PINS] =
{
/*    gpio dir  pu   pwm     GPIO      UART        PWM       SPI       ADC       I2C  */
	{ 11,   32, 33, 0xff,  IO0_GPIO, IO0_UART,     0xff,     0xff,     0xff,     0xff, 0},   /* IO0 */
	{ 12,   28, 29, 0xff,  IO1_GPIO, IO1_UART,     0xff,     0xff,     0xff,     0xff, 0},   /* IO1 */
	{ 13,   34, 35, 0xff,  IO2_GPIO, IO2_UART,     0xff,     0xff,     0xff,     0xff, 0},   /* IO2 */
	{ 14,   16, 17,    1,  IO3_GPIO, IO3_UART,  IO3_PWM,     0xff,     0xff,     0xff, 0},   /* IO3 */
	{  6,   36, 37, 0xff,  IO4_GPIO,     0xff,     0xff,     0xff,     0xff,     0xff, 0},   /* IO4 */
	{  0,   18, 19,    3,  IO5_GPIO,     0xff,  IO5_PWM,     0xff,     0xff,     0xff, 0},   /* IO5 */
	{  1,   20, 21,    5,  IO6_GPIO,     0xff,  IO6_PWM,     0xff,     0xff,     0xff, 0},   /* IO6 */
	{ 38, 0xff, 39, 0xff,  IO7_GPIO,     0xff,     0xff,     0xff,     0xff,     0xff, 0},   /* IO7 */
	{ 40, 0xff, 41, 0xff,  IO8_GPIO,     0xff,     0xff,     0xff,     0xff,     0xff, 0},   /* IO8 */
	{  4,   22, 23,    7,  IO9_GPIO,     0xff,  IO9_PWM,     0xff,     0xff,     0xff, 0},   /* IO9 */
	{ 10,   26, 27,   11, IO10_GPIO,     0xff, IO10_PWM,     0xff,     0xff,     0xff, 0},   /* IO10 */
	{  5,   24, 25,    9, IO11_GPIO,     0xff, IO11_PWM, IO11_SPI,     0xff,     0xff, 0},   /* IO11 */
	{ 15,   42, 43, 0xff, IO12_GPIO,     0xff,     0xff, IO12_SPI,     0xff,     0xff, 0},   /* IO12 */
	{  7,   30, 31, 0xff, IO13_GPIO,     0xff,     0xff, IO13_SPI,     0xff,     0xff, 0},   /* IO13 */
	{ 48, 0xff, 49, 0xff, IO14_GPIO,     0xff,     0xff,     0xff, IO14_ADC,     0xff, 0},   /* IO14 - DIO in, ADC in */
	{ 50, 0xff, 51, 0xff, IO15_GPIO,     0xff,     0xff,     0xff, IO15_ADC,     0xff, 0},   /* IO15 - DIO in, ADC in */
	{ 52, 0xff, 53, 0xff, IO16_GPIO,     0xff,     0xff,     0xff, IO16_ADC,     0xff, 0},   /* IO16 - DIO in, ADC in */
	{ 54, 0xff, 55, 0xff, IO17_GPIO,     0xff,     0xff,     0xff, IO17_ADC,     0xff, 0},   /* IO17 - DIO in, ADC in */
	{ 56,    0, 57, 0xff, IO18_GPIO,     0xff,     0xff,     0xff, IO18_ADC, IO19_I2C, 0},   /* IO18 */
	{ 58,    0, 59, 0xff, IO19_GPIO,     0xff,     0xff,     0xff, IO19_ADC, IO19_I2C, 0},   /* IO19 */
};

#endif   /* __INC_VX_GALILEO_MUX_H__ */
