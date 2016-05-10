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

#ifndef __INC_VX_GALILEO_IO_H
#define __INC_VX_GALILEO_IO_H

#define NUM_DIGITAL_PINS  20
#define NUM_ANALOG_INPUTS  6

/* GPIO pin status indicators */
#define GPIO_STATUS_ALLOCATED           0x0001
#define GPIO_STATUS_PULLUP_ALLOCATED    0x0002
#define GPIO_STATUS_PULLUP              0x0004
#define GPIO_STATUS_PULLDOWN            0x0008
#define GPIO_STATUS_OUTPUT              0x0010
#define GPIO_STATUS_INPUT               0x0020

#define PIN_SETUP_AS_GPIO               0x0100
#define PIN_SETUP_AS_UART               0x0200
#define PIN_SETUP_AS_PWM                0x0400
#define PIN_SETUP_AS_SPI                0x0800
#define PIN_SETUP_AS_ADC                0x1000
#define PIN_SETUP_AS_I2C                0x2000

#define LO      0x0100      /* set pin low */
#define HI      0x0200      /* set pin high */
#define DE      0x0400      /* deallocated */

extern int adcReadResolution;
extern int pwmWriteResolution;

/*
 * This structure is used when monitoring pseudo-interrupts.
 */
struct interruptSetup
{
	/* variables for polled int.1 */
    int 	pin1;
    void 	(*callback1)(void);
    int 	mode1;
    int 	fd1;
    uint8_t psstate1;
	/* variables for polled int.2 */
    int 	pin2;
    void 	(*callback2)(void);
    int 	mode2;
    int 	fd2;
    uint8_t psstate2;
	/* general polled interrupt control */
    int 	cycleTime;
    int 	interruptsEnabled;
    int 	interruptsAttached;
};

#define DEFAULT_INT_CYCLETIME  2;  /* in ticks */


#ifdef __cplusplus
extern "C" {
#endif

STATUS pinMux(uint8_t pin, uint8_t mode);
STATUS setupPin(int sequence);
STATUS pinInit();
STATUS pinConfigDefault();

void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(register uint8_t pin, register uint8_t val);
int digitalRead(uint8_t pin);
int analogRead(register uint8_t pin);
void analogWrite(uint32_t pin, uint32_t value);
void analogReference(uint8_t type);
void analogReadResolution(int numBits);
void analogWriteResolution(int numBits);

STATUS attachInterruptSource(uint32_t pin, void (*callback)(void), uint32_t mode);
void runPseudoInterrupt(void);
void detachInterruptSource(uint32_t pin);
void interruptsEnabled(int state);
void usleep (long microseconds);

#ifdef __cplusplus
}
#endif

#endif /* __INC_VX_GALILEO_IO_H */
