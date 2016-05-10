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
 *  - Arduino-lite.cpp
 * Arduino API source.
 */

#include <vxWorks.h>
#include <math.h>
#include <tickLib.h>
#include <sysLib.h>
#include <stdlib.h>
#include <stdio.h>
#include <taskLib.h>
#include <fcntl.h>
#include <ioLib.h>
#include <stddef.h>
#include <string.h>
#include <i2c.h>
#include <assert.h>

#include <Arduino-lite.h>
#include "../intel-galileo-gen2/wiring-lite.h"

/* ******************************************************************* */
/* Add these defines to enable debug and/or error message printing
 * Messages take the form...
 *        DEBUG_ARDUINO("Debug data is %d\n", value);
 *        ERROR_ARDUINO("Error message: %.2d\n", floatvalue);
*/
#define ENABLE_ERROR_ARDUINO
/* #define ENABLE_DEBUG_ARDUINO */

#ifdef ENABLE_ERROR_ARDUINO
#define ERROR_ARDUINO(...) fprintf (stderr, __VA_ARGS__)
#else
#define ERROR_ARDUINO(...)
#endif
#ifdef ENABLE_DEBUG_ARDUINO
#define DEBUG_ARDUINO(...) fprintf (stderr, __VA_ARGS__)
#else
#define DEBUG_ARDUINO(...)
#endif
/* ******************************************************************* */

bool arduino_running = FALSE;

/*******************************************************************************
 * main - The main for Arduino API
 *
 * Calls the user's setup function then runs the user's loop function indefinitely.
 *
 * First break point for Helix App Cloud Debugger
 *
 * Returns:  void
 */
int main(void)
{
	startArduino();
	while(1)
	{
		/* now run forever */
	}
}

/*******************************************************************************
 * startArduino - this is where the system is initialised and the main
 *                Arduino task is spawned.
 *
 * Parameters: none
 * Returns:    none
 */
void startArduino(void)
{
	pinInit();
	if (arduino_running)
	{
		ERROR_ARDUINO("[ERROR] startArduino - tArduino is already running\n");
	}
	arduino_running = TRUE;

	/* this taskspawn was originally at priority 100 - changed to 60 */
    taskSpawn ((char *)"tArduino", DEFAULT_TASK_PRIORITY, 0, DEFAULT_STACK_SIZE,
    		(FUNCPTR)runArduino, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

	/* -- if the above taskspawn fails then run the function directly -- */
	/* runArduino(); */
}

/*******************************************************************************
 * runArduino - this is the function that runs within the context of the
 *              tArduino task. This loop part of this function/task will only
 *              run as long as the arduino_running variable is TRUE
 *              (otherwise this function (and the task) will run to
 *              completion).
 *
 * Parameters: none
 * Returns:    none
 */
void runArduino(void)
{
	setup();
	while (arduino_running)
	{
		loop();
	}
}

/*******************************************************************************
 * stopArduino - run this function to reset the arduino_running variable and
 *               cause the tArduino task to terminate.
 *
 * Parameters: none
 * Returns:    none
 */
void stopArduino(void)
{
	arduino_running = FALSE;
}

/*******************************************************************************
 * millis - Arduino API call, return the number of milliseconds since the
 *            Arduino started running - may wrap to 0 at some point
 *
 * Arguments: none
 *
 * Return:    unsigned long, number of milliseconds
 */
unsigned long millis(void)
{
	UINT64 ticks = tick64Get();
	return ticks * (MS_PER_SEC/sysClkRateGet());
}

/*******************************************************************************
 * micros - Arduino API call, return the number of microseconds since the
 *            Arduino started running - may wrap to 0 at some point
 *
 * Arguments: none
 *
 * Return:    unsigned long, number of microseconds
 */
unsigned long micros(void)
{
	UINT64 ticks = tick64Get();
	return ticks * (US_PER_SEC/sysClkRateGet());
}

/*******************************************************************************
 * delay - Arduino API call, delay for a number of milliseconds
 *
 * Arguments: unsigned long, number of ms
 *
 * Return:    void
 */
void delay(unsigned long ms)
{
	usleep(ms * 1000);
}

/*******************************************************************************
 * delayMicroseconds - Arduino API call, delay for a number of microseconds
 *
 * Arguments: unsigned int, number of microseconds
 *
 * Return:    void
 */
void delayMicroseconds(unsigned int us)
{
    usleep(us);
}

/* Random Numbers */

/*******************************************************************************
 * randomSeed - Seed the random number generator with a new pseudo random number
 *
 * Arguments: unsigned int, seed variable
 *
 * Return:    none
 */
void randomSeed(uint32_t seed)
{
	srand(seed);
}

/* Advanced I/O */

/*******************************************************************************
 * tone - play a tone on a buzzer or speaker
 *
 * Arguments: unsigned int8 - pin, where buzzer is connected
 *            unsigned int  - frequency, of the tone to be played
 *            unsigned long - duration, of the tone
 *
 * Return:    none
 */
void tone(uint8_t pin, unsigned int frequency, unsigned long duration)
{
	/* not supported on Galileo Gen2 with VxWorks */
	DEBUG_ARDUINO("[DEBUG] arduino-tone - not supported\n");
	return;
}

/*******************************************************************************
 * noTone - stop playing a tone on a buzzer or speaker
 *
 * Arguments: unsigned int8 - pin, where buzzer is connected
 *
 * Return:    none
 */
void noTone(uint8_t pin)
{
	/* not supported on Galileo Gen2 with VxWorks */
	DEBUG_ARDUINO("[DEBUG] arduino-noTone - not supported\n");
	return;
}

/*******************************************************************************
 * shiftOut - shift data out of a data pin using another pin for a clock pin.
 *
 * Arguments: unsigned int8 - dataPin, used for the data bits themselves
 *            unsigned int8 - clockPin, used as a sync clock pin
 *            unsigned int8 - bitOrder, either MSBFIRST or LSBFIRST
 *            unsigned int8 - value, the byte of data to send
 *
 * Return:    none
 */
void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t value)
{
	DEBUG_ARDUINO("[DEBUG] arduino-shiftOut - not supported\n");
	return;
}

/*******************************************************************************
 * shiftIn - shift data into a data pin using another pin for a clock pin.
 *
 * Arguments: unsigned int8 - dataPin, used for the data bits themselves
 *            unsigned int8 - clockPin, used as a sync clock pin
 *            unsigned int8 - bitOrder, either MSBFIRST or LSBFIRST
 *
 * Return:    unsigned int8 - single byte of data received
 */
uint8_t shiftIn(uint8_t dataPin, uint8_t clockPing, uint8_t bitOrder)
{
	DEBUG_ARDUINO("[DEBUG] arduino-shiftIn - not supported\n");
	return 0;
}

/*******************************************************************************
 * pulseIn - time the duration of a pulse (either high or low) that arrives
 *           on a pin.
 *
 * Arguments: unsigned int8 - dataPin, where to watch for a pulse
 *            unsigned int8 - value, watch for either HIGH or LOW pulse
 *            unsigned long - timeout, time to wait (in uS) for pulse to arrive
 *                            (defaults to 1 second)
 *
 * Return:    unsigned long - length of pulse (in uS) or 0 for no pulse
 */
unsigned long pulseIn(uint8_t pin, uint8_t value, unsigned long timeout)
{
	/* TODO - set default value of timeout to 1000000L */
	DEBUG_ARDUINO("[DEBUG] arduino-pulseIn - not supported\n");
	return 0;
}

/* Math */

/*******************************************************************************
 * map - map a value from one range to another
 *
 * Arguments: uint32 - value, the input value
 *            uint32 - fromLow, 'from' range low value
 *            uint32 - fromHigh, 'from' range high value
 *            uint32 - toLow, 'to' range low value
 *            uint32 - toHigh, 'to' range high value
 *
 * Return:    uint32, the re-mapped value
 */
uint32_t map(uint32_t value, uint32_t fromLow, uint32_t fromHigh, \
             uint32_t toLow, uint32_t toHigh)
{
	return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}

/* Interrupts */

/*******************************************************************************
 * attachInterrupt - attach an interrupt (callback routine) to a digital I/O pin
 *
 * Arguments: int32 - pin, to detach
 *            *callback - the callback routine to call when the interrupt hits
 *            uint32 - mode, of the interrupt (either CHANGE, FALLING, RISING,
 *                                 HIGH or LOW)
 *
 * Return:    none
 */
void attachInterrupt(uint32_t pin, void (*callback)(void), uint32_t mode)
{
	attachInterruptSource(pin, callback, mode);
}

/*******************************************************************************
 * detachInterrupt - detach an interrupt from a digital I/O pin (that was
 *                   previously attached using the attachInterrupt routine)
 *
 * Arguments: int32 - pin, to detach
 * Return:    none
 */
void detachInterrupt(uint32_t pin)
{
	detachInterruptSource(pin);
}

/*******************************************************************************
 * interrupts - enable interrupts from Arduino digital I/O pins
 *
 * Arguments: none
 * Return:    none
 */
void interrupts(void)
{
	interruptsEnabled(TRUE);
}

/*******************************************************************************
 * noInterrupts - disable interrupts from Arduino digital I/O pins
 *
 * Arguments: none
 * Return:    none
 */
void noInterrupts(void)
{
	interruptsEnabled(FALSE);
}


/* ************* Below this line are C++ classes and functions ************** */

using namespace std;

/*******************************************************************************
 * random - return a random number that is between specific limits
 *
 * Arguments: long min, lowest random number required, optional, default 0
 *            long max, highest random number required
 * Return:    long int, random number
 *
 * Syntax:    random(min, max)
 *        or  random(max)
 */
long random(long min, long max)
{
	int randomNumber;

	if(max == MAX_DEFAULT_RAND)
	{
		max = min;
		min = 0;
	}
	randomNumber = (rand() % MODULO_RAND);
	return map(randomNumber, 0, MODULO_RAND, min, max);
}

