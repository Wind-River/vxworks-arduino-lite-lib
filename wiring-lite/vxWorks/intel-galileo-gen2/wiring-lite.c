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

/* Description:
 *  vxworks-galileo-io.c - Galileo I/O library
 */

/*
modification history
--------------------
03a,04mar16,pty  Heavily modified for Gal.Gen2 Arduino support & fs interface
02a,26jun14,rcw	 Ported to VxWorks 7
01a,17apr14,dm   First version (based on the Galileo VxWorks Arduino libraries)
*/

/*
 * DESCRIPTION
 * This module is an access library for the analog and digital IOs present
 * on a Galileo board.
 */

#include <vxWorks.h>
#include <time.h>
#include <sysLib.h>
#include <stdio.h>
#include <taskLib.h>
#include <string.h>
#include <tickLib.h>
#include <semLib.h>

#include "wiring-lite.h"
#include "wiring-lite-mux.h"
#include "gpioutils.h"
#include "../../../Arduino-lite.h"

/* ******************************************************************* */
/* Add these define to enable debug and/or error message printing
 * Messages take the form...
 *        DEBUG_VXGAL("Debug data is %d\n", value);
 *        ERROR_VXGAL("Error message: %.2d\n", floatvalue);
*/
#define ENABLE_ERROR_VXGAL
/* #define ENABLE_DEBUG_VXGAL */

#ifdef ENABLE_ERROR_VXGAL
#define ERROR_VXGAL(...) fprintf (stderr, __VA_ARGS__)
#else
#define ERROR_VXGAL(...)
#endif
#ifdef ENABLE_DEBUG_VXGAL
#define DEBUG_VXGAL(...) fprintf (stderr, __VA_ARGS__)
#else
#define DEBUG_VXGAL(...)
#endif
/* ******************************************************************* */

/* Pseudo Interrupt control structure */
struct interruptSetup pInt = {};
SEM_ID pIntSem;             /* ID of pseudo interrupt enablement semaphore */

/*******************************************************************************
 * pinMux - sets an Arduino pin to a specific i/o state
 *
 * Arguments: pin  - Arduino connector pin number
 *            mode - mode/state to set (see header file)
 *
 * Returns:   0 - Success
 *           -1 - Failure
 */
STATUS pinMux(uint8_t pin, uint8_t mode)
{
    int returnStatus = OK;

    switch(mode) {
    case MUX_GPIO:
        if(pinTableFs[pin].setupGpio == 0xff)
        {
            returnStatus = ERROR;   // requested pin state not allowed
            break;
        }
        if(pinTableFs[pin].pinStatus & PIN_SETUP_AS_GPIO)
        {
            break;          // requested pin state already active
        }
        if(setupPin(pinTableFs[pin].setupGpio) == OK)
        {
            pinTableFs[pin].pinStatus = PIN_SETUP_AS_GPIO;
        }
        break;
    case MUX_UART:
        if(pinTableFs[pin].setupUart == 0xff)
        {
            returnStatus = ERROR;   // requested pin state not allowed
            break;
        }
        if(pinTableFs[pin].pinStatus & PIN_SETUP_AS_UART)
        {
            break;          // requested pin state already active
        }
        if(setupPin(pinTableFs[pin].setupUart) == OK)
        {
            pinTableFs[pin].pinStatus = PIN_SETUP_AS_UART;
        }
        break;
    case MUX_PWM:
        if(pinTableFs[pin].setupPwm == 0xff)
        {
            returnStatus = ERROR;   // requested pin state not allowed
            break;
        }
        if(pinTableFs[pin].pinStatus & PIN_SETUP_AS_PWM)
        {
            break;          // requested pin state already active
        }
        if(setupPin(pinTableFs[pin].setupPwm) == OK)
        {
            gpio_pwm_export(pinTableFs[pin].pwmPin);
            pinTableFs[pin].pinStatus = PIN_SETUP_AS_PWM;
        }
        break;
    case MUX_SPI:
        if(pinTableFs[pin].setupSpi == 0xff)
        {
            returnStatus = ERROR;   // requested pin state not allowed
            break;
        }
        if(pinTableFs[pin].pinStatus & PIN_SETUP_AS_SPI)
        {
            break;          // requested pin state already active
        }
        if(setupPin(pinTableFs[pin].setupSpi) == OK)
        {
            pinTableFs[pin].pinStatus = PIN_SETUP_AS_SPI;
        }
        break;
    case MUX_ADC:
        if(pinTableFs[pin].setupAdc == 0xff)
        {
            returnStatus = ERROR;   // requested pin state not allowed
            break;
        }
        if(pinTableFs[pin].pinStatus & PIN_SETUP_AS_ADC)
        {
            break;          // requested pin state already active
        }
        if(setupPin(pinTableFs[pin].setupAdc) == OK)
        {
            pinTableFs[pin].pinStatus = PIN_SETUP_AS_ADC;
        }
        break;
    case MUX_I2C:
        if(pinTableFs[pin].setupI2c == 0xff)
        {
            returnStatus = ERROR;   // requested pin state not allowed
            break;
        }
        if(pinTableFs[pin].pinStatus & PIN_SETUP_AS_I2C)
        {
            break;          // requested pin state already active
        }
        if(setupPin(pinTableFs[pin].setupI2c) == OK)
        {
            pinTableFs[pin].pinStatus = PIN_SETUP_AS_I2C;
        }
        break;
    }
	return returnStatus;
}

/*******************************************************************************
 * setupPin - worker function for the pinMux call, does the actual setting of a
 *            pin to a specific mode/state (uses info stored in the pinMuxFs
 *            table to set the mux state).
 *
 * Arguments: sequence - refers to an entry in the pinMuxFs table that defines
 *                  a set of actions to set the pin mux states
 *
 * Returns:   0 - Success
 *           -1 - Failure
 */
STATUS setupPin(int sequence)
{
    int vPin, vMode;

    if(pinMuxFs[sequence].enablePin1 != 0xff)
    {
        vPin = pinMuxFs[sequence].enablePin1 & 0x00ff;
        vMode = pinMuxFs[sequence].enablePin1 & 0xff00;
        if(vMode == DE)
        {
            gpio_dealloc(vPin);
        }
        else if (vMode == HI)
        {
            gpio_alloc(vPin);
            gpio_set_direction(vPin, "out");
            gpio_write_pin(vPin, "1");
        }
        else if (vMode == LO)
        {
            gpio_alloc(vPin);
            gpio_set_direction(vPin, "out");
            gpio_write_pin(vPin, "0");
        }
    }
    if(pinMuxFs[sequence].enablePin2 != 0xff)
    {
        vPin = pinMuxFs[sequence].enablePin2 & 0x00ff;
        vMode = pinMuxFs[sequence].enablePin2 & 0xff00;
        if(vMode == DE)
        {
            gpio_dealloc(vPin);
        }
        else if (vMode == HI)
        {
            gpio_alloc(vPin);
            gpio_set_direction(vPin, "out");
            gpio_write_pin(vPin, "1");
        }
        else if (vMode == LO)
        {
            gpio_alloc(vPin);
            gpio_set_direction(vPin, "out");
            gpio_write_pin(vPin, "0");
        }
    }
    return OK;
}

/*******************************************************************************
 * pinInit - initialise the Arduino pins ready to start the Arduino task
 *
 * Arguments: none
 *
 * Returns:   0 - Success
 *           -1 - Failure
 */
STATUS pinInit()
{
    /* create pseudo-interrupt enablement semaphore */
    pIntSem = semBCreate (SEM_Q_FIFO, SEM_FULL);
    pInt.interruptsEnabled = TRUE;
    pInt.interruptsAttached = 0;
    pInt.cycleTime = DEFAULT_INT_CYCLETIME;
    /* spawn the pseudo-interrupt monitor task */
    taskSpawn ((char *)"tIntMonitor", (DEFAULT_TASK_PRIORITY - 1), 0, DEFAULT_STACK_SIZE,
    		(FUNCPTR)runPseudoInterrupt, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
	return OK;
}

/*******************************************************************************
 * pinConfigDefault - set Arduino pins to a default state
 *
 * Arguments: none
 *
 * Returns:   0 - Success
 *           -1 - Failure
 */
STATUS pinConfigDefault()
{
	return OK;
}

/*******************************************************************************
 * pinMode - Arduino API call, configure a specific pin to act as either a
 *           digital input or output
 *
 * Arguments: pin  - Arduino connector pin number
 *            mode - mode to set (INPUT, OUTPUT, INPUT_PULLUP, INPUT_PULLDOWN)
 *             also...
 *              this can be used to call through to pinMux by setting a mode
 *              in the main (GPIO, UART, PWM, SPI, ADC, and I2C)
 *
 * Returns:   none
 */
void pinMode(uint8_t pin, uint8_t mode)
{
	if (mode >> 4)
	{	/* top bits set means we enter pinMux mode... */
		if(pinMux(pin, (mode & 0xf0)))
		{
			ERROR_VXGAL("[ERROR] wiring-pinMode - call to pinMux failed on pin %d\n", pin);
			return;
		}
		else
		{
			DEBUG_VXGAL("[DEBUG] wiring-pinMode - call to pinMux passed on pin %d\n", pin);
		}
	}
	if ( mode & 0x0f )
	{
        mode = (mode & 0x0f);
        if(!(pinTableFs[pin].pinStatus & GPIO_STATUS_ALLOCATED))
        {
            gpio_alloc(pinTableFs[pin].gpioPin);
            pinMux(pin, MUX_GPIO);
            if(pinTableFs[pin].directionPin != 0xff)
            {
                gpio_alloc(pinTableFs[pin].directionPin);
                gpio_set_direction(pinTableFs[pin].directionPin, "out");
            }
            pinTableFs[pin].pinStatus = (pinTableFs[pin].pinStatus | GPIO_STATUS_ALLOCATED);
        }
        if((mode == INPUT_PULLUP) || (mode == INPUT_PULLDOWN))
        {
            if(!(pinTableFs[pin].pinStatus & GPIO_STATUS_PULLUP_ALLOCATED))
            {
                gpio_alloc(pinTableFs[pin].pullupPin);
                gpio_set_direction(pinTableFs[pin].pullupPin, "out");
                pinTableFs[pin].pinStatus = (pinTableFs[pin].pinStatus | GPIO_STATUS_PULLUP_ALLOCATED);
            }
        }
    
        switch(mode)
        {
        case OUTPUT:
            /* for pintype 2 this should be ignored because they are inputs only */
            if(pinTableFs[pin].directionPin != 0xff)
            {
                gpio_write_pin(pinTableFs[pin].directionPin, "0");
            }
            gpio_set_direction(pinTableFs[pin].gpioPin, "out");
            pinTableFs[pin].pinStatus = (pinTableFs[pin].pinStatus | GPIO_STATUS_OUTPUT);
            break;
        case INPUT_PULLUP:
            gpio_write_pin(pinTableFs[pin].pullupPin, "1");
            pinTableFs[pin].pinStatus = (pinTableFs[pin].pinStatus | GPIO_STATUS_PULLUP);
            if(pinTableFs[pin].directionPin != 0xff)
            {
                gpio_write_pin(pinTableFs[pin].directionPin, "1");
            }
            gpio_set_direction(pinTableFs[pin].gpioPin, "in");
            pinTableFs[pin].pinStatus = (pinTableFs[pin].pinStatus | GPIO_STATUS_INPUT);
            break;
        case INPUT_PULLDOWN:
            gpio_write_pin(pinTableFs[pin].pullupPin, "0");
            pinTableFs[pin].pinStatus = (pinTableFs[pin].pinStatus | GPIO_STATUS_PULLDOWN);
            if(pinTableFs[pin].directionPin != 0xff)
            {
                gpio_write_pin(pinTableFs[pin].directionPin, "1");
            }
            gpio_set_direction(pinTableFs[pin].gpioPin, "in");
            pinTableFs[pin].pinStatus = (pinTableFs[pin].pinStatus | GPIO_STATUS_INPUT);
            break;
        case INPUT:
            if(pinTableFs[pin].directionPin != 0xff)
            {
                gpio_write_pin(pinTableFs[pin].directionPin, "1");
            }
            gpio_set_direction(pinTableFs[pin].gpioPin, "in");
            pinTableFs[pin].pinStatus = (pinTableFs[pin].pinStatus | GPIO_STATUS_INPUT);
            break;
        }
	}
}

/*******************************************************************************
 * digitalWrite - set the output value of a digital gpio pin
 *
 * Calls lower level gpio write function to set pin state
 *
 * Arguments: pin - Arduino connector pin number
 *            val - state to set (HIGH or LOW)
 *
 * Returns:   none
 */
void digitalWrite(register uint8_t pin, register uint8_t val)
{
	STATUS status = OK;

    if(pinTableFs[pin].pinStatus & GPIO_STATUS_OUTPUT)
    {
        /* if this pin is currently set to a digital output */
        if(val)
        {
            status = gpio_write_pin((int)pinTableFs[pin].gpioPin, "1");
        }
        else
        {
            status = gpio_write_pin((int)pinTableFs[pin].gpioPin, "0");
        }
    }
    else if(pinTableFs[pin].pinStatus & GPIO_STATUS_INPUT)
    {
        /* if this pin is currently set to a digital input */
        if(val)
        {
            pinMode(pin, INPUT_PULLUP);
        }
        else
        {
            pinMode(pin, INPUT);
        }
    }
    else if(pinTableFs[pin].pinStatus & PIN_SETUP_AS_PWM)
    {
        /* if this pin is currently set to a pwm mode then switch to dio */
        status = pinMux(pin, MUX_GPIO);
        pinMode(pin, OUTPUT);
        if(val)
        {
            status = gpio_write_pin((int)pinTableFs[pin].gpioPin, "1");
        }
        else
        {
            status = gpio_write_pin((int)pinTableFs[pin].gpioPin, "0");
        }
    }

    if (status)
    {
        ERROR_VXGAL("[ERROR] osDigitalWrite - GPIO set error on pin %d, status %d!!\n", pin, status);
    }
}

/*******************************************************************************
 * digitalRead - Arduino API call, read state of digital i/p pin
 *
 * Arguments: pin   - Arduino connector pin number
 *
 * Returns:   int  - state of pin (0 or 1), return 0 on error
 */
int digitalRead(register uint8_t pin)
{
	STATUS status = OK;
	uint8_t val;

    status = gpio_read_pin((int)pinTableFs[pin].gpioPin, &val);

    if (status)
    {
        ERROR_VXGAL("[ERROR] digitalRead - GPIO read error on pin %d, status %d!!\n", pin, status);
    }
	return (int)val;
}

/*******************************************************************************
 * analogRead - Arduino API call, read value from an analog i/p pin (resolution
 *              is set using the analogReadResolution call)
 *
 * Arguments: pin - Arduino connector pin number
 *
 * Return:    value from pin (int) - between 0 and ((2 ^ adcReadResolution) - 1)
 */
int analogRead(register uint8_t pin)
{
    if(!(pinTableFs[pin].pinStatus & PIN_SETUP_AS_ADC))
    {
        pinMux(pin, MUX_ADC);
    }
    /* simple conversion from Arduino pin to ADC number - analog pins
       start at Arduino pin 14 */
    pin = pin - 14;
    return gpio_read_adc(pin);
}

/*******************************************************************************
 * analogWrite - Arduino API call, write value to an analog pwm o/p pin
 *
 *  Note that data resolution is important here. For example, writing 800 to a
 *  pwm pin when analogWriteResolution is set to 8 bits will cause the value
 *  to "wrap around" and the pin will not output what you expect. Use the map
 *  function to avoid this when necessary.
 *
 * Arguments: pin - Arduino connector pin number
 *            value - value to write to pwm pin
 * Return:    none
 */
void analogWrite(uint32_t pin, uint32_t value)
{
    if(!(pinTableFs[pin].pinStatus & PIN_SETUP_AS_PWM))
    {
        pinMux(pin, MUX_PWM);
    }
    gpio_write_pwm(pinTableFs[pin].pwmPin, (int)value);
}

/*******************************************************************************
 * analogReference - change the analog reference on the board
 *     ***** not currently implemented *****
 *
 * Arguments: type - reference mode to set
 * Return:    none
 */
void analogReference(uint8_t type)
{
	/* not supported on Galileo Gen2 with VxWorks */
	DEBUG_VXGAL("[DEBUG] analogReference - Not currently implemented on Galileo GEN2\n");
	return;
}

/*******************************************************************************
 * analogReadResolution - Arduino API call, set adc (read) resolution to a
 *              specific number of bits
 *
 * Arguments: int, number of bits of resolution.
 *            Arduino default is 10 bit (set at initialisation time)
 *            Note that Galileo Gen2 hardware always returns 12 bit result.
 *
 * Return:    none
 */
void analogReadResolution(int numBits)
{
	if(numBits < ANALOG_READ_RESOLUTION_MIN)
	{
		numBits = ANALOG_READ_RESOLUTION_MIN;
	}
	else if(numBits > ANALOG_READ_RESOLUTION_MAX)
	{
		numBits = ANALOG_READ_RESOLUTION_MAX;
	}
	adcReadResolution = numBits;
}

/*******************************************************************************
 * analogWriteResolution - Arduino API call, set pwm (write) resolution to a
 *              specific number of bits
 *
 * Arguments: int, number of bits of resolution.
 *            Arduino default is 8 bit (set at initialisation time)
 *
 * Return:    none
 */
void analogWriteResolution(int numBits) {
	if(numBits < ANALOG_WRITE_RESOLUTION_MIN)
	{
		numBits = ANALOG_WRITE_RESOLUTION_MIN;
	}
	else if(numBits > ANALOG_WRITE_RESOLUTION_MAX)
	{
		numBits = ANALOG_WRITE_RESOLUTION_MAX;
	}
	pwmWriteResolution = numBits;
}

/*******************************************************************************
 * attachInterruptSource - attach and enable a pseudo interrupt (polled mode)
 *                         to a digital I/O pin
 *
 * Arguments: pin - Arduino connector pin number
 *            callback - callback routine when interrupt is triggered
 *            mode - interrupt mode
 *
 * Returns: status (OK)
 */
STATUS attachInterruptSource(uint32_t pin, void (*callback)(void), uint32_t mode)
{
    if(pInt.interruptsAttached > 3) return ERROR;  /* both ints being used */
    if(!(pInt.interruptsAttached & 0x01))
    {   /* int1 is empty, use it... */
        pInt.pin1 = pin;
        pInt.callback1 = callback;
        pInt.mode1 = mode;
        pInt.fd1 = gpio_fd_open((int)pinTableFs[pInt.pin1].gpioPin);
        pInt.psstate1 = gpio_fd_read_pin(pInt.fd1);
        pInt.interruptsAttached = pInt.interruptsAttached | 0x01;
    	DEBUG_VXGAL("[DEBUG] attachInterruptSource - pin = %d, mode = %d, attached to pInt.1\n", pin, mode);
    }
    else if (!(pInt.interruptsAttached & 0x02))
    {   /* int2 is empty, use it... */
        pInt.pin2 = pin;
        pInt.callback2 = callback;
        pInt.mode2 = mode;
        pInt.fd2 = gpio_fd_open((int)pinTableFs[pInt.pin2].gpioPin);
        pInt.psstate2 = gpio_fd_read_pin(pInt.fd2);
        pInt.interruptsAttached = pInt.interruptsAttached | 0x02;
    	DEBUG_VXGAL("[DEBUG] attachInterruptSource - pin = %d, mode = %d, attached to pInt.2\n", pin, mode);
    }
    return OK;
}

/*******************************************************************************
 * runPseudoInterrupt - routine (run as task) that polls a specific pin to
 *                      monitor for a pseudo interrupt. Uses information in
 *                      pInt structure.
 *
 * Arguments: none
 *
 * Returns: none
 */
void runPseudoInterrupt(void)
{
    uint8_t rval = 0;

    while (1)
    {
        taskDelay(pInt.cycleTime);
        semTake(pIntSem, WAIT_FOREVER);

        if(pInt.interruptsAttached & 0x01)
        {
            rval = gpio_fd_read_pin(pInt.fd1);
            if(rval != pInt.psstate1)
            {
                if(pInt.mode1 == CHANGE)
                {
                	DEBUG_VXGAL("[DEBUG] runPseudoInterrupt - int1 triggered, mode = CHANGE\n");
                    pInt.callback1();
                }
                if(rval && (pInt.mode1 == RISING))
                {
                	DEBUG_VXGAL("[DEBUG] runPseudoInterrupt - int1 triggered, mode = RISING\n");
                    pInt.callback1();
                }
                if(!rval && (pInt.mode1 == FALLING))
                {
                	DEBUG_VXGAL("[DEBUG] runPseudoInterrupt - int1 triggered, mode = FALLING\n");
                    pInt.callback1();
                }
                pInt.psstate1 = rval;
            }
            else
            {
                if(rval && (pInt.mode1 == HIGH))
                {
                	DEBUG_VXGAL("[DEBUG] runPseudoInterrupt - int1 triggered, mode = HIGH\n");
                    pInt.callback1();
                }
                if(!rval && (pInt.mode1 == LOW))
                {
                	DEBUG_VXGAL("[DEBUG] runPseudoInterrupt - int1 triggered, mode = LOW\n");
                    pInt.callback1();
                }
            }
        }
        if(pInt.interruptsAttached & 0x02)
        {
            rval = gpio_fd_read_pin(pInt.fd2);
            if(rval != pInt.psstate2)
            {
                if(pInt.mode2 == CHANGE)
                {
                	DEBUG_VXGAL("[DEBUG] runPseudoInterrupt - int2 triggered, mode = CHANGE\n");
                    pInt.callback2();
                }
                if(rval && (pInt.mode2 == RISING))
                {
                	DEBUG_VXGAL("[DEBUG] runPseudoInterrupt - int2 triggered, mode = RISING\n");
                    pInt.callback2();
                }
                if(!rval && (pInt.mode2 == FALLING))
                {
                	DEBUG_VXGAL("[DEBUG] runPseudoInterrupt - int2 triggered, mode = FALLING\n");
                    pInt.callback2();
                }
                pInt.psstate2 = rval;
            }
            else
            {
                if(rval && (pInt.mode2 == HIGH))
                {
                	DEBUG_VXGAL("[DEBUG] runPseudoInterrupt - int2 triggered, mode = HIGH\n");
                    pInt.callback2();
                }
                if(!rval && (pInt.mode2 == LOW))
                {
                	DEBUG_VXGAL("[DEBUG] runPseudoInterrupt - int2 triggered, mode = LOW\n");
                    pInt.callback2();
                }
            }
        }
        semGive(pIntSem);
    }
    return;
}

/*******************************************************************************
* detachInterruptSource - Detach interrupt - modify the interrupt structure
*                     to remove a pin from monitoring.
*
* Arguments: pin (int)
*
* Returns: void
*/
void detachInterruptSource(uint32_t pin)
{
    if(pInt.pin1 == pin)
    {   /* detach this pin */
        pInt.interruptsAttached = pInt.interruptsAttached & 0x02;
    	gpio_fd_close(pInt.fd1);
        pInt.pin1 = 0;
        pInt.callback1 = NULL;
        pInt.mode1 = 0;
        pInt.fd1 = 0;
        pInt.psstate1 = 0;
    	DEBUG_VXGAL("[DEBUG] detachInterruptSource - pInt.1 (pin %d) detached\n", pin);
    	return;
    }
    if (pInt.pin2 == pin)
    {   /* detach this pin */
        pInt.interruptsAttached = pInt.interruptsAttached & 0x01;
    	gpio_fd_close(pInt.fd2);
        pInt.pin2 = 0;
        pInt.callback2 = NULL;
        pInt.mode2 = 0;
        pInt.fd2 = 0;
        pInt.psstate2 = 0;
    	DEBUG_VXGAL("[DEBUG] detachInterruptSource - pInt.2 (pin %d) detached\n", pin);
    	return;
    }
    /* if we get here then the pin specified is not connected to an interrupt */
    return;
}

/*******************************************************************************
* interruptsEnabled - Either enable or disable interrupts. This uses a
*                     semaphore to cause the the monitor task
*                     (runPseudoInterrupt) to pend.
*
* Arguments: state (int) - Enable (TRUE) or disable (FALSE) interrupts
*
* Returns: void
*/
void interruptsEnabled(int state)
{
    if(!state && pInt.interruptsEnabled)
    {     /* request is to disable interrupts and they are currently enabled... */
        pInt.interruptsEnabled = FALSE;
        semTake(pIntSem, WAIT_FOREVER);
    	DEBUG_VXGAL("[DEBUG] interruptsEnabled - interrupts disabled\n");
    }
    else if (state && !pInt.interruptsEnabled)
    {     /* request is to enable interrupts and they are currently disabled... */
        pInt.interruptsEnabled = TRUE;
        semGive(pIntSem);
    	DEBUG_VXGAL("[DEBUG] interruptsEnabled - interrupts enabled\n");
    }
}

/*******************************************************************************
* usleep - Sleep/delay for the requested number of uSec.
*
* Check that the arguments are valid and use clock_nanosleep() to implement
* the uSec sleep.
*
* Arguments: microseconds - Number if uSec to sleep/delay for
*
* Returns: void
*/
void usleep (long microseconds)
{
    struct timespec t1, t2;
    DEBUG_VXGAL("[DEBUG] usleep - delay start - %d\n", tickGet());

    if (microseconds == 0)
    {   /* Nothing to do */
        return ;
    }
    DEBUG_VXGAL("[DEBUG] usleep - delay for %d uS\n",(int)microseconds);

    /* Handle situations where called with > 999,999 microseconds. */
    t1.tv_sec = t1.tv_nsec = 0;
    t1.tv_sec = (time_t)(microseconds / 1000000);
    t1.tv_nsec = (long)((microseconds % 1000000) * 1000);
    t2.tv_sec = 0;
    t2.tv_nsec = 0;

    (void)clock_nanosleep(CLOCK_REALTIME, 0, &t1, &t2);
    DEBUG_VXGAL("[DEBUG] usleep - delay end - %d\n", tickGet());
    return;
}
