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
 *  - wire.cpp
 * Wire Library for communicating with I2C
 */

#include <stdio.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <ioLib.h>
#include <unistd.h>
#include <i2c.h>

#include <Arduino-lite.h>
#include <Wire.h>

/* ******************************************************************* */
/* Add these defines to enable debug and/or error message printing
 * Messages take the form...
 *        DEBUG_WIRE("Debug data is %d\n", value);
 *        ERROR_WIRE("Error message: %.2d\n", floatvalue);
*/
#define ENABLE_ERROR_WIRE
// #define ENABLE_DEBUG_WIRE

#ifdef ENABLE_ERROR_WIRE
#define ERROR_WIRE(...) fprintf (stderr, __VA_ARGS__)
#else
#define ERROR_WIRE(...)
#endif
#ifdef ENABLE_DEBUG_WIRE
#define DEBUG_WIRE(...) fprintf (stderr, __VA_ARGS__)
#else
#define DEBUG_WIRE(...)
#endif
/* ******************************************************************* */

/* The following instantiation is here because, by default, the Arduino
 * environment instantiates the Wire class at startup. Comment this out 
 * if this behaviour is not required.
 */
wire Wire;

/*******************************************************************************
 * Wire library constructor
 *
 * Arguments: none
 * Returns:  none
 */
wire::wire()
{
	transmitAddr = 0;
	transmit_fd = -1;
	byteTransmitCount = 0;
	memset(transmitData, 0, TRANSMIT_DATA_SIZE);
	readAddr = 0;
	read_fd = -1;
	readCount = 0;
	currentReadByte = 0;
	bufferedBytes = 0;
	memset(readData, 0, READ_DATA_SIZE);
}

/*******************************************************************************
 * Wire.begin() - Initiate the Wire library and join the I2C bus as a master.
 *                This should normally be called only once.
 *
 * Arguments: join the i2c bus as a master
 *   -- the "slave device" option for this function is not yet supported ---
 *
 * Returns:  none
 *
 * Syntax:   begin()
 */
void wire::begin ( void )
{
	// wire:begin -- currently does nothing
}

/*******************************************************************************
 * Wire.beginTransmission() - Begin a transmission to the I2C slave device
 *                            with the given address
 *
 * Arguments: address: the 7-bit address of the device to transmit to
 *
 * Returns:  none
 *
 * Syntax:   beginTransmission(address)
 */
void wire::beginTransmission ( uint8_t devaddr )
{
	DEBUG_WIRE("[DEBUG] wire::beginTransmission(devaddr = 0x%x)\n", transmitAddr);
	if((transmitAddr != 0) && (transmit_fd > 0))
	{
	    (void)close(transmit_fd);
	    transmitAddr = 0;
	    transmit_fd = -1;
		byteTransmitCount = 0;
		memset(transmitData, 0, TRANSMIT_DATA_SIZE);
	}
    transmit_fd = open("/dev/i2c-0", O_RDWR);
    if (transmit_fd == -1)
    {
        ERROR_WIRE("[ERROR] Error opening I2C Bus\n");
        return;
    }
    if (ioctl(transmit_fd, I2C_SLAVE_ADDR, devaddr) < 0)
    {
        ERROR_WIRE("[ERROR] Error setting LCD I2C address to 0x%x\n", devaddr);
        return;
    }
	transmitAddr = devaddr;
}

/*******************************************************************************
 * Wire.write()  - write single byte, actually queues bytes ready for 
 *                 transmission, used in-between calls to beginTransmission() &
 *                 endTransmission()
 *
 * Arguments:  value: a value to send as a single byte
 *
 * Returns:  byte: number of bytes written, though reading return is optional
 *
 * Syntax:   Wire.write(value)
 */
int wire::write ( uint8_t bvalue )
{
	transmitData[byteTransmitCount++] = bvalue;
	return 1;
}

/*******************************************************************************
 * Wire.write()  - write an array of data (bytes)
 *
 * Arguments:  data: an array of data to send as bytes
 *             length: the number of bytes to transmit
 *
 * Returns:  byte: number of bytes written, though reading return is optional
 *
 * Syntax:   Wire.write(data, length)
 */
int wire::write ( uint8_t *barray, int bnum )
{
	int x = 0;
	while(x < bnum)
	{
		transmitData[byteTransmitCount++] = barray[x++];
	}
	return x;
}

/*******************************************************************************
 * Wire.write()  - write a string (of bytes)
 *
 * Arguments:  string: a string to send as a series of bytes
 *
 * Returns:  byte: number of bytes written, though reading return is optional
 *
 * Syntax:   Wire.write(string)
 */
int wire::write ( char *strdata )
{
	int x = 0;
	while(strdata[x] != 0)
	{
		transmitData[byteTransmitCount++] = strdata[x++];
	}
	return x;
}

/*******************************************************************************
 * Wire.endTransmission()  - Ends a transmission to a slave device that was begun
 *                           by beginTransmission() and transmits the bytes that
 *                           were queued by write()
 *
 * Arguments: stop - Boolean
 *                   True will send a stop message, releasing the bus after
 *                   transmission.
 *                   False will send a restart, keeping the connection active
 *  *** note: false (restart) is not yet implemented
 *  *** a parameter greater than 1 will be assumed to be an i2c address and
 *      selects which fd to send to.
 *
 * Returns:  byte, which indicates the status of the transmission:
 *           0:  success
 *           1:  data too long to fit in transmit buffer
 *           2:  received NACK on transmit of address
 *           3:  received NACK on transmit of data
 *           4:  other error
 *
 * Syntax:   endTransmission()
 *           endTransmission(address)
 */
void wire::endTransmission ( int mode )
{
	switch(mode)
	{
	case 0:
	case 1:
	    if (::write(transmit_fd, transmitData, byteTransmitCount) != byteTransmitCount)
	    {
	        ERROR_WIRE("[ERROR] Error sending data to I2C device\n");
	    }
		break;
	default:
	    if (::write(mode, transmitData, byteTransmitCount) != byteTransmitCount)
	    {
	        ERROR_WIRE("[ERROR] Error sending I2C data to Grove LCD display\n");
	    }
		break;
	}
    byteTransmitCount = 0;
	memset(transmitData, 0, TRANSMIT_DATA_SIZE);
}

/*******************************************************************************
 * Wire.requestFrom()  - Used by the master to request bytes from a slave
 *                       device. The bytes may then be retrieved with the
 *                       available() and read() functions.
 *
 * Arguments: address: the 7-bit address of the device to request bytes from
 *            quantity: the number of bytes to request
 *            stop : boolean. true will send a stop message after the request,
 *                         releasing the bus. false will continually send a
 *                         restart after the request, keeping the connection
 *                         active.
 *
 *  *** note: stop is not yet implemented - true assumed
 *
 * Returns:  byte, the number of bytes returned from the slave device
 *
 * Syntax:   requestFrom(address, quantity)
 *           requestFrom(address, quantity, stop)
 *
 * Note: in this implementation we use ::read to load bytes into the readData
 *       buffer which is then unloaded one byte at a time using the wire::read
 *       function. The number of bytes remaining in the buffer can be found
 *       by using the wire::available function.
 */
int wire::requestFrom ( uint8_t devaddr, int numbytes = 1 )
{
	/* if we are NOT already connected to the required device... */
	if(readAddr != devaddr)
	{
		/* if we are already connected to a device then disconnect and restart */
		if((readAddr != 0) && (read_fd > 0))
		{
		    (void)close(read_fd);
		    readAddr = 0;
		    read_fd = -1;
			readCount = 0;
			currentReadByte = 0;
		}
	    read_fd = open("/dev/i2c-0", O_RDWR);
	    if (read_fd == -1)
	    {
	        ERROR_WIRE("[ERROR] Error opening I2C Bus for reading\n");
	        return -1;
	    }
	    if (ioctl(read_fd, I2C_SLAVE_ADDR, devaddr) < 0)
	    {
	        ERROR_WIRE("[ERROR] Error setting LCD I2C address for reading\n");
	        return -1;
	    }
		readAddr = devaddr;
	}
	bufferedBytes = ::read(read_fd, readData, numbytes); /* readCount?? */
	readCount = numbytes;   /* not sure if this is needed */
	return bufferedBytes;
}

/*******************************************************************************
 * Wire.read()  - Reads a byte that was transmitted from a slave device to a
 *                master after a call to requestFrom()
 *
 * Arguments: none
 *
 * Returns:  unsigned char, the next byte received
 *
 * Note: in this implementation this function returns a byte from the readData
 *       buffer. If it's empty then we return the value DEFAULT_EMPTYBUFFER_VALUE.
 *       We could 'reload' the buffer by using the requestFrom function again
 *       but that's not included in this code. The number of bytes remaining in 
 *       the buffer can be found by using the wire::available function.
 */
uint8_t wire::read( void )
{
	if(bufferedBytes)
	{
		currentReadByte++;
		bufferedBytes--;
		return readData[currentReadByte - 1];
	}
	return DEFAULT_EMPTYBUFFER_VALUE;
}

/*******************************************************************************
 * Wire.available()  - returns the number of bytes available for reading with
 *                     wire::read.
 *
 * Arguments: none
 *
 * Returns:  int, the number of bytes available
 */
int wire::available ( void )
{
	/* if bufferedBytes is zero then maybe we should try reading again !! */
	return bufferedBytes;
}
