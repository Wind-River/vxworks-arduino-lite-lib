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
 *  - serial.cpp
 * Serial API source.
 */

#include <stdio.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <ioLib.h>
#include <unistd.h>
#include <i2c.h>
#include <assert.h>

#include <Arduino-lite.h>
#include "serial.h"

/* ************* Below this line are C++ classes and functions ************** */

using namespace std;

serial Serial;

/* ***** Serial *****
 * Using for communication between the user and the Galileo Gen2 board. These
 * functions currently assume stdin/stdout - which in debug configuration is
 * the debug terminal within the Helix App Cloud.
 */
serial::serial()
{
  intval = 0;
}

/*******************************************************************************
 * begin(int) - prepare serial port for use
 * 
 * Arguments: int, baud rate - not used on VxWorks/Galileo Gen2 implementation
 * 
 * Returns:   none
 */
void serial::begin(int baud)
{
	/* currently does nothing in context of Galileo Gen2 debugging */
}

/*******************************************************************************
 * available() - return numbers of characters available to read from stdin
 * 
 * Arguments: none
 * 
 * Returns:  int, number of chars available
 */
int serial::available()
{
    int numToRead,rc;
    /* probe STDIN to see if operator entered any characters */
    rc = ioctl(0, FIONREAD, &numToRead);
    assert(rc != ERROR);
	return numToRead;
}

/*******************************************************************************
 * read() - get one character from stdin
 * 
 * Arguments: none
 * 
 * Returns:  int, character read from input (in integer form)
 */
int serial::read()
{
	return (int)getc(stdin);
}

/*******************************************************************************
 * readString() - read a string from stdin
 * 
 * Arguments: none
 * 
 * Returns:  char * - to returned string (max length of 80)
 */
char * serial::readString()
{
	memset(thisText,0,strlen(thisText));
    fgets(thisText, MAX_READSTRING_LEN - 1, stdin);
    return thisText;
}

/*******************************************************************************
 * println(int) - print an integer (plus linefeed) to stdio
 * 
 * Arguments: int, integer to be printed
 * 
 * Returns:  int, number of chars printed
 */
int serial::println ( int q )
{
	int rv = 0;
	rv += print(q);
	return rv + printf("\n");
}

/*******************************************************************************
 * println(int,int) - print an integer in the specified number base (plus 
 *                    linefeed) to stdio
 * 
 * Arguments: int, integer to be printed
 *            int, numberbase
 * 
 * Returns:  int, number of chars printed
 */
int serial::println ( int q, int nbase )
{
	int rv = 0;
	rv += print(q, nbase);
	return rv + printf("\n");
}

/*******************************************************************************
 * println(double) - print a double (plus linefeed) to stdio
 * 
 * Arguments: double, integer to be printed
 * 
 * Returns:  int, number of chars printed
 */
int serial::println ( double q )
{
	int rv = 0;
	rv += print(q);
	return rv + printf("\n");
}

/*******************************************************************************
 * println(double,int) - print a double to a specified number of places (plus 
 *                       linefeed) to stdio
 * 
 * Arguments: double, number to be printed
 *            int, number of decimal places
 * 
 * Returns:  int, number of chars printed
 */
int serial::println ( double q, int places )
{
	int rv = 0;
	rv += print(q, places);
	return rv + printf("\n");
}

/*******************************************************************************
 * println(char) - print a character (plus linefeed) to stdio
 * 
 * Arguments: char, character to be printed
 * 
 * Returns:  int, number of chars printed
 */
int serial::println ( char q )
{
	int rv = 0;
	rv += print(q);
	return rv + printf("\n");
}

/*******************************************************************************
 * println(int) - print a character string (plus linefeed) to stdio
 * 
 * Arguments: char *, string to be printed
 * 
 * Returns:  int, number of chars printed
 */
int serial::println ( char *q )
{
	int rv = 0;
	rv += print(q);
	return rv + printf("\n");
}

/*******************************************************************************
 * println(int,int) - print an integer in the specified number base to stdio
 * 
 * Arguments: int, integer to be printed
 *            int, numberbase
 * 
 * Returns:  int, number of chars printed
 */
int serial::print ( int p, int nbase )
{
	switch (nbase) {
	case HEX:
		return printf("%X", p);
		break;
	case OCT:
		return printf("%o", p);
		break;
	case BIN:
		/* method from http://stackoverflow.com/questions/111928/is-there-a-printf-converter-to-print-in-binary-format */
		static char b[33];
		b[32] = '\0';
		for (int z = 0; z < 32; z++) {
			b[31-z] = ((p>>z) & 0x1) ? '1' : '0';
		}
		return printf("%s", b);
		break;
	default:
		return printf("%d", p);
		break;
	}
}
