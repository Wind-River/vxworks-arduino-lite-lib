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
 *  - Extras.c
 * Library for various additional routines
 */

#include <stdio.h>
#include <signal.h>
#include <net/if.h>
#include <ipnet/ipioctl.h>
#include <ioLib.h>
#include <sockLib.h>
#include <vxWorks.h>

#include <Arduino-lite.h>
#include <Extras.h>

/* ******************************************************************* */
/* Add these defines to enable debug and/or error message printing
 * Messages take the form...
 *        DEBUG_EXTRAS("Debug data is %d\n", value);
 *        ERROR_EXTRAS("Error message: %.2d\n", floatvalue);
*/
#define ENABLE_ERROR_EXTRAS
/* #define ENABLE_DEBUG_EXTRAS */

#ifdef ENABLE_ERROR_EXTRAS
#define ERROR_EXTRAS(...) fprintf (stderr, __VA_ARGS__)
#else
#define ERROR_EXTRAS(...)
#endif
#ifdef ENABLE_DEBUG_EXTRAS
#define DEBUG_EXTRAS(...) fprintf (stderr, __VA_ARGS__)
#else
#define DEBUG_EXTRAS(...)
#endif
/* ******************************************************************* */


/*******************************************************************************
 * getMacAddress - find mac address of board and store it in macAddress
 *                 protected variable
 *
 * Arguments: none
 *
 * Returns:  int, SUCCESS or FAILURE
 */
int getMacAddress(char *macAddr)
{
	int fd;
	struct ifreq req;

	/* get MAC from first interface */
	fd = socket( AF_INET, SOCK_RAW, 0 );
	if(fd < 0)
	{
	    ERROR_EXTRAS("[ERROR] extras-getMacAddress - socket() error = %d\n", fd);
		return -1;
	}

	memset((char *)&req, 0, sizeof(req));
	strncpy( req.ifr_name, "qrkgmac0", strlen("qrkgmac0") );
	if(ioctl( fd, SIOCGIFLLADDR, &req ) < 0)
	{
	    ERROR_EXTRAS("[ERROR] extras-getMacAddress - ioctl() error\n");
		return -1;
	}

	/* MAC is now in req.ifr_addr.sa_data[] put it in string format */
	sprintf(macAddr, "%02x%02x%02x%02x%02x%02x", \
		(unsigned char)req.ifr_addr.sa_data[0], (unsigned char)req.ifr_addr.sa_data[1], \
		(unsigned char)req.ifr_addr.sa_data[2], (unsigned char)req.ifr_addr.sa_data[3], \
		(unsigned char)req.ifr_addr.sa_data[4], (unsigned char)req.ifr_addr.sa_data[5]);

	return(0);
}

/*******************************************************************************
 * groveReadTemp - read a temp sensor and convert to degrees C
 * 
 * Arguments: aline, the analog input on which the sensor is connected
 *            *result, a pointer to a double used for storing the result
 * 
 * Returns:  none
 */
void groveReadTemp ( uint8_t aline, double *result) {
    double resistance;
    int a;
    int B=3975;                  // B value of the thermistor
    
    a = analogRead(aline);
    resistance = (float)(1023-a)*10000/a; 
    *result = 1/(log(resistance/10000)/B+1/298.15)-273.15;
}

/*******************************************************************************
 * groveReadLux - read a light sensor and convert to lux
 *     Note - this is an uncalibrated and very approximate conversion
 * 
 * Arguments: rline, the analog input on which the sensor is connected
 *            *result, a pointer to a double used for storing the result
 * 
 * Returns:  none
 */
void groveReadLux ( uint8_t rline, double *result) {
    int a;
    float b = 2.4;
    float c = 4;

    a = analogRead(rline);
    /* *** approximate conversion to lux *** */
    *result = pow(10, ( c * ( log10((double)a) - b ) ));
}
