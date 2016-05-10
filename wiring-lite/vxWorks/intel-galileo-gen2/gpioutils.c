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
 *  - gpioutils.c
 * A set of useful utilities to work with GPIO through the sysfs interface
 */

#include <vxWorks.h>
#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>

#include "gpioutils.h"

int adcReadResolution = DEFAULT_ADC_RESOLUTION;
int pwmWriteResolution = DEFAULT_PWM_RESOLUTION;
int pwmPeriod = DEFAULT_PWM_PERIOD;

/*******************************************************************************
 * gpio_alloc - Allocate/reserve a GPIO pin through the sysfs interface
 *
 * Open /sys/class/gpio/export, write the pin number (in ASCII) to the file 
 * descriptor, and close the file descriptor
 *
 * Arguments: gpionum - Pin number to allocate/reserve
 *
 * Returns:   0 - OK
 *           -1 - ERROR
 */
int gpio_alloc(int gpionum)
{
    int fd;
    char buffer[FD_BUFFER_SZ];
    int  bytes;
    
    /* Open the sysfs interface for exporting/allocating a GPIO pin  */
    fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd == -1)
        {
        fprintf(stderr, "Error opening /sys/class/gpio/export, errno = %d\n", errno);
        return (ERROR);
        }
       
    /* Write the GPIO pin number to allocate/reserve (in ASCII) */ 
    sprintf(buffer, "%d", gpionum);

    bytes = write(fd, buffer, strlen(buffer));     
    if (bytes != strlen(buffer))
        {
        fprintf(stderr, "Error allocating GPIO %d errno=%d\n", gpionum, errno);
        return(ERROR);
        }
       
    /* Close the sysfs interface */ 
    (void)close(fd);
    
    return(OK);    
}

/*******************************************************************************
 * gpio_dealloc - deallocate/unreserve a GPIO pin through the sysfs interface
 *
 * Open /sys/class/gpio/unexport, write the pin number (in ASCII) to the file 
 * descriptor, and close the file descriptor
 *
 * Arguments: gpionum - Pin number to allocate/reserve
 *
 * Returns:  0 - OK
 *           -1 - ERROR
 */
int gpio_dealloc(int gpionum)
{
    int fd;
    char buffer[FD_BUFFER_SZ];
    int  bytes;
    
    /* Open the sysfs interface for deallocating/unreserving a GPIO pin  */
    fd = open("/sys/class/gpio/unexport", O_WRONLY);
    if (fd == -1)
        {
        fprintf(stderr, "Error opening /sys/class/gpio/unexport, errno = %d\n", errno);
        return (ERROR);
        }

    /* Write the GPIO pin number to deallocate/unreserve (in ASCII) */        
    sprintf(buffer, "%d", gpionum);

    bytes = write(fd, buffer, strlen(buffer));     
    if (bytes != strlen(buffer))
        {
        fprintf(stderr, "Error deallocating GPIO %d errno=%d\n", gpionum, errno);
        return(ERROR);
        }
       
    /* Close the sysfs interface */ 
    (void)close(fd);
    
    return(OK);    
}

/*******************************************************************************
 * gpio_set_direction - Set the direction (input/output) of a GPIO pin
 *
 * Open the appropriate sysfs file, write the direction, close the file 
 * descriptor
 *
 * Arguments: gpionum - Pin number to allocate/reserve
 *            direction - "in" or "out"
 *
 * Returns:   0 - OK
 *           -1 - ERROR
 */
int gpio_set_direction(int gpionum, char *direction)
{
    int fd;
    char buffer[FD_BUFFER_SZ];
    int  bytes;

    /* Open the appropriate sysfs interface */        
    sprintf(buffer, "/sys/class/gpio/gpio%d/direction", gpionum);    
        
    fd = open(buffer, O_WRONLY);
    if (fd == -1)
        {
        fprintf(stderr, "Error opening %s, errno = %d\n", buffer, errno);
        return (ERROR);
        }

    /* Write the direction (has to be in ASCII) */
    bytes = write(fd, direction, strlen(direction));
    if (bytes != strlen(direction))
        {
        fprintf(stderr, "Error setting GPIO %d as %s errno=%d\n", gpionum, direction, errno);
        return (ERROR);
        }

    /* Close the sysfs interface */
    (void) close(fd);
    
    return(OK);
}

/* -------------------------------------------------------------------------- */


/*******************************************************************************
 * gpio_write_pin - Write a value to a GPIO pin
 *
 * Open the appropriate sysfs file, write the data, close the file descriptor
 *
 * Arguments: gpionum - Pin number to allocate/reserve
 *            value - "0" or "1"
 *
 * Returns:   0 - OK
 *           -1 - ERROR
 */
int gpio_write_pin(int gpionum, char *value)
{
    int fd;
    char buffer[FD_BUFFER_SZ];
    int  bytes;

    /* Open the appropriate sysfs interface */ 
    sprintf(buffer, "/sys/class/gpio/gpio%d/value", gpionum);
    fd = open(buffer, O_WRONLY);
    if (fd == -1)
        {
        fprintf(stderr, "Error opening %s, errno = %d\n", buffer, errno);
        return (ERROR);
        }

    /* Write the value (has to be in ASCII) */
    bytes = write(fd, value, strlen(value));
    if (bytes != strlen(value))
        {
        fprintf(stderr, "Error setting GPIO %d to %s errno=%d\n", gpionum, value, errno);
        return (ERROR);
        }
       
    /* Close the sysfs interface */ 
    (void) close(fd);
    
    return (OK);
}

/*******************************************************************************
 * gpio_read_pin - Read a value from a GPIO pin
 *
 * Open the appropriate sysfs file, read the data, close the file descriptor
 *
 * Arguments: gpionum - GPIO pin number to read (linux number - NOT pin number)
 *
 * Returns:  Value of pin (*uint8_t) = 0 or 1 
 *           Status (OK=0,  ERROR=-1)
 */
int gpio_read_pin(uint8_t gpionum, uint8_t* val)
{
    int fd;
    char buffer[FD_BUFFER_SZ];
    int  bytes;
    char byteread[DT_BUFFER_SZ];

    *val = 0;

    /* Clear the data buffer */
    memset(byteread, 0, DT_BUFFER_SZ);
    
    /* Open the appropriate sysfs interface */ 
    sprintf(buffer, "/sys/class/gpio/gpio%d/value", gpionum);    
    fd = open(buffer, O_RDONLY);
    if (fd == -1)
        {
        fprintf(stderr, "Error opening %s, errno = %d\n", buffer, errno);
        return (ERROR);
        }

    /* Read the value (will be in ASCII) */
    bytes = read(fd, byteread, 2);
    if (bytes < 0)
        {
        fprintf(stderr, "Error reading GPIO %d - bytes read %d, error = %d\n", gpionum, bytes, errno);
        return (ERROR);
        }
       
    /* Close the sysfs interface */ 
    (void) close(fd);

    *val = ((uint8_t)byteread[0] & 0x01);
    return OK;
}

/*******************************************************************************
 * gpio_read_adc - Read a value from an ADC input pin
 *
 * Open the appropriate sysfs file, read the data, close the file descriptor
 *
 * Arguments: adcnum - ADC pin to read (adc input pin - NOT physical pin number)
 *
 * Returns:  Scaled value from pin (int)
 */
int gpio_read_adc(uint8_t adcnum)
{
    char fdBuffer[FD_BUFFER_SZ];
    char dtBuffer[DT_BUFFER_SZ];
    int fd;
    int value = 9999;

    /* Clear the data buffer */
    memset(dtBuffer, 0, 64);
    
    /* Open the ADC device 0 */
    sprintf(fdBuffer, "/sys/bus/iio/devices/iio:device0/in_voltage%d_raw", adcnum);    
    fd = open(fdBuffer, O_RDONLY, 0664);
    if (fd < 0)
        {
        fprintf (stderr, "ERROR: iio open failed! errno = %d\n", errno);
        return 9999;
        }

    /* Read up to 5 bytes of data */
    read(fd, dtBuffer, 5*sizeof(char));

    /* Convert the buffer to a number */
    if (sscanf(dtBuffer, "%d", &value) != 1)
        {
        fprintf (stderr, "Error reading voltage %s\n", dtBuffer);
        }

    /* Close the sysfs interface */ 
    close (fd);

    /* Scale the output according to adcReadResolution */
    /* The device always returns 12-bit data - scaling is done in code here */
    if(adcReadResolution > 12) 
        {
        return (value << (adcReadResolution - 12));
        } 
    else if (adcReadResolution < 12) 
        {
        return (value >> (12 - adcReadResolution));
        } 
    else 
        {
        return value;
        }
}

/*******************************************************************************
 * gpio_pwm_export - Export and prepare a PWM pin through the sysfs interface
 *
 * Open /sys/class/pwm/pwmchip0/export, export the pin, set the period, 
 * enable the pin, and close the file descriptor
 *
 * Arguments: pwmnum - PWM to prepare (pwm number, NOT the physical pin num)
 *
 * Returns:   0 - OK
 *           -1 - ERROR
 */
int gpio_pwm_export(int pwmnum)
{
    int fd;
    char buffer[FD_BUFFER_SZ];
    char buf2[DT_BUFFER_SZ];
    int  bytes;
    
    /* Open the pwm device through the sysfs interface */
    fd = open("/sys/class/pwm/pwmchip0/export", O_WRONLY);
    if (fd == -1)
        {
        fprintf(stderr, "Error opening /sys/class/pwm/pwmchip0/export, errno = %d\n", errno);
        return (ERROR);
        }

    /* Write the PWM pin number to allocate/reserve (in ASCII) */ 
    sprintf(buffer, "%d", pwmnum);
    bytes = write(fd, buffer, strlen(buffer));     
    if (bytes != strlen(buffer))
        {
        fprintf(stderr, "Error allocating GPIO %d errno=%d\n", pwmnum, errno);
        return(ERROR);
        }

    /* Close the sysfs interface */ 
    (void)close(fd);

    /* ** Set the period of a PWM pin ** */

    /* Open the pwm device through the sysfs interface */
    sprintf(buffer, "/sys/class/pwm/pwmchip0/pwm%d/period", pwmnum);    
    fd = open(buffer, O_WRONLY);
    if (fd == -1)
        {
        fprintf(stderr, "Error opening /sys/class/pwm/pwmchip0/pwm%d/period, errno = %d\n", pwmnum, errno);
        return (ERROR);
        }

    /* Write the PWM period (in ASCII) */ 
    sprintf(buf2, "%d", pwmPeriod);
    bytes = write(fd, buf2, strlen(buf2));     
    if (bytes != strlen(buf2))
        {
        fprintf(stderr, "Error writing period to PWM %d errno=%d\n", pwmnum, errno);
        return(ERROR);
        }

    /* Close the sysfs interface */ 
    (void)close(fd);

    /* ** Enable the PWM pin ** */

    /* Open the pwm device through the sysfs interface */
    sprintf(buffer, "/sys/class/pwm/pwmchip0/pwm%d/enable", pwmnum);    
    fd = open(buffer, O_WRONLY);
    if (fd == -1)
        {
        fprintf(stderr, "Error opening /sys/class/pwm/pwmchip0/pwm%d/enable, errno = %d\n", pwmnum, errno);
        return (ERROR);
        }

    /* Enable by writing a "1" (in ASCII) */ 
    bytes = write(fd, "1", 1);     
    if (bytes != 1)
        {
        fprintf(stderr, "Error writing enable to PWM %d errno=%d\n", pwmnum, errno);
        return(ERROR);
        }

    /* Close the sysfs interface */ 
    (void)close(fd);
    
    return(OK);
}

/*******************************************************************************
 * gpio_write_pwm - Write a value to a prepared PWM output pin
 *
 * Open the appropriate sysfs file, write the data, close the file descriptor
 *
 * Arguments: pwmnum - PWM pin to write (pwm pin num - NOT physical pin number)
 *            value - duty_cycle to write (int) - between 0 and max value which  
 *                       is ((2 ^ pwmWriteResolution) - 1)
 *
 * Returns:   0 - OK
 *           -1 - ERROR
 */
int gpio_write_pwm(uint8_t pwmnum, int value)
{
    char fdBuffer[FD_BUFFER_SZ];
    char dtBuffer[DT_BUFFER_SZ];
    int bytes;
    int fd;
    int pval;

    /* Clear the data buffer */
    memset(dtBuffer, 0, DT_BUFFER_SZ);

    /* Scale the input value */
    /* PWM period, in ns, is stored as a global. pwmWriteResolution can be
       changed through the adcWriteResolution() function. */
    pval = (pwmPeriod >> pwmWriteResolution) * value;
    sprintf(dtBuffer, "%d", pval);

    /* Open the PWM device - pwmchip0 */
    sprintf(fdBuffer, "/sys/class/pwm/pwmchip0/pwm%d/duty_cycle", pwmnum);    
    fd = open(fdBuffer, O_WRONLY, 0664);
    if (fd == -1)
        {
        fprintf(stderr, "Error opening %s, errno = %d\n", fdBuffer, errno);
        return (ERROR);
        }
        
    /* Write data to the pwm */
    bytes = write(fd, dtBuffer, strlen(dtBuffer));     
    if (bytes != strlen(dtBuffer))
        {
        fprintf(stderr, "Error writing duty cycle PWM %d errno=%d\n", pwmnum, errno);
        return(ERROR);
        }

    /* Close the sysfs interface */ 
    (void) close(fd);

    return OK;
}

/*******************************************************************************
 * int_gpio_read_pin - Read a value from a GPIO pin
 *
 * Open the appropriate sysfs file, read the data, close the file descriptor
 *
 * Arguments: gpionum - GPIO pin number to read (linux number - NOT pin number)
 *
 * Returns:  Value of pin (*uint8_t) = 0 or 1 
 *           Status (OK=0,  ERROR=-1)
 */
int int_gpio_read_pin(uint8_t gpionum)
{
    int fd;
    char buffer[FD_BUFFER_SZ];
    int  bytes;
    char byteread[DT_BUFFER_SZ];

    /* Clear the data buffer */
    memset(byteread, 0, DT_BUFFER_SZ);
    
    /* Open the appropriate sysfs interface */ 
    sprintf(buffer, "/sys/class/gpio/gpio%d/value", gpionum);    
    fd = open(buffer, O_RDONLY);
    if (fd == -1)
        {
        fprintf(stderr, "Error opening %s, errno = %d\n", buffer, errno);
        return (ERROR);
        }

    /* Read the value (will be in ASCII) */
    bytes = read(fd, byteread, 2);
    if (bytes < 0)
        {
        fprintf(stderr, "Error reading GPIO %d - bytes read %d, error = %d\n", gpionum, bytes, errno);
        return (ERROR);
        }
       
    /* Close the sysfs interface */ 
    (void) close(fd);
    return ((uint8_t)byteread[0] & 0x01);
}

/*******************************************************************************
 * gpio_fd_open - Open a file descriptor to a specific gpio pin on the 
 *                Galileo Gen2. This is used for faster access to gpio pins.
 *
 * Arguments: gpionum - GPIO pin number to open a fd for
 *
 * Returns:   int - the fd returned
 */
int gpio_fd_open(uint8_t gpionum)
{
    int fd;
    char buffer[FD_BUFFER_SZ];

    /* Open the appropriate sysfs interface */ 
    sprintf(buffer, "/sys/class/gpio/gpio%d/value", gpionum);    
    fd = open(buffer, O_RDONLY);
    if (fd == -1)
        {
        fprintf(stderr, "Error opening %s, errno = %d\n", buffer, errno);
        return (ERROR);
        }
    return fd;
}

/*******************************************************************************
 * gpio_fd_close - Close a file descriptor
 *
 * Arguments: int - fd, a file descriptor
 *
 * Returns:   int - either OK or FAIL
 */
int gpio_fd_close(int fd)
{
    /* Close the sysfs interface */ 
    (void) close(fd);
    return OK;
}

/*******************************************************************************
 * gpio_fd_read_pin - Read the value of a digital I/O, using a file descriptor
 *                    to the gpio pin as the input. This should be used after
 *                    opening the file descriptor using the gpio_fd_open
 *                    routine.
 *
 * Arguments: int - fd, a file descriptor
 *
 * Returns:   int - value/state read from the gpio pin (or -1 if error)
 */
int gpio_fd_read_pin(int fd)
{
    int  bytes;
    char byteread[DT_BUFFER_SZ];

    /* Clear the data buffer */
    memset(byteread, 0, DT_BUFFER_SZ);

    /* Read the value (will be in ASCII) */
    bytes = read(fd, byteread, 2);
    if (bytes < 0)
        {
        fprintf(stderr, "Error reading GPIO(fd) %d - bytes read %d, error = %d\n", fd, bytes, errno);
        return (ERROR);
        }
    return ((uint8_t)byteread[0] & 0x01);
}

/*******************************************************************************
 * gpio_get_descriptor - Return a file descriptor for a specific gpio pin
 *          NOTE - this is only valid for digital GPIO I/O pins
 *
 * Arguments: pinnum - pin to write (gpio pin num - NOT physical pin number)
 *
 * Returns:  fd associated with pin (int)
 *
 */
int gpio_get_descriptor(uint8_t pinnum)
{
    char fdBuffer[FD_BUFFER_SZ];
    int fd;

    if(descriptors[pinnum]) 
    {
        return descriptors[pinnum];
    } 
    else 
    {
        /* Open the GPIO device */
        sprintf(fdBuffer, "/sys/class/gpio/gpio%d/value", pinnum);    
        fd = open(fdBuffer, O_RDWR, 0664);
        if (fd == -1)
        {
            fprintf(stderr, "Error opening %s, errno = %d\n", fdBuffer, errno);
            return (ERROR);
        }
        descriptors[pinnum] = fd;
        return fd;
    }
}
