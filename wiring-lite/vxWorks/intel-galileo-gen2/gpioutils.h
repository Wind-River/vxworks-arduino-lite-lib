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
 *  - gpioutils.h
 * A set of useful utilities to work with GPIO through the sysfs interface.
 * See gpioutils.c
 */

#ifndef _GPIOUTILS_H_
#define _GPIOUTILS_H_

#define FD_BUFFER_SZ  128
#define DT_BUFFER_SZ  16

#define DEFAULT_PWM_PERIOD      1000000
#define DEFAULT_ADC_RESOLUTION  10
#define DEFAULT_PWM_RESOLUTION  8

/* storage for file descriptors */
int descriptors[64];

extern int gpio_alloc(int gpionum);
extern int gpio_dealloc(int gpionum);
extern int gpio_set_direction(int gpuinum, char *direction);
extern int gpio_write_pin(int gpuinum, char *value);
extern int gpio_read_pin(uint8_t gpionum, uint8_t* val);
extern int gpio_read_adc(uint8_t adcnum);
extern int gpio_pwm_export(int pwmnum);
extern int gpio_write_pwm(uint8_t pwmnum, int value);
extern int int_gpio_read_pin(uint8_t gpionum);
extern int gpio_fd_open(uint8_t gpionum);
extern int gpio_fd_close(int fd);
extern int gpio_fd_read_pin(int fd);
extern int gpio_get_descriptor(uint8_t pinnum);

#endif  /* _GPIOUTILS_H_ */
