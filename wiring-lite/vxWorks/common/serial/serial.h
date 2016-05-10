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
 *  - serial.h
 * Serial API source header.
 */

#ifndef _SERIAL_HPP_
#define _SERIAL_HPP_

#include <vxWorks.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <ctype.h>        /* character functions */

#define  MAX_READSTRING_LEN  80

#ifdef __cplusplus

class serial   /* Standard way of defining the class */
{
public:
  serial();    /* Constructor */
  void begin( int baud );
  int available ();
  int read();
  char * readString();
  int println ( int q );
  int println ( int p, int nbase );
  int println ( double p );
  int println ( double p, int places );
  int println ( char p );
  int println ( char *p );
  int print ( int p ) { return printf("%d", p); }
  int print ( int p, int nbase );
  int print ( double p ) { return printf("%.2f", p); }
  int print ( double p, int places ) { return printf("%.*f", places, p); }
  int print ( char p ) { return printf("%c", p); }
  int print ( char *p ) { return printf("%s", p); }
  char thisText[MAX_READSTRING_LEN];
private:
  int intval;
};

extern serial Serial;

#endif /*  __cplusplus */

#endif /* _SERIAL_HPP_ */
