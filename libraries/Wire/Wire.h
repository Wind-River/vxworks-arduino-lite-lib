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
 *  - wire.h
 * Wire Library for communicating with I2C
 */

#ifndef _WIRE_HPP_
#define _WIRE_HPP_

#define DEFAULT_EMPTYBUFFER_VALUE  0xff
#define TRANSMIT_DATA_SIZE         128
#define READ_DATA_SIZE             32

#ifdef __cplusplus

class wire 
{
public:
  wire();
  void begin ( void );
  void beginTransmission ( uint8_t devaddr );
  int write ( uint8_t bvalue );
  int write ( char *strdata );
  int write ( uint8_t *barray, int bnum );
  void endTransmission ( int mode = 0 );
  void groveLCDSetColor ( uint8_t red, uint8_t green, uint8_t blue);
  void groveLCDCommand ( uint8_t command);
  void groveLCDWriteLine ( uint8_t line, char *strdata);
  int requestFrom ( uint8_t devaddr, int numbytes );
  uint8_t read( void );
  int available(void);
private:
  uint8_t transmitAddr;
  int transmit_fd;
  int byteTransmitCount;
  uint8_t transmitData[TRANSMIT_DATA_SIZE];
  uint8_t readAddr;
  int read_fd;
  int readCount;
  int currentReadByte;
  int bufferedBytes;
  uint8_t readData[READ_DATA_SIZE];
};

extern wire Wire;

#endif /* __cplusplus */

#endif /* _WIRE_HPP_ */
