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
 *  - rgb_lcd.cpp
 * Interface for LCD display on Grove Starter kit on Galileo Gen 2 
 */

#include <stdio.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <ioLib.h>
#include <unistd.h>
#include <i2c.h>
#include <math.h>

#include <Arduino-lite.h>
#include <Wire.h>
#include <rgb_lcd.h>

/* ******************************************************************* */
/* Add these defines to enable debug and/or error message printing 
 * Messages take the form...
 *        DEBUG_GROVE("Debug data is %d\n", value);
 *        ERROR_GROVE("Error message: %.2d\n", floatvalue);
*/
#define ENABLE_ERROR_GROVE
// #define ENABLE_DEBUG_GROVE

#ifdef ENABLE_ERROR_GROVE
#define ERROR_GROVE(...) fprintf (stderr, __VA_ARGS__)
#else
#define ERROR_GROVE(...)
#endif
#ifdef ENABLE_DEBUG_GROVE
#define DEBUG_GROVE(...) fprintf (stderr, __VA_ARGS__)
#else
#define DEBUG_GROVE(...)
#endif
/* ******************************************************************* */

/* ***** rgb_lcd class *****
 * Replicates the functionality provided by the Grove rgb_lcd class to drive
 * the Grove LCD display. Overlays (uses) the LCD routines in the Grove class.
 */

/* The following instantiation would normally be placed in the Arduino sketch 
 * by default. Uncomment the following to make this unnecessary. */
rgb_lcd lcd;

/*******************************************************************************
 * rgb_lcd library constructor
 *
 * Arguments: none
 * Returns:  none 
 */
rgb_lcd::rgb_lcd()
{
}

/*******************************************************************************
 * begin() - initialise the LCD display.
 *
 * Arguments: unsigned char, cols, number of columns
 *            unsigned char, lines, number of lines
 * Returns:   none
 * 
 * Note: this function currently only sets the display to 2 line mode.
 */
void rgb_lcd::begin(uint8_t cols, uint8_t lines)
{
    /* Set the LCD to 2 line mode */    
    command(LCD_FUNCTIONSET | LCD_2LINE);
    /* Display on. Cursor on. No Blink */
    lcd_display_state = (LCD_DISPLAYON | LCD_CURSORON | LCD_BLINKOFF);
    command(LCD_DISPLAYCONTROL | lcd_display_state);
    /* Clear the display */
    command(LCD_CLEAR);
}

/*******************************************************************************
 * clear() - clear the LCD display.
 *
 * Arguments: none
 * Returns:   none
 */
void rgb_lcd::clear(void)
{
    command(LCD_CLEAR);         // clear the lcd display
}

/*******************************************************************************
 * home() - send the cursor to the home position.
 *
 * Arguments: none
 * Returns:   none
 */
void rgb_lcd::home(void)
{
    command(LCD_CURSORHOME);    // send lcd cursor home
}

/*******************************************************************************
 * noDisplay() - switch off the LCD display.
 *
 * Arguments: none
 * Returns:   none
 */
void rgb_lcd::noDisplay(void)
{
    lcd_display_state &= LCD_DISPLAYOFF;
    command(LCD_DISPLAYCONTROL | lcd_display_state);
}

/*******************************************************************************
 * display() - switch on the LCD display.
 *
 * Arguments: none
 * Returns:   none
 */
void rgb_lcd::display(void)
{
    lcd_display_state |= LCD_DISPLAYON;
    command(LCD_DISPLAYCONTROL | lcd_display_state);
}

/*******************************************************************************
 * noCursor() - switch off the LCD cursor.
 *
 * Arguments: none
 * Returns:   none
 */
void rgb_lcd::noCursor(void)
{
    lcd_display_state &= LCD_CURSOROFF;
    command(LCD_DISPLAYCONTROL | lcd_display_state);
}

/*******************************************************************************
 * cursor() - switch on the LCD cursor.
 *
 * Arguments: none
 * Returns:   none
 */
void rgb_lcd::cursor(void)
{
    lcd_display_state |= LCD_CURSORON;
    command(LCD_DISPLAYCONTROL | lcd_display_state);
}

/*******************************************************************************
 * noBlink() - stop blinking the LCD cursor.
 *
 * Arguments: none
 * Returns:   none
 */
void rgb_lcd::noBlink(void)
{
    lcd_display_state &= LCD_BLINKOFF;
    command(LCD_DISPLAYCONTROL | lcd_display_state);
}

/*******************************************************************************
 * blink() - start blinking the LCD cursor.
 *
 * Arguments: none
 * Returns:   none
 */
void rgb_lcd::blink(void)
{
    lcd_display_state |= LCD_BLINKON;
    command(LCD_DISPLAYCONTROL | lcd_display_state);
}

/*******************************************************************************
 * scrollDisplayLeft() - scroll the LCD display to the left.
 *
 * Arguments: none
 * Returns:   none
 */
void rgb_lcd::scrollDisplayLeft(void)
{
    command(LCD_SHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}

/*******************************************************************************
 * scrollDisplayRight() - scroll the LCD display to the right.
 *
 * Arguments: none
 * Returns:   none
 */
void rgb_lcd::scrollDisplayRight(void)
{
    command(LCD_SHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

/*******************************************************************************
 * leftToRight() - configure the LCD for text that flows left to right.
 *
 * Arguments: none
 * Returns:   none
 */
void rgb_lcd::leftToRight(void)
{
    lcd_display_state |= LCD_ENTRYLEFT;
    command(LCD_INPUTSET | lcd_display_state);
}

/*******************************************************************************
 * rightToLeft() - configure the LCD for text that flows right to left.
 *
 * Arguments: none
 * Returns:   none
 */
void rgb_lcd::rightToLeft(void)
{
    lcd_display_state &= ~LCD_ENTRYLEFT;
    command(LCD_INPUTSET | lcd_display_state);
}

/*******************************************************************************
 * autoscroll() - configure the LCD to right justify test from the cursor.
 *
 * Arguments: none
 * Returns:   none
 */
void rgb_lcd::autoscroll(void)
{
    lcd_display_state |= LCD_ENTRYSHIFTINCREMENT;
    command(LCD_INPUTSET | lcd_display_state);
}

/*******************************************************************************
 * noAutoscroll() - configure the LCD to left justify test from the cursor.
 *
 * Arguments: none
 * Returns:   none
 */
void rgb_lcd::noAutoscroll(void)
{
    lcd_display_state &= ~LCD_ENTRYSHIFTINCREMENT;
    command(LCD_INPUTSET | lcd_display_state);
}

/*******************************************************************************
 * setRGB(unsigned char,unsigned char,unsigned char) - change the backlight of
 *                the LCD display 
 *
 * Arguments: unsigned char, red (0 to 255)
 *            unsigned char, green (0 to 255)
 *            unsigned char, blue (0 to 255)
 * Returns:   none
 */
void rgb_lcd::setRGB(unsigned char r, unsigned char g, unsigned char b)
{
    int lcd_fd = -1;
    
    lcd_fd = open("/dev/i2c-0", O_RDWR);
    if (lcd_fd == -1) 
    {
        ERROR_GROVE("[ERROR] rgb_lcd::setRGB - Error opening I2C Bus\n");
        return;
    }
    if (ioctl(lcd_fd, I2C_SLAVE_ADDR, RGB_I2C_ADDR) < 0) 
    {
        ERROR_GROVE("[ERROR] rgb_lcd::setRGB - Error setting LCD I2C address\n");
        return;
    }
    wire::write((uint8_t)0);
    wire::write((uint8_t)0);
    endTransmission(lcd_fd);
    wire::write((uint8_t)0);
    wire::write((uint8_t)1);
    endTransmission(lcd_fd);
    wire::write((uint8_t)8);
    wire::write((uint8_t)0xaa);
    endTransmission(lcd_fd);
    wire::write((uint8_t)4);
    wire::write((uint8_t)r);
    endTransmission(lcd_fd);
    wire::write((uint8_t)3);
    wire::write((uint8_t)g);
    endTransmission(lcd_fd);
    wire::write((uint8_t)2);
    wire::write((uint8_t)b);
    endTransmission(lcd_fd);
    (void)close(lcd_fd);
}

/*******************************************************************************
 * print(char *, unsigned char) - show a line of text on a specific line
 *                         of the LCD display
 *
 * Arguments: char *, a string of text
 *            unsigned char, line (1 = top line, 2 = lower line)
 * Returns:   none
 */
void rgb_lcd::print( char *textdata, uint8_t row )
{
	int y;

	if(row == 1)
	{
		command(0x80);
	} else {
		command(0xc0);
	}
	beginTransmission(LCD_I2C_ADDR);
	wire::write((uint8_t)0x40);
    y = wire::write(textdata);
    if(y < 16)
    {
		char spaces[17] = "                ";
		spaces[16 - y] = 0;
		wire::write(spaces);
    }
    endTransmission();
}

/*******************************************************************************
 * createChar(unsigned char,unsigned char[]) - Write a custom character to 
 *                               one of eight CGRAM locations
 *
 * Arguments: unsigned char, number of CGRAM location
 *            unsigned char[], 8 byte image map for custom character
 * Returns:   none
 */
void rgb_lcd::createChar(uint8_t charloc, uint8_t charmap[])
{
    charloc &= 0x7;             // only allow locations 0 to 7
    command(LCD_SETCGRAMADDR | (charloc << 3));

    unsigned char cdata[9];
    cdata[0] = 0x40;
    for(int i=0; i<8; i++)
    {
        cdata[i+1] = charmap[i];
    }
    beginTransmission(LCD_I2C_ADDR);
    wire::write(cdata, 9);
    endTransmission(1);
}

/*******************************************************************************
 * setCursor - send the cursor to a specific place on the display
 * 
 * Arguments: column, column location (visible range 0-15)
 *            row, row location (visible range 0-1)
 * Returns:   none
 */
void rgb_lcd::setCursor(uint8_t column, uint8_t row)
{
    column = (row == 0 ? (column | LCD_SETDDRAMADDR) :
                         (column | LCD_SETDDRAMADDR | LCD_SETCGRAMADDR));
    command(column);
}

/*******************************************************************************
 * command - write a command byte to the LCD display
 * 
 * Arguments: command, a single byte command to write to the display
 * Returns:   none
 */
void rgb_lcd::command ( uint8_t command )
{
    beginTransmission(LCD_I2C_ADDR);
    wire::write((uint8_t)0);
    wire::write(command);
    endTransmission(1);
	/* add a delay for command completion */
	switch(command)
    {
    case LCD_CLEAR:        // Wait 1.53 ms (rounded to 1.6 ms)
    case LCD_CURSORHOME:
        delayMicroseconds(1600);
        break;
    default:              // Wait 39 usec (rounded up to 40)
        delayMicroseconds(40);
        break;
    }
}

/*******************************************************************************
 * write - write a single byte of data to the LCD display
 * 
 * Arguments: value, a single byte of data to write to the display
 * Returns:   1 for success
 */
size_t rgb_lcd::write(uint8_t value)
{
    beginTransmission(LCD_I2C_ADDR);
    wire::write(0x40);
    wire::write(value);
    endTransmission(1);
    return 1;               /* assume success */
}

/*******************************************************************************
 * writeArray - write an array of data bytes to the LCD display
 * 
 * Arguments: values, an array of bytes to write to the display
 * Returns:   1 for success
 */
size_t rgb_lcd::writeArray(uint8_t values[], int arraySize)
{
    uint8_t cdata[32];
    cdata[0] = 0x40;

    for(int i=0; i < arraySize; i++)
    {
        cdata[i+1] = values[i];
    }
    beginTransmission(LCD_I2C_ADDR);
    wire::write(cdata, (arraySize + 1));
    endTransmission(1);
    return 1;               /* assume success */
}
