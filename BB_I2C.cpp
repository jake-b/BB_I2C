/*
 * BB_I2C - Bit-banged I2C Library for the Arduino Zero
 * Copyright (C) 2015 J. Bordens
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of  MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// BitBang I2C for the Arduino Zero
// Derrived from http://codinglab.blogspot.com/2008/10/i2c-on-avr-using-bit-banging.html

#include "BB_I2C.h"
#include <Arduino.h>

// Delay used to set speed of transfer (microseconds)
#define I2C_DELAY() delayMicroseconds(3)

// Macros for controlling pins.
#define I2C_DATA_HI() \
    PORT->Group[I2C_DAT_PORT].PINCFG[I2C_DAT_PIN].reg=(uint8_t)(PORT_PINCFG_INEN|PORT_PINCFG_PULLEN);\
    PORT->Group[I2C_DAT_PORT].DIRCLR.reg = (1ul << I2C_DAT_PIN);\
    PORT->Group[I2C_DAT_PORT].OUTSET.reg = (1ul << I2C_DAT_PIN);

#define I2C_CLOCK_HI() \
   PORT->Group[I2C_CLK_PORT].PINCFG[I2C_CLK_PIN].reg=(uint8_t)(PORT_PINCFG_INEN|PORT_PINCFG_PULLEN);\
   PORT->Group[I2C_CLK_PORT].DIRCLR.reg = (1ul << I2C_CLK_PIN);\
   PORT->Group[I2C_CLK_PORT].OUTSET.reg = (1ul << I2C_CLK_PIN);

#define I2C_CLOCK_LO() \
   PORT->Group[I2C_CLK_PORT].PINCFG[I2C_CLK_PIN].reg&=~(uint8_t)(PORT_PINCFG_INEN); \
   PORT->Group[I2C_CLK_PORT].DIRSET.reg = (1ul << I2C_CLK_PIN); \
   PORT->Group[I2C_CLK_PORT].OUTCLR.reg = (1ul << I2C_CLK_PIN);

#define I2C_DATA_LO() \
   PORT->Group[I2C_DAT_PORT].PINCFG[I2C_DAT_PIN].reg&=~(uint8_t)(PORT_PINCFG_INEN);\
   PORT->Group[I2C_DAT_PORT].DIRSET.reg = (1ul << I2C_DAT_PIN);\
   PORT->Group[I2C_DAT_PORT].OUTCLR.reg = (1ul << I2C_DAT_PIN);

void I2C_WriteBit(unsigned char c) {
    if (c > 0) {
        I2C_DATA_HI();
    }
    else {
        I2C_DATA_LO();
    }

    //Clock Stretching
    I2C_CLOCK_HI();   
    while ((PORT->Group[I2C_CLK_PORT].IN.reg & (1ul << I2C_CLK_PIN))== 0);
    I2C_DELAY();

    I2C_CLOCK_LO();
    I2C_DELAY();
}

unsigned char I2C_ReadBit() {
    I2C_DATA_HI();

    //Clock Stretching
    I2C_CLOCK_HI();
    while ((PORT->Group[I2C_CLK_PORT].IN.reg & (1ul << I2C_CLK_PIN))==0);
    I2C_DELAY();

//    I2C_CLOCK_HI();
//    I2C_DELAY();

    unsigned long c = PORT->Group[I2C_DAT_PORT].IN.reg;

    I2C_CLOCK_LO();
    I2C_DELAY();

    return (c >>I2C_DAT_PIN) & 1;
}

// Inits bitbanging port, must be called before using the functions below
//
void I2C_Init()
{
   I2C_CLOCK_HI();
    I2C_DATA_HI();

    I2C_DELAY();
}

// Send a START Condition
//
void I2C_Start()
{
    // set both to high at the same time
    I2C_DATA_HI();
    I2C_CLOCK_HI();
    
    I2C_DELAY();

    I2C_DATA_LO();
    I2C_DELAY();

    I2C_CLOCK_LO();
    I2C_DELAY();
}

// Send a STOP Condition
//
void I2C_Stop()
{
    // I2C_CLOCK_LO();
    // I2C_DELAY(); 

    // I2C_DATA_LO();
    // I2C_DELAY();

    I2C_CLOCK_HI();
    I2C_DELAY();

    I2C_DATA_HI();
    I2C_DELAY();
}

// write a byte to the I2C slave device
//
unsigned char I2C_Write(unsigned char c)
{
    for (char i = 0; i < 8; i++)
    {
        I2C_WriteBit(c & 128);

        c <<= 1;
    }
    return I2C_ReadBit();
}


// read a byte from the I2C slave device
//
unsigned char I2C_Read(unsigned char ack)
{
    unsigned char res = 0;

    for (char i = 0; i < 8; i++)
    {
        res <<= 1;
        res |= I2C_ReadBit();
    }

I2C_DELAY();
    if (ack > 0)
    {
        I2C_WriteBit(0);
    }
    else
    {
        I2C_WriteBit(1);
    }

    I2C_DELAY();

    return res;
}


uint8_t I2C_ReadByte(uint8_t deviceAddress, uint8_t registerAddress) { 
    uint8_t data; 
    I2C_Start(); 
    I2C_Write(deviceAddress << 1); 
    I2C_Write(registerAddress); 
    I2C_Start(); 
    I2C_Write((deviceAddress << 1) | 0x01 ); 
    data=I2C_Read(I2C_NAK); 
    I2C_Stop(); 
    return(data); 
} 

uint16_t I2C_ReadWord(uint8_t deviceAddress, uint8_t registerAddress) { 
    uint8_t msb, lsb; 

    I2C_Start(); 
    I2C_Write(deviceAddress << 1); 
    I2C_Write(registerAddress);
    I2C_Start(); 
    I2C_Write((deviceAddress << 1) | 0x01 ); 
    msb = I2C_Read(I2C_ACK);
    lsb = I2C_Read(I2C_NAK);
    I2C_Stop();

    return ((uint16_t)msb << 8) | lsb;
} 

void I2C_WriteByte(uint8_t deviceAddress, uint8_t registerAddress, uint8_t data) { 
   I2C_Start(); 
   I2C_Write(deviceAddress << 1); 
   I2C_Write(registerAddress); 
   I2C_Write(data); 
   I2C_Stop(); 
} 