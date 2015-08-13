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

#ifndef __BB_I2C_H__
#define __BB_I2C_H__

#include <Arduino.h>

// Pins to be used in the bit banging
// A5 = PPB02 on the SAMD21
#define I2C_CLK_PORT PORTB
#define I2C_CLK_PIN 2
// A4 = PA05 on the SAMD21
#define I2C_DAT_PORT PORTA
#define I2C_DAT_PIN 5


#define I2C_ACK 1
#define I2C_NAK 0

// Inits bitbanging port, must be called before using the functions below
void I2C_Init();

// Send a START Condition
void I2C_Start();

// Send a STOP Condition
void I2C_Stop();

// write a byte to the I2C slave device
unsigned char I2C_Write(unsigned char c);

// read a byte from the I2C slave device
unsigned char I2C_Read(unsigned char ack);

// High-level Function to read a byte from a specific register on a device
uint8_t I2C_ReadByte(uint8_t deviceAddress, uint8_t registerAddress);

// High-level Function to read a 16-bit integer from a specific register on a device
uint16_t I2C_ReadWord(uint8_t deviceAddress, uint8_t registerAddress);

// High-level Function to wirte a byte integer from a specific register on a device
void I2C_WriteByte(uint8_t deviceAddress, uint8_t registerAddress, uint8_t data);

#endif //__BB_I2C_H__