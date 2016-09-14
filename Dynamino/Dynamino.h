/*
MIT License

Copyright (c) 2016 Alistair Wick

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/

#ifndef DYNAMINO_H
#define DYNAMINO_H

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

// Assorted defines
#define DXL1_INSTR_PING             0x01
#define DXL1_INSTR_READ             0x02
#define DXL1_INSTR_WRITE            0x03
#define DXL1_INSTR_REG_WRITE        0x04
#define DXL1_INSTR_ACTION           0x05
#define DXL1_INSTR_RESET            0x06

#define DXL1_ERRFLAGS_VOLTAGE       0x01
#define DXL1_ERRFLAGS_ANGLELIM      0x02
#define DXL1_ERRFLAGS_OVERHEAT      0x04
#define DXL1_ERRFLAGS_OPRANGE       0x08
#define DXL1_ERRFLAGS_CHECKSUM      0x10
#define DXL1_ERRFLAGS_OVERLOAD      0x20
#define DXL1_ERRFLAGS_INSTR         0x40
#define DXL1_ERRFLAGS_UNDEF         0x80

#define DXL1_INDEX_ID               2
#define DXL1_INDEX_LENGTH           3
#define DXL1_INDEX_INSTR            4
#define DXL1_INDEX_ERR              4
#define DXL1_INDEX_PARAMS           5

#define DXL1_PACKET_BASELENGTH      4
#define DXL1_BODY_BASELENGTH        2

// Extended flags
#define DXL__ERRFLAGS_OK            0x0000
#define DXL__ERRFLAGS_TIMEOUT       0x0100
#define DXL__ERRFLAGS_CORRUPT       0x0200
#define DXL__ERRFLAGS_MALFORMED     0x0400

// Core functionality for V1 protocol
void DXL1_write_1mb(uint8_t data, uint8_t pin = 2);
uint16_t DXL1_receive_1mb(uint8_t pin, uint8_t params_buff[], uint8_t *nparams);
uint16_t DXL1_read(uint8_t pin, uint8_t id, uint8_t *params_buff, uint8_t *nparams, uint8_t adr, uint8_t len);
uint16_t DXL1_write(uint8_t pin, uint8_t id, uint8_t *params_buff, uint8_t nparams, uint8_t adr);

// Debug/util
void DXL1_printErrorMessage(uint16_t errorCode);

#endif

