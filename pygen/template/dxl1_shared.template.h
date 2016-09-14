#ifndef DYNAMINO_H
#define DYNAMINO_H

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

// Assorted defines
#define {{prefix}}_INSTR_PING             0x01
#define {{prefix}}_INSTR_READ             0x02
#define {{prefix}}_INSTR_WRITE            0x03
#define {{prefix}}_INSTR_REG_WRITE        0x04
#define {{prefix}}_INSTR_ACTION           0x05
#define {{prefix}}_INSTR_RESET            0x06

#define {{prefix}}_ERRFLAGS_VOLTAGE       0x01
#define {{prefix}}_ERRFLAGS_ANGLELIM      0x02
#define {{prefix}}_ERRFLAGS_OVERHEAT      0x04
#define {{prefix}}_ERRFLAGS_OPRANGE       0x08
#define {{prefix}}_ERRFLAGS_CHECKSUM      0x10
#define {{prefix}}_ERRFLAGS_OVERLOAD      0x20
#define {{prefix}}_ERRFLAGS_INSTR         0x40
#define {{prefix}}_ERRFLAGS_UNDEF         0x80

#define {{prefix}}_INDEX_ID               2
#define {{prefix}}_INDEX_LENGTH           3
#define {{prefix}}_INDEX_INSTR            4
#define {{prefix}}_INDEX_ERR              4
#define {{prefix}}_INDEX_PARAMS           5

#define {{prefix}}_PACKET_BASELENGTH      4
#define {{prefix}}_BODY_BASELENGTH        2

// Extended flags
#define {{family}}__ERRFLAGS_OK            0x0000
#define {{family}}__ERRFLAGS_TIMEOUT       0x0100
#define {{family}}__ERRFLAGS_CORRUPT       0x0200
#define {{family}}__ERRFLAGS_MALFORMED     0x0400

// Core functionality for V1 protocol
void {{prefix}}_write_1mb(uint8_t data, uint8_t pin = 2);
uint16_t {{prefix}}_receive_1mb(uint8_t pin, uint8_t params_buff[], uint8_t *nparams);
uint16_t {{prefix}}_read(uint8_t pin, uint8_t id, uint8_t *params_buff, uint8_t *nparams, uint8_t adr, uint8_t len);
uint16_t {{prefix}}_write(uint8_t pin, uint8_t id, uint8_t *params_buff, uint8_t nparams, uint8_t adr);

// Debug/util
void {{prefix}}_printErrorMessage(uint16_t errorCode);

#endif
