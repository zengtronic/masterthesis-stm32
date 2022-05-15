/*
 * util.h
 *
 * Collection of useful functions
 * Source over the specific code sections
 *
 *  Created on: Nov 1, 2021
 *      Author: Stefan
 */

#ifndef INC_UTIL_H_
#define INC_UTIL_H_

#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdbool.h>


/* constrain function (from Arduino.h) */
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

// Type Definitions
typedef uint8_t byte;

enum DRIVEMODE
{
    off,
	driving,
	driving_yabr,
	driving_stefan
} drivingMode;

typedef struct rxData{
	uint8_t command;
	uint8_t data[16];
} rxdata_t;

// full telemetry data - size 60 bytes
typedef struct telemetryData{
	uint8_t startbyte;
	uint8_t typebyte;
	int16_t yaw;
	int16_t pitch;
	int16_t roll;

	int16_t gyroX;
	int16_t gyroY;
	int16_t gyroZ;
	int16_t accX;
	int16_t accY;
	int16_t accZ;

	int16_t balancing_pid_in;
	int16_t balancing_pid_out;
	int16_t balancing_pid_set;
	//int16_t balancing_pterm;
	//int16_t balancing_iterm;
	//int16_t balancing_dterm;

	int16_t speed_pid_in;
	int16_t speed_pid_out;
	int16_t speed_pid_set;
	//int16_t speed_pterm;
	//int16_t speed_iterm;
	//int16_t speed_dterm;

	int16_t angle_actual;
	int16_t speed_actual;

	int16_t throttle;
	int16_t steering;

	int16_t motor_left;
	int16_t motor_right;
	bool fallen;

	uint8_t notused_1;
	uint8_t notused_2;
	uint8_t endbyte;
}telemetrydata_t;

// small telemtry data struct - size 24 bytes
typedef struct smallTelemetryData{
	uint8_t startbyte;
	uint8_t typebyte;
	int16_t yaw;
	int16_t pitch;
	int16_t roll;
	int16_t motor_left;
	int16_t motor_right;
	int16_t speed_actual;
	int16_t angle_actual;
	int16_t throttle;
	int16_t steering;
	bool fallen;
	uint8_t notused_1;
	uint8_t notused_2;
	uint8_t endbyte;
} smalltelemetrydata_t;

// small parameter data struct - size 24 bytes
typedef struct parameterData{
	uint8_t startbyte;
	uint8_t typebyte;
	int16_t balancing_kp;
	int16_t balancing_ki;
	int16_t balancing_kd;
	int16_t speed_kp;
	int16_t speed_ki;
	int16_t speed_kd;
	int16_t angle_offset;
	bool 	speed_mirror;
	bool 	balancing_mirror;
	uint8_t notused_1;
	uint8_t notused_2;
	uint8_t notused_3;
	uint8_t notused_4;
	uint8_t notused_5;
	uint8_t endbyte;
} parameterdata_t;

void processCommand(rxdata_t datablock);

void scan_i2c();

/* timing functions */
void delay_us (uint16_t us);
void delay (uint16_t ms);
uint32_t millis();

/* init functions */
void initIMU();
void initPID();

/* handle functions */
void handleIMU(void);
void handleMovement(void);
void handleTelemetry(void);

/* PID Controllers */
float calcSpeedPID(float input, float setpoint);
float calcBalancingPID(float input, float setpoint);


/**
  * This file is part of the hoverboard-sideboard-hack project.
  *
  * Copyright (C) 2020-2021 Emanuel FERU <aerdronix@gmail.com>
  *
  * This program is free software: you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation, either version 3 of the License, or
  * (at your option) any later version.
  *
  * This program is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  * GNU General Public License for more details.
  *
  * You should have received a copy of the GNU General Public License
  * along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/* general functions */
void consoleLog(char *message);
void get_tick_count_ms(unsigned long *count);

/* i2c write/read functions */
int8_t i2c_writeBytes(uint8_t slaveAddr, uint8_t regAddr, uint8_t length, uint8_t *data);
int8_t i2c_writeByte (uint8_t slaveAddr, uint8_t regAddr, uint8_t data);
int8_t i2c_writeBit  (uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t  data);
int8_t i2c_readBytes (uint8_t slaveAddr, uint8_t regAddr, uint8_t length, uint8_t *data);
int8_t i2c_readByte  (uint8_t slaveAddr, uint8_t regAddr, uint8_t *data);
int8_t i2c_readBit   (uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data);

#endif /* INC_UTIL_H_ */
