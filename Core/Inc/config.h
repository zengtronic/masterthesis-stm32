/*
 * 	config.c
 *	This file has all settings for the Self Balancing Robot Project
 *  Created on: Oct 25, 2021
 *      Author: Stefan Zengerle
 */


// Define to prevent recursive inclusion
#ifndef CONFIG_H
#define CONFIG_H

/* ==================================== STEPPER =================================== */
#define STEPS_PER_ROUND 			800					// The actual Steps needed to make one round with the stepper
#define US_PER_IR 					1000000 / 50000		// 1s / Interrupt rate in Hz = 20 us (currently)
#define MAX_RPS						11.0					// The maximum rounds per second the stepper can drive

/* ==================================== OFFSETS ==================================== */
#define GYRO_X_OFFSET 			  		29				// MPU gyroscope X-axis offset
#define GYRO_Y_OFFSET 			  		-7				// MPU gyroscope Y-axis offset
#define GYRO_Z_OFFSET 			 		57				// MPU gyroscope Z-axis offset
#define ACC_X_OFFSET 			  		-694			// MPU Accel. X-axis offset
#define ACC_Y_OFFSET 			  		-417			// MPU Accel. Y-axis offset
#define ACC_Z_OFFSET 			  		-64				// MPU Accel. Z-axis offset
#define ANGLE_OFFSET					-1.35			// Init Angle Offset Parameter

/* ============================== PID PARAMETERS =================================== */
#define BALANCING_KP					3.33            // Init Balancing PID P-parameter
#define BALANCING_KI					0.0				// Init Balancing PID I-parameter
#define BALANCING_KD					0.16 			// Init Balancing PID D-parameter
#define BALANCING_DIR					true			// PID Mirroring

#define SPEED_KP						0.01        	    // Init Speed PID P-parameter
#define SPEED_KI						0.0				// Init Speed PID I-parameter
#define SPEED_KD						0.0 			// Init Speed PID D-parameter
#define SPEED_DIR						false			// PID Mirroring

/* ==================================== MOVEMENT =================================== */
#define MAX_THROTTLE 					10.0				// Maximum throttle speed
#define MAX_STEERING 					1.0 			// Maximum steering speed
#define MAX_TILT						2			//
/* =============================== FALL-DETECTION =================================== */
#define TILT_DEACTIVATE					20				// Tilt angle where the motors get deactivated
#define TILT_ACTIVATE 					1				// Tilt angle where the motors get reactivated

/* ======================================= OTHERS =================================== */
#define USE_SMALL_TELEMETRY				false			// Determines if small or full telemetry will be send
#define DEBUG_PRINT						true			// Determines if printf() prints on UART2 (USB)


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


/* ==================================== DO NOT TOUCH SETTINGS ==================================== */
//#define BYPASS_CUBEMX_DEFINES                 // [-] Use this flag to bypass the Ports definitions generated by CUBE MX in main.h and use the ones in defines.h
#define MPU9250                               // [-] Define IMU sensor type
#define MPU_GYRO_FSR              2000        // [deg/s] Set Gyroscope Full Scale Range: 250 deg/s, 500 deg/s, 1000 deg/s, 2000 deg/s. !! DMP sensor fusion works only with 2000 deg/s !!
#define MPU_ACCEL_FSR             2           // [g] Set Acceleromenter Full Scale Range: 2g, 4g, 8g, 16g. !! DMP sensor fusion works only with 2g !!
//#define MPU_I2C_SPEED             400000      // [bit/s] Define I2C speed for communicating with the MPU6050
//#define DELAY_IN_MAIN_LOOP        1           // [ms] Delay in the main loop
// #define PRINTF_FLOAT_SUPPORT                  // [-] Uncomment this for printf to support float on Serial Debug. It will increase code size! Better to avoid it!
/* =============================================================================================== */


/* ==================================== SETTINGS MPU-6050 ==================================== */
#define MPU_SENSOR_ENABLE                     // [-] Enable flag for MPU-6050 sensor. Comment-out this flag to Disable the MPU sensor and reduce code size.
#define MPU_DMP_ENABLE                        // [-] Enable flag for MPU-6050 DMP (Digital Motion Processing) functionality.
#define MPU_DEFAULT_HZ            200         // [Hz] Default MPU frequecy: must be between 1Hz and 200Hz.
#define TEMP_READ_MS              500         // [ms] Temperature read time interval
#define PEDO_READ_MS              1000        // [ms] Pedometer read time interval
#define USE_CAL_HW_REGISTERS                  // [-] Uncommnent this to SAVE the sensor calibration to the MPU-6050 registers after the Self-test was run
#define SERIAL_DEBUG			  0

// DMP Tap Detection Settings
#define DMP_TAP_AXES              TAP_XYZ     // [-] Set which axes will register a tap: TAP_XYZ, TAP_X, TAP_Y, TAP_Z
#define DMP_TAP_THRESH            250         // [mg/ms] Set tap threshold for the selected axis.
#define DMP_TAP_COUNT             4           // [-] Set minimum number of taps needed for an interrupt. Minimum consecutive taps: 1 to 4
#define DMP_TAP_TIME              100         // [ms] Set time length between valid taps.
#define DMP_TAP_TIME_MULTI        500         // [ms] Set max time between taps to register as a multi-tap.
#define DMP_SHAKE_REJECT_THRESH   200         // [deg/s] Set shake rejection threshold in degree per second (dps). If the DMP detects a gyro sample larger than the thresh, taps are rejected.
#define DMP_SHAKE_REJECT_TIME     40          // [ms] Set shake rejection time. Sets the length of time that the gyro must be outside of the DMP_SHAKE_REJECT_THRESH before taps are rejected. A mandatory 60 ms is added to this parameter.
#define DMP_SHAKE_REJECT_TIMEOUT  10          // [ms] Set shake rejection timeout. Sets the length of time after a shake rejection that the gyro must stay inside of the threshold before taps can be detected again. A mandatory 60 ms is added to this parameter.

#endif

