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

// Define to prevent recursive inclusion
#ifndef DEFINES_H
#define DEFINES_H


// Includes
#include "stm32f4xx_hal.h"
#include "config.h"
#include "util.h"


#if defined(PRINTF_FLOAT_SUPPORT) && defined(SERIAL_DEBUG) && defined(__GNUC__)
    asm(".global _printf_float"); 		// this is the magic trick for printf to support float. Warning: It will increase code considerably! Better to avoid!
#endif

/* =========================== Defines General =========================== */
// #define _BV(bit)                     (1 << (bit)) 
#define ARRAY_LEN(x)                (uint32_t)(sizeof(x) / sizeof(*(x)))
#define CLAMP(x, low, high)         (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))
#define min(a, b)                   (((a) < (b)) ? (a) : (b))
#define max(a, b)                   (((a) > (b)) ? (a) : (b))
#define i2c_write                   i2c_writeBytes
#define i2c_read                    i2c_readBytes
#define delay_ms                    HAL_Delay
#define get_ms                      get_tick_count_ms
#define log_i                       printf          // redirect the log_i debug function to printf



/* =========================== Defines MPU-6050 =========================== */
#define RAD2DEG                     57.295779513082323      // RAD2DEG = 180/pi. Example: angle[deg] = angle[rad] * RAD2DEG
#define q30                         1073741824              // 1073741824 = 2^30
#define ACCEL_ON                    (0x01)
#define GYRO_ON                     (0x02)
#define COMPASS_ON                  (0x04)

#define PRINT_ACCEL                 (0x01)
#define PRINT_GYRO                  (0x02)
#define PRINT_QUAT                  (0x04)
#define PRINT_EULER                 (0x08)
#define PRINT_TEMP                  (0x10)
#define PRINT_PEDO                  (0x20)

typedef struct{
    int16_t     x;
    int16_t     y;
    int16_t     z;
} Gyro;

typedef struct{
    int16_t     x;
    int16_t     y;
    int16_t     z;
} Accel;

typedef struct{
    int32_t     w;
    int32_t     x;
    int32_t     y;
    int32_t     z;
} Quaternion;

typedef struct{
    int16_t     roll;
    int16_t     pitch;
    int16_t     yaw;
} Euler;

typedef struct{
    int16_t     x;
    int16_t     y;
    int16_t     z;
} Compass;

typedef struct {
    Gyro        gyro;
    Accel       accel;
    Quaternion  quat;
    Euler       euler;
    int16_t     temp;
    Compass 	comp;
} MPU_Data;


#endif //  DEFINES_H 
