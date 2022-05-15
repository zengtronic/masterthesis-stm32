/*
 * util.c
 *
 * Collection of useful functions as well as the handles
 * Source over the specific code sections
 *
 *  Created on: Nov 1, 2021
 *      Author: Zengtronic
 *
 * Todo:
 *      - Testing the new driving and steering code
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include "math.h"
#include "stm32f4xx_hal.h"
#include "main.h"
#include "defines.h"
#include "config.h"
#include "util.h"
#include "mpu6050.h"
#include "stepper.h"

/* Hardware variables */
extern TIM_HandleTypeDef htim1;
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart4;

/* timing variables */
uint8_t imuLoopCounter = 0;
uint32_t imuPreviousMillis = 0;
uint32_t motorsPreviousMillis = 0;
uint32_t telemetryPreviousMillis = 0;

/* Function interval in ms */
uint16_t imuInterval        = 5;		// 200 Hz
uint16_t movementInterval   = 5;        // 200 Hz
uint16_t telemetryInterval  = 100;      //  10 Hz

/* IMU variables */
extern MPU_Data mpu;                            // holds the MPU-6050 data
static ErrorStatus  mpu_status;                 // holds the MPU-6050 status: SUCCESS or ERROR
float angle_offset;                             // offset which gets added to measured angle
float angle_actual;	                            // holds the actual tilt
float angle_before;                             // holds the previous angle
float target_angle;                             // holds the target angle for movement

/* ===== Speed PID ===== */
/* Parameters */
float speed_kp;		                            // P-Constant of Speed-PID   Gets
float speed_ki;		                            // I-Constant of Speed-PID 	 Filled
float speed_kd;		                            // D-Constant of Speed-PID   at startup
bool  speed_pid_mirror = false;	                // Mirror the PID output?

/* Variables for printing purposes only */
float speed_p_term = 0;
float speed_i_term = 0;
float speed_d_term = 0;
float speed_input_disp = 0;
float speed_output_disp = 0;
float speed_setpoint_disp = 0;
bool brake_needed = false;


enum DRIVEMODE drivemode = driving_stefan;
int rounds_since_control = 0;

/* Internal PID Vars */
unsigned long speed_last_calculation = 0;	    // timestamp of last calculation
float speed_last_error = 0.0;				    // last error.... needed for D-Part of PID
bool  speed_pid_firstrun = true;			    // Determine if this is the first run of the PID



/* ===== Balancing PID ===== */
/* Parameters */
float balancing_kp;		                        // P-Constant of Balancing-PID   Gets
float balancing_ki;		                        // I-Constant of Balancing-PID 	 Filled
float balancing_kd;		                        // D-Constant of Balancing-PID   at startup
bool  balancing_pid_mirror = false;	            // Mirror the PID output?

/* Variables for printing purposes only */
float balancing_p_term = 0;
float balancing_i_term = 0;
float balancing_d_term = 0;
float balancing_input_disp = 0;
float balancing_output_disp = 0;
float balancing_setpoint_disp = 0;

/* Internal PID Vars */
unsigned long balancing_last_calculation = 0;	// timestamp of last calculation
float balancing_last_error = 0.0;				// last error.... needed for D-Part of PID
bool  balancing_pid_firstrun = true;			// Determine if this is the first run of the PID


/* ===== Drive/Motor control ===== */
float steering = 0;						        //gets added to balancing output in juxtaposed maner
float throttle = 0;						        // gets processed to an goal tilt

float speed_actual = 0;
float speed_before = 0;
float motor_speed = 0;
float motor_left_speed = 0;	                    // Speed for left motor		Mainly for
float motor_right_speed = 0;	                // Speed for right motor	display purposes

int fallen = 0;	// Determines if the robot has fallen



/* function to scan for i2c devices */
void scan_i2c(){
	uint8_t error, address;
	  int nDevices;
	  int y = 20;
	  int x = 0;

	  printf( "I2C Scan\n");

	  nDevices = 0;

	  for(address = 1; address < 255; address++) {
	    // The i2c_scanner uses the return value of
	    // the Write.endTransmisstion to see if
	    // a device did acknowledge to the address.
		  error = HAL_I2C_IsDeviceReady(&hi2c1, (int16_t)address,  5, 1000);

	    if (error == 0) {
		  printf( " Found 0x%02X (0x%02X)\n", address, address >> 1);
	  	  x += 42;
	      nDevices++;
	      if(nDevices % 3 == 0){
		      y += 10;
		      x = 0;
	      }
	    }
	  }
}

/* Microsecond delay */
void delay_us (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait for the counter to reach the us input in the parameter
}

/* Millisecond delay (better than the original one which takes +1ms) */
void delay (uint16_t ms)
{
	delay_us(1000*ms);
}

/* timing function variants */
uint32_t millis(){
	return HAL_GetTick();
}



/* calculate offsets and print it on serial Monitor */
void mpu_calculate_offsets(){
  	mpu_start_self_test();
}


void processCommand(rxdata_t datablock){
	//float kp, ki, kd;
	uint16_t var1;
	uint16_t var2;
	uint16_t var3;
	uint16_t var4;
	uint16_t var5;
	uint16_t var6;
	bool sign1;
	bool sign2;
	//bool sign3;
	switch (datablock.command){
	case 255:
		printf("OK\r\n");
		break;

	// Full PID Data - [P higher, P lower, I higher, I lower, D higher, D lower, 0, ...]
	case 1:
		var1 =  (datablock.data[0] << 8) | datablock.data[1];
		var2 =  (datablock.data[2] << 8) | datablock.data[3];
		var3 =  (datablock.data[4] << 8) | datablock.data[5];
		sign1 = datablock.data[6];
		var4 =  (datablock.data[7] << 8) | datablock.data[8];
		var5 =  (datablock.data[9] << 8) | datablock.data[10];
		var6 =  (datablock.data[11] << 8) | datablock.data[12];
		sign2 = datablock.data[13];
		balancing_kp = (float)var1 / 1000;
		balancing_ki = (float)var2 / 1000;
		balancing_kd = (float)var3 / 1000;
		balancing_pid_mirror = sign1;
		speed_kp = (float)var4 / 1000;
		speed_ki = (float)var5 / 1000;
		speed_kd = (float)var6 / 1000;
		speed_pid_mirror = sign2;
		printf("PIDF OK %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %d %d\r\n", balancing_kp, balancing_ki, balancing_kd, speed_kp, speed_ki, speed_kd, balancing_pid_mirror, speed_pid_mirror);
		break;

	/*
	case 5:
		printf("MCO Offsetcalculation started\r\n");
		HAL_Delay(1000);
		mpu_calculate_offsets();
	    HAL_Delay(1000);
		printf("MCO Offsetcalculation in finished\r\n");
	    break;
	*/
	case 6:
		sign1 = datablock.data[0];
		var1 =  (datablock.data[1] << 8) | datablock.data[2];
		if(sign1){
			angle_offset = (float) -var1 / 1000;
		}else{
			angle_offset = (float) var1 / 1000;
		}
		printf("AO OK %0.3f f\r\n", angle_offset);
		break;
	case 7:;	// semicolon needed to not get the label error
		// send back parameter data
		parameterdata_t parameters = {
			.startbyte = 63,
			.typebyte = 3,
			.endbyte = 64
		};
		parameters.angle_offset = round(angle_offset * 1000);
		parameters.balancing_kp = round(balancing_kp * 1000);
		parameters.balancing_ki = round(balancing_ki * 1000);
		parameters.balancing_kd = round(balancing_kd * 1000);
		parameters.balancing_mirror = balancing_pid_mirror;
		parameters.speed_kp = round(speed_kp * 1000);
		parameters.speed_ki = round(speed_ki * 1000);
		parameters.speed_kd = round(speed_kd * 1000);
		parameters.speed_mirror = speed_pid_mirror;

		uint8_t parameter_bytes[sizeof(parameters)];
	    const byte * p = (const byte*) &parameters;
	    unsigned int i;
	    for (i = 0; i < sizeof parameters; i++)
	          parameter_bytes[i] = *p++;
		HAL_UART_Transmit(&huart4, parameter_bytes, sizeof(parameter_bytes), 10);
		break;

	// Driving Data - [throttle sign, throttle higher, throttle lower, steering sign, steering higher, steering lower, 0, ...]
	case 128:
		sign1 = datablock.data[0];
		var1 =  (datablock.data[1] << 8) | datablock.data[2];
		sign2 = datablock.data[3];
		var2 =  (datablock.data[4] << 8) | datablock.data[5];
		if(sign1){
			throttle = (float) -var1 / 1000;
		}else{
			throttle = (float) var1 / 1000;
		}

		if(sign2){
			steering = (float) -var2 / 1000;
		}else{
			steering = (float) var2 / 1000;
		}

		steering = constrain(steering, -MAX_STEERING, MAX_STEERING);
		steering = -steering;	//Mirror steering input
		printf("DC OK %0.3f %0.3f\r\n", throttle, steering);
		break;
	default:
		printf("RECEIVED -> Command: %c, data: %s\r\n", datablock.command, datablock.data);
	}
}

/* ============================ INIT FUNCTIONS =========================== */

/*
 *  Configures MPU and sets its offsets
 */
void initIMU(){
	  if(mpu_config()){
		  printf("MPU error.... Please restart the robot.\r\n");
		  while(1){
			HAL_Delay(50);
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		  }
		  //mpuStatus = ERROR;
	  }else{
		  	printf("MPU Started.... \r\n");
		    //HAL_Delay(10000);
		    //mpu_calculate_offsets();
		    //HAL_Delay(10000);
		    const long accel_reg_biass[3] = {ACC_X_OFFSET, ACC_Y_OFFSET, ACC_Z_OFFSET};
		    mpu_set_accel_bias_6500_reg(accel_reg_biass);
		    long gyro_reg_biass[3] = {GYRO_X_OFFSET, GYRO_Y_OFFSET, GYRO_Z_OFFSET};
		    mpu_set_gyro_bias_reg(gyro_reg_biass);

	  }
}


/* ============================ HANDLE FUNCTIONS =========================== */

/*
 * Handle of the MPU-6050 IMU sensor
 */
void handleIMU(void) {
	uint32_t currentMillis = millis();
	if(currentMillis - imuPreviousMillis >= imuInterval){
		HAL_GPIO_WritePin(TESTPIN_GPIO_Port, TESTPIN_Pin, GPIO_PIN_SET);
		imuPreviousMillis = currentMillis;
		// Get MPU data. Because the MPU-6050 interrupt pin is not wired we have to check DMP data by pooling periodically
		if (mpu_status == SUCCESS ) {
			mpu_get_data();
		} else if (mpu_status == ERROR && imuLoopCounter % 5 == 0) {
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);                   // Blink the LED to signalize error
		}
		HAL_GPIO_WritePin(TESTPIN_GPIO_Port, TESTPIN_Pin, GPIO_PIN_RESET);
	}
}


void initPID(void){
	balancing_kp = BALANCING_KP;
	balancing_ki = BALANCING_KI;
	balancing_kd = BALANCING_KD;
	balancing_pid_mirror = BALANCING_DIR;

	speed_kp = SPEED_KP;
	speed_ki = SPEED_KI;
	speed_kd = SPEED_KD;
	speed_pid_mirror = SPEED_DIR;

	angle_offset = ANGLE_OFFSET;
}


/* Balancing PID Calculation function */
float calcSpeedPID(float input, float setpoint){
	HAL_GPIO_WritePin(TESTPIN3_GPIO_Port, TESTPIN3_Pin, GPIO_PIN_SET);
	unsigned long now = HAL_GetTick();
	unsigned long time_change = (now - speed_last_calculation);
	speed_last_calculation = now;
	float error = setpoint - input;	// Error = wanted value - current value
	// Calculate the first time only with P cause of lack of lastCalculation
	if(speed_pid_firstrun){
		// only use P
		speed_pid_firstrun = false;
		speed_p_term = balancing_kp * error;
		speed_last_error = error;
	}else{
		// Use full PID
		speed_p_term = (float)speed_kp * error;
		speed_i_term += (float)speed_ki * (error * (time_change*0.001));
		speed_d_term = (float)speed_kd * ((error - speed_last_error) / (time_change*0.001));
		speed_last_error = error;
	}
	float output = 0;

	//Mirror PID Output if needed
	if(speed_pid_mirror){
		output = - (speed_p_term + speed_i_term + speed_d_term);
	}else{
		output = speed_p_term + speed_i_term + speed_d_term;
	}

	// Fill the display variables
	speed_input_disp = input;
	speed_output_disp = output;
	speed_setpoint_disp = setpoint;
	HAL_GPIO_WritePin(TESTPIN3_GPIO_Port, TESTPIN3_Pin, GPIO_PIN_RESET);
	return output;
}


/* Balancing PID Calculation function */
float calcBalancingPID(float input, float setpoint){
	HAL_GPIO_WritePin(TESTPIN4_GPIO_Port, TESTPIN4_Pin, GPIO_PIN_SET);
	unsigned long now = HAL_GetTick();
	unsigned long time_change = (now - balancing_last_calculation);
	balancing_last_calculation = now;
	float error = setpoint - input;	// Error = wanted value - current value
	// Calculate the first time only with P cause of lack of lastCalculation
	if(balancing_pid_firstrun){
		// only use P
		balancing_pid_firstrun = false;
		balancing_p_term = balancing_kp * error;
		balancing_last_error = error;
	}else{
		// Use full PID
		balancing_p_term = (float)balancing_kp * error;
		balancing_i_term += (float)balancing_ki * (error * (time_change*0.001));
		balancing_d_term = (float)balancing_kd * ((error - balancing_last_error) / (time_change*0.001));
		balancing_last_error = error;
	}
	float output = 0;

	//Mirror PID Output if needed
	if(balancing_pid_mirror){
		output = - (balancing_p_term + balancing_i_term + balancing_d_term);
	}else{
		output = balancing_p_term + balancing_i_term + balancing_d_term;
	}

	// Fill the display variables
	balancing_input_disp = input;
	balancing_output_disp = output;
	balancing_setpoint_disp = setpoint;
	HAL_GPIO_WritePin(TESTPIN4_GPIO_Port, TESTPIN4_Pin, GPIO_PIN_RESET);
	return output;
}

/*
 * Handle of the motors, balancing, etc...
 */
void handleMovement(void) {
	uint32_t currentMillis = millis();
	if(currentMillis - motorsPreviousMillis >= movementInterval){
		HAL_GPIO_WritePin(TESTPIN2_GPIO_Port, TESTPIN2_Pin, GPIO_PIN_SET);
		motorsPreviousMillis = currentMillis;

		if(drivemode == off)
		{
			float speed = -throttle;
			motor_left_speed = speed + steering;
			motor_right_speed = speed - steering;
			setSpeedRPS(motor_left_speed, motor_right_speed);
		}
		else if(drivemode == driving)
		{
			angle_before = angle_actual;
			angle_actual = (float)mpu.euler.pitch/100 + angle_offset;
			speed_before = speed_actual;
			speed_actual = (motor_left_speed + motor_right_speed) / 2;

			// Calc both PIDs
			target_angle = calcSpeedPID(speed_actual, throttle);
			target_angle = constrain(target_angle, -MAX_TILT, MAX_TILT);
			if(throttle != 0){
				motor_speed  =  calcBalancingPID(angle_actual, target_angle);
			}else{
				motor_speed = calcBalancingPID(angle_actual, 0);
			}
			motor_speed = constrain(motor_speed,-MAX_RPS, MAX_RPS );
			// GAP where motors stand still,
			if(fabs(motor_speed) < 0.05){
				motor_speed = 0;
			}

			// fall detection
			if(abs((int)angle_actual) > TILT_DEACTIVATE){
				// fall over -> deactivate Steppers
				fallen = 1;
				disableSteppers();
				setSpeedRPS(0,  0);
				balancing_i_term = 0;	// Reset I-Term of Balancing-PID
				speed_i_term = 0;	    // Reset I-Term of Balancing-PID
			}else{
				// reactivate steppers after robot is put up again
				if(fallen == 1 && abs((int)angle_actual) < TILT_ACTIVATE){
					fallen = 0;
					enableSteppers();
				}
				// Normal driving
				if(fallen == 0){
					motor_left_speed = motor_speed + steering;
					motor_right_speed = motor_speed - steering;
					setSpeedRPS(motor_left_speed, motor_right_speed);
				}
			}
		}
		else if(drivemode == driving_yabr)
		{
			float max_target_angle = 0.5;
			float max_motor_speed = 1;
			float steer_tmp = 0;
			angle_actual = (float)mpu.euler.pitch/100 + angle_offset;

			motor_speed  =  calcBalancingPID(angle_actual, target_angle);
			motor_speed = constrain(motor_speed, -(MAX_RPS - MAX_STEERING), (MAX_RPS - MAX_STEERING));
			if(abs((int)angle_actual) > TILT_DEACTIVATE){
				// fall over -> deactivate Steppers
				fallen = 1;
				if(throttle == 0 && steering == 0){
					disableSteppers();
					motor_left_speed = 0;
					motor_right_speed = 0;
					setSpeedRPS(motor_left_speed, motor_right_speed);
				}else{
					enableSteppers();
					float speed = -throttle;//constrain(throttle, -(MAX_RPS - MAX_STEERING), (MAX_RPS - MAX_STEERING));
					motor_left_speed = speed + steering;
					motor_right_speed = speed - steering;
					setSpeedRPS(motor_left_speed, motor_right_speed);

				}
			}else{
				// reactivate steppers after robot is put up again
				if(fallen == 1 && abs((int)angle_actual) < TILT_ACTIVATE){
					fallen = 0;
					enableSteppers();
				}
				// Normal driving
				if(fallen == 0){
					if(throttle > 0.5){
						if(motor_speed < throttle){
							if(target_angle < max_target_angle){
								target_angle += 0.025;
							}
						}
						if(motor_speed > max_motor_speed){
							target_angle -= 0.025;
						}
						rounds_since_control = 0;
						brake_needed = true;
					}

					if(throttle < -0.5){

						if(motor_speed > throttle){
							if(target_angle > -max_target_angle){
								target_angle -= 0.025;
							}
						}
						if(motor_speed < -max_motor_speed){
							target_angle += 0.025;
						}
						rounds_since_control = 0;
						brake_needed = true;
					}

					if(throttle <= 0.5 && throttle >= -0.5){
						if(brake_needed){
							// angle is opposite sign as motor output so:
							if(motor_speed > 0.25){
								target_angle += 0.025;
							}if(motor_speed < -0.25){
								target_angle -= 0.025;
							}else{
								target_angle = 0;
								brake_needed = false;
							}
						}
						/*if(target_angle > 0.25){
							target_angle -= 0.025;
						}else if(target_angle < -0.25){
							target_angle += 0.025;
						}else{
							target_angle = 0;
						}
						*/
					}

					if(steering > 0.5){
						steer_tmp = 0.5;
						rounds_since_control = 0;
					}

					if(steering < -0.5){
						steer_tmp = -0.5;
						rounds_since_control = 0;
					}

					if(target_angle == 0){
						if(rounds_since_control >= 1000){	// wait 5s since last control input before adjust the angle offset
							if(motor_speed > 0.25){
								angle_offset += 0.001;
							}
							else if(motor_speed < -0.25){
								angle_offset -= 0.001;
							}
						}else{
							rounds_since_control += 1;
						}
					}

					motor_left_speed = motor_speed + steer_tmp;
					motor_right_speed = motor_speed - steer_tmp;
					setSpeedRPS(motor_left_speed, motor_right_speed);
				}
			}

		}else if(drivemode == driving_stefan)
		{
			float target_step = 0.001;
			float target_down_multiplicator = 1;
			float offset_step = 0.001;
			float max_target_angle = 2;
			float min_throttle = 0.25;
			float max_throttle = 1;
			float steer_tmp = 0;
			angle_actual = (float)mpu.euler.pitch/100 + angle_offset;
			motor_speed  =  calcBalancingPID(angle_actual, target_angle);
			motor_speed = constrain(motor_speed, -(MAX_RPS - MAX_STEERING), (MAX_RPS - MAX_STEERING));

			//Fallerkennung
			if(abs((int)angle_actual) > TILT_DEACTIVATE){
				// fall over -> deactivate Steppers
				fallen = 1;
				if(throttle == 0 && steering == 0){
					disableSteppers();
					motor_left_speed = 0;
					motor_right_speed = 0;
					setSpeedRPS(motor_left_speed, motor_right_speed);
				}else{
					// Steuerbarkeit beim Fall
					enableSteppers();
					float speed = throttle;//constrain(throttle, -(MAX_RPS - MAX_STEERING), (MAX_RPS - MAX_STEERING));
					motor_left_speed = speed + steering;
					motor_right_speed = speed - steering;
					setSpeedRPS(motor_left_speed, motor_right_speed);

				}
			}else{
				// reactivate steppers after robot is put up again
				if(fallen == 1 && abs((int)angle_actual) < TILT_ACTIVATE){
					fallen = 0;
					enableSteppers();
				}
				// Normal driving
				if(fallen == 0){
					//Autocalibrate
					if(target_angle == 0){
						rounds_since_control++;
						if(rounds_since_control >= 1000){	// wait 5s since last control input before adjust the angle offset.. one loop = 5ms
							if(motor_speed > 0){
								angle_offset -= offset_step;
							}
							if(motor_speed < 0){
								angle_offset += offset_step;
							}
						}
					}

					//throttle Control
					if(throttle > min_throttle){		// Forward
						brake_needed = true;
						rounds_since_control = 0;
						if(motor_speed < max_throttle){
							if(target_angle < max_target_angle){
								target_angle += target_step;
							}
						}else{
							target_angle -= target_step * target_down_multiplicator;
						}
					}else if(throttle < -min_throttle){ //Backward
						rounds_since_control = 0;
						brake_needed = true;
						if(motor_speed > -max_throttle){
							if(target_angle > -max_target_angle){
								target_angle -= target_step;
							}
						}else{
							target_angle += target_step * target_down_multiplicator;
						}
					}else{	// Standing still
						if(brake_needed){
							if(motor_speed > 0.1){
								target_angle -= target_step;
							}else if(motor_speed < -0.1){
								target_angle += target_step;
							}else{
								target_angle = 0;
								brake_needed = false;
							}
						}
					}


					motor_left_speed = motor_speed;
					motor_right_speed = motor_speed;
					setSpeedRPS(motor_left_speed, motor_right_speed);
				}
			}

		}
		HAL_GPIO_WritePin(TESTPIN2_GPIO_Port, TESTPIN2_Pin, GPIO_PIN_RESET);
	}
}



/*
 * Handle of the debug serial print
 * Fills the telemetry struct, serialize it and send it over uart
 */
void handleTelemetry(void) {
	uint32_t currentMillis = millis();
	if(currentMillis - telemetryPreviousMillis >= telemetryInterval){
		HAL_GPIO_WritePin(TESTPIN5_GPIO_Port, TESTPIN5_Pin, GPIO_PIN_SET);
		telemetryPreviousMillis = currentMillis;
		if(USE_SMALL_TELEMETRY){
			smalltelemetrydata_t telemetry = {
					.startbyte = 63,
					.typebyte =  2,
					.endbyte = 	64
			};
			telemetry.speed_actual = round(speed_actual*100);
			telemetry.angle_actual = round(angle_actual*100);
			telemetry.motor_right = round(motor_right_speed*100);
			telemetry.motor_left = round(motor_left_speed*100);
			telemetry.fallen = fallen;

			telemetry.throttle = round(throttle*100);
			telemetry.steering = round(steering*100);

			telemetry.yaw =   mpu.euler.yaw;
			telemetry.pitch = mpu.euler.pitch;
			telemetry.roll =  mpu.euler.roll;


			uint8_t telemetry_bytes[sizeof(telemetry)];
		    const byte * p = (const byte*) &telemetry;
		    unsigned int i;
		    for (i = 0; i < sizeof telemetry; i++)
		          telemetry_bytes[i] = *p++;
			HAL_UART_Transmit(&huart4, telemetry_bytes, sizeof(telemetry_bytes), 10);
		}else{
			telemetrydata_t telemetry = {
					.startbyte = 63,
					.typebyte = 1,
					.endbyte = 64
			};

			telemetry.balancing_pid_in = round(balancing_input_disp * 100);
			telemetry.balancing_pid_out  = round(balancing_output_disp * 100);
			telemetry.balancing_pid_set  = round(balancing_setpoint_disp * 100);
			//telemetry.balancing_pterm = round(balancing_p_term * 100);
			//telemetry.balancing_iterm = round(balancing_i_term * 100);
			//telemetry.balancing_dterm = round(balancing_d_term * 100);
			telemetry.speed_pid_in = round(speed_input_disp * 100);
			telemetry.speed_pid_out  = round(speed_output_disp * 100);
			telemetry.speed_pid_set  = round(speed_setpoint_disp * 100);
			//telemetry.speed_pterm = round(speed_p_term * 100);
			//telemetry.speed_iterm = round(speed_i_term * 100);
			//telemetry.speed_dterm = round(speed_d_term * 100);

			telemetry.speed_actual = round(speed_actual*100);
			telemetry.angle_actual = round(angle_actual*100);
			telemetry.motor_right = round(motor_right_speed*100);
			telemetry.motor_left = round(motor_left_speed*100);
			telemetry.fallen = fallen;

			telemetry.throttle = round(throttle*100);
			telemetry.steering = round(steering*100);

			telemetry.yaw = mpu.euler.yaw;
			telemetry.pitch = mpu.euler.pitch;
			telemetry.roll = mpu.euler.roll;
			telemetry.gyroX = mpu.gyro.x;
			telemetry.gyroY = mpu.gyro.y;
			telemetry.gyroZ = mpu.gyro.z;
			telemetry.accX = mpu.accel.x;
			telemetry.accY = mpu.accel.y;
			telemetry.accZ = mpu.accel.z;

			uint8_t telemetry_bytes[sizeof(telemetry)];
		    const byte * p = (const byte*) &telemetry;
		    unsigned int i;
		    for (i = 0; i < sizeof telemetry; i++)
		          telemetry_bytes[i] = *p++;
			HAL_UART_Transmit(&huart4, telemetry_bytes, sizeof(telemetry_bytes), 5);
		}
		HAL_GPIO_WritePin(TESTPIN5_GPIO_Port, TESTPIN5_Pin, GPIO_PIN_RESET);
	}
}


/* Things from hoverboard-sideboard-hack - few things needed to keep the IMU lib useable */
/*
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

/* =========================== General Functions =========================== */

void consoleLog(char *message)
{
    #ifdef SERIAL_DEBUG
        log_i("%s", message);
    #endif
}


void get_tick_count_ms(unsigned long *count)
{
    *count = HAL_GetTick();
}
/* =========================== I2C WRITE Functions =========================== */

/*
 * write bytes to chip register
 */
int8_t i2c_writeBytes(uint8_t slaveAddr, uint8_t regAddr, uint8_t length, uint8_t *data)
{
    // !! Using the I2C Interrupt will fail writing the DMP.. could be that DMP memory writing requires more time !! So use the I2C without interrupt.
    // HAL_I2C_Mem_Write_IT(&hi2c1, slaveAddr << 1, regAddr, 1, data, length);
    // while(HAL_I2C_STATE_READY != HAL_I2C_GetState(&hi2c1));                     // Wait until all data bytes are sent/received
    // return 0;

    return HAL_I2C_Mem_Write(&hi2c1, slaveAddr << 1, regAddr, 1, data, length, 100);    // Address is shifted one position to the left. LSB is reserved for the Read/Write bit.

}

/*
 * write 1 byte to chip register
 */
int8_t i2c_writeByte(uint8_t slaveAddr, uint8_t regAddr, uint8_t data)
{
    return i2c_writeBytes(slaveAddr, regAddr, 1, &data);
}

/*
 * write one bit to chip register
 */
int8_t i2c_writeBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data) {
    uint8_t b;
    i2c_readByte(slaveAddr, regAddr, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return i2c_writeByte(slaveAddr, regAddr, b);
}

/* =========================== I2C READ Functions =========================== */

/*
 * read bytes from chip register
 */
int8_t i2c_readBytes(uint8_t slaveAddr, uint8_t regAddr, uint8_t length, uint8_t *data)
{
    // !! Using the I2C Interrupt will fail writing the DMP.. could be that DMP memory writing requires more time !! So use the I2C without interrupt.
    // HAL_I2C_Mem_Read(&hi2c1, slaveAddr << 1, regAddr, 1, data, length);
    // while(HAL_I2C_STATE_READY != HAL_I2C_GetState(&hi2c1));                   // Wait until all data bytes are sent/received
    // return 0;

    return HAL_I2C_Mem_Read(&hi2c1, slaveAddr << 1, regAddr, 1, data, length, 100);     // Address is shifted one position to the left. LSB is reserved for the Read/Write bit.

}

/*
 * read 1 byte from chip register
 */
int8_t i2c_readByte(uint8_t slaveAddr, uint8_t regAddr, uint8_t *data)
{
    return i2c_readBytes(slaveAddr, regAddr, 1, data);
}

/*
 * read 1 bit from chip register
 */
int8_t i2c_readBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data)
{
    uint8_t b;
    int8_t status = i2c_readByte(slaveAddr, regAddr, &b);
    *data = b & (1 << bitNum);
    return status;
}
