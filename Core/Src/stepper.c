/*
 * stepper.c
 *	This functions should control the stepper motors via RPM or RPS
 *  Created on: Oct 25, 2021
 *      Author: Stefan Zengerle
 */


#include "stepper.h"
#include "main.h"	// for GPIO DEFINITIONS
#include <stdio.h>
#include <stdbool.h>
#include "config.h"
#include "util.h"


uint16_t speedAdjustmentLoop = 0;
uint16_t speedAdjustmentDivider = 50; 	// = every 1ms speed gets adjusted
float 	 speedAdjustmentStep = 0.05;

uint16_t stepsToWaitA = 0;
int16_t currentStepA = 0;
uint16_t stepsToWaitB = 0;
int16_t currentStepB = 0;
float speedGoalA = 0.0;
float speedCurrentA = 0.0;
float speedGoalB = 0.0;
float speedCurrentB = 0.0;
float tmpSpeedA = 0.0;
float tmpSpeedB = 0.0;

/* Enable Motors */
void enableSteppers(){
	HAL_GPIO_WritePin(MOT_ENA_GPIO_Port, MOT_ENA_Pin, GPIO_PIN_RESET);

}

/* Disable Motors */
void disableSteppers(){
	HAL_GPIO_WritePin(MOT_ENA_GPIO_Port, MOT_ENA_Pin, GPIO_PIN_SET);
	speedCurrentA = 0;
	speedCurrentB = 0;
}

/* Calculates the timings and sets the Direction */
void setSpeed(float A, float B){
	float tempA, tempB;
	// Motor A
	if(A != 0){
		if(A < 0){
			if(-A > MAX_RPS)
				A = -MAX_RPS;
			tempA = -A;
			HAL_GPIO_WritePin(MOTA_DIR_GPIO_Port, MOTA_DIR_Pin, GPIO_PIN_RESET);
		}else{
			if(A > MAX_RPS)
				A = MAX_RPS;
			tempA = A;
			HAL_GPIO_WritePin(MOTA_DIR_GPIO_Port, MOTA_DIR_Pin, GPIO_PIN_SET);
		}

		uint16_t stepPerSecondA = STEPS_PER_ROUND * tempA;
		uint16_t timePerStepA = 1000000 / stepPerSecondA;
		stepsToWaitA = timePerStepA / 20;					// Somehow crashes if US_PER_IR is used

	}else{
		stepsToWaitA = 0;
	}

	// Motor B
	if(B != 0){
		// The GPIO Pins are mirrored to A because the B Motor is mirrored to A
		if(B < 0){
			if(-B > MAX_RPS)
				B = -MAX_RPS;
			tempB = -B;
			HAL_GPIO_WritePin(MOTB_DIR_GPIO_Port, MOTB_DIR_Pin, GPIO_PIN_SET);
		}else{
			if(B > MAX_RPS)
				B = MAX_RPS;
			tempB = B;
			HAL_GPIO_WritePin(MOTB_DIR_GPIO_Port, MOTB_DIR_Pin, GPIO_PIN_RESET);
		}
		uint16_t stepPerSecondB = STEPS_PER_ROUND * tempB;
		uint16_t timePerStepB = 1000000 / stepPerSecondB;
		stepsToWaitB = timePerStepB / 20;
	}else{
		stepsToWaitB = 0;
	}

}

void printStepVars(){
	printf("Step A: %d\r\n", currentStepA);
	printf("Goal A: %d\r\n", stepsToWaitA);
	printf("Step B: %d\r\n", currentStepB);
	printf("Goal B: %d\r\n", stepsToWaitB);
}

void setSpeedRPS(float A, float B){
	speedGoalA = A;
	speedGoalB = B;
}

// set speed in RPM
void setSpeedRPM(float A, float B){
	setSpeedRPS(A/60, B/60);
}

/* This function gets called by the timer interrupt and fires a step impulse if needed*/
void stepperISR(){
	speedAdjustmentLoop++;
	if(speedAdjustmentLoop >= speedAdjustmentDivider){
		uint8_t speedChanged = 1;
		// Speed adjustment code
		if(speedCurrentA < speedGoalA){
			speedCurrentA += speedAdjustmentStep;
		}else if(speedCurrentA > speedGoalA){
			speedCurrentA -= speedAdjustmentStep;
		}else{
			speedChanged = 0;
		}

		if(speedCurrentB < speedGoalB){
			speedCurrentB += speedAdjustmentStep;
		}else if(speedCurrentB > speedGoalB){
			speedCurrentB -= speedAdjustmentStep;
		}else{
			speedChanged = 0;
		}

		if(speedChanged == 1){
			setSpeed(speedCurrentA, speedCurrentB);
		}
		speedAdjustmentLoop = 0;
	}




	// Set currentStep negative so it never reaches stepsToWait if it is 0
	if(stepsToWaitA != 0){
		currentStepA++;
	}else{
		currentStepA = -1;	// -1 = it should never step
	}
	// Same for the second motor
	if(stepsToWaitB != 0){
		currentStepB++;
	}else{
		currentStepB = -1;
	}

	//Step if its necessary
	if((currentStepA >= stepsToWaitA) && (currentStepB >= stepsToWaitB)){
		// Step both
		HAL_GPIO_WritePin(MOTA_STEP_GPIO_Port, MOTA_STEP_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MOTB_STEP_GPIO_Port, MOTB_STEP_Pin, GPIO_PIN_SET);
		delay_us(1);
		HAL_GPIO_WritePin(MOTA_STEP_GPIO_Port, MOTA_STEP_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MOTB_STEP_GPIO_Port, MOTB_STEP_Pin, GPIO_PIN_RESET);
		currentStepA = 0;
		currentStepB = 0;
	}else if((currentStepA >= stepsToWaitA) && (currentStepB < stepsToWaitB)){
		// Step A only
		HAL_GPIO_WritePin(MOTA_STEP_GPIO_Port, MOTA_STEP_Pin, GPIO_PIN_SET);
		delay_us(1);
		HAL_GPIO_WritePin(MOTA_STEP_GPIO_Port, MOTA_STEP_Pin, GPIO_PIN_RESET);
		currentStepA = 0;
	}else if((currentStepA < stepsToWaitA) && (currentStepB >= stepsToWaitB)){
		// Step B only
		HAL_GPIO_WritePin(MOTB_STEP_GPIO_Port, MOTB_STEP_Pin, GPIO_PIN_SET);
		delay_us(1);
		HAL_GPIO_WritePin(MOTB_STEP_GPIO_Port, MOTB_STEP_Pin, GPIO_PIN_RESET);
		currentStepB = 0;
	}
}

