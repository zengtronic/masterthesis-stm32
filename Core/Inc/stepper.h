/*
 * stepper.h
 *
 *  Created on: Nov 2, 2021
 *      Author: Stefan
 */

#ifndef INC_STEPPER_H_
#define INC_STEPPER_H_


void enableSteppers();

void disableSteppers();

void printStepVars();

// calculates the timings and sets the Direction if neccessary
void setSpeedRPS(float A, float B);

// set speed in RPM
void setSpeedRPM(float A, float B);

// This function gets called by the timer interrupt
void stepperISR();




#endif /* INC_STEPPER_H_ */
