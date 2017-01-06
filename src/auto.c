/** @file auto.c
 * @brief File for autonomous code
 *
 * This file should contain the user autonomous() function and any functions related to it.
 *
 * Copyright (c) 2011-2014, Purdue University ACM SIG BOTS.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Purdue University ACM SIG BOTS nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL PURDUE UNIVERSITY ACM SIG BOTS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Purdue Robotics OS contains FreeRTOS (http://www.freertos.org) whose source code may be
 * obtained from http://sourceforge.net/projects/freertos/files/ or on request.
 */

#include "main.h"

/*
 * Runs the user autonomous code. This function will be started in its own task with the default
 * priority and stack size whenever the robot is enabled via the Field Management System or the
 * VEX Competition Switch in the autonomous mode. If the robot is disabled or communications is
 * lost, the autonomous task will be stopped by the kernel. Re-enabling the robot will restart
 * the task, not re-start it from where it left off.
 *
 * Code running in the autonomous task cannot access information from the VEX Joystick. However,
 * the autonomous function can be invoked from another task if a VEX Competition Switch is not
 * available, and it can access joystick information if called in this way.
 *
 * The autonomous task may exit, unlike operatorControl() which should never exit. If it does
 * so, the robot will await a switch to another mode or disable/enable cycle.
 */



//Motor Port Constants
const int rightBackDrive = 10;
const int rightFrontDrive = 9;
const int rightLiftInner = 8;
const int rightLiftOuter = 7;
const int rightClaw = 6;
const int leftClaw = 5;
const int leftLiftOuter = 4;
const int leftLiftInner = 3;
const int leftFrontDrive = 2;
const int leftBackDrive = 1;

const int clawPot = 1;

//////////////////////////
// Function Prototypes	//
//////////////////////////
void move();
void turn();
void PID();
void lift();

//lift globals
int liftHeight;
int currHeight;
int liftSpeed;

//Encoder Globals
Encoder rEnc;
Encoder lEnc;
Encoder liftEnc;
Encoder clawEnc;

void autonomous() {

	lcdInit(uart1);
	lcdSetBacklight(uart1, true);

	//Initalize encoders if not already set
	if(!lEnc){
		lEnc = encoderInit(1, 2, 0);
	}
	if(!rEnc){
		rEnc = encoderInit(3, 4, 0);
	}
	if(!liftEnc) {
		liftEnc = encoderInit(5, 6, 0);
	}

	//Reset after initialization
	encoderReset(lEnc);
	encoderReset(rEnc);

	liftHeight = encoderGet(liftEnc);

	turn(50, 0);

}

/**
 * turn()
 * Turns Robot
 *
 * @param dist is distance in turn
 * @param dir is direction 1 is left and 0 right default right
 */
void turn(int dist, int dir){
	encoderReset(lEnc);
	encoderReset(rEnc);
	if(dir == 1){
		motorSet(leftBackDrive, 110);
		motorSet(rightBackDrive, 110);
		motorSet(leftFrontDrive, 110);
		motorSet(rightFrontDrive, 110);
	} else if(dir !=  1) {
		motorSet(leftBackDrive, -110);
		motorSet(rightBackDrive, -110);
		motorSet(leftFrontDrive, -110);
		motorSet(rightFrontDrive, -110);
	}
	while(encoderGet(lEnc) <= dist && encoderGet(rEnc) <= dist){
		lcdPrint(uart1, 1, "B:%d", encoderGet(lEnc));
	}
	motorSet(leftBackDrive, 0);
	motorSet(rightBackDrive, 0);
	motorSet(leftFrontDrive, 0);
	motorSet(rightFrontDrive, 0);
}

/**
 * move()
 * Moves the robot forward or reverse for a set distance
 *
 * @param dist the distance in encoder ticks that the robot will travel
 * @param reverse if set to 1, the robot reverses, otherwise it moves forward;
 */
void move(int dist, int reverse){
	encoderReset(lEnc);
	encoderReset(rEnc);

	//Forward/Reverse statements
	if(reverse == 1){ 	//Reverse
		motorSet(leftBackDrive, -110);
		motorSet(rightBackDrive, 110);
		motorSet(leftFrontDrive, -110);
		motorSet(rightFrontDrive, 110);
	}else {				//Forward
		motorSet(leftBackDrive, 110);
		motorSet(rightBackDrive, -110);
		motorSet(leftFrontDrive, 110);
		motorSet(rightFrontDrive, -110);
	}


	//Run while distance is being traveled
	while(encoderGet(lEnc) <= dist && encoderGet(rEnc) <= dist) {
		lcdPrint(uart1, 1, "Left: %d", encoderGet(lEnc));
		lcdPrint(uart1, 2, "Right: %d", encoderGet(rEnc));
	}

	//Stop
	motorSet(leftBackDrive, 0);
	motorSet(rightBackDrive, 0);
	motorSet(leftFrontDrive, 0);
	motorSet(rightFrontDrive, 0);
}

/**
 * PID()
 * Controls the lift through a PID funtion.
 * Insert into every user function
 */
void PID() {
	currHeight = encoderGet(liftEnc);

	if(currHeight < liftHeight){

	}
}

/**
 * lift()
 * Moves the lift, and changes the value that PID() uses to control the position
 *
 * @param height denotes the height, in encoder ticks, that the lift will go to.
 */
void lift(int height) {
	liftHeight = height;
	if(encoderGet(liftEnc) < height) {
		motorSet(leftLiftInner, 110);
		motorSet(leftLiftOuter, -110);
		motorSet(rightLiftInner, -110);
		motorSet(rightLiftOuter, 110);
		while(encoderGet(liftEnc) < height) {

		}
	} else if (encoderGet(liftEnc) > height) {
		motorSet(leftLiftInner, -110);
		motorSet(leftLiftOuter, 110);
		motorSet(rightLiftInner, 110);
		motorSet(rightLiftOuter, -110);
		while(encoderGet(liftEnc) > height){

		}
	}

	motorSet(leftLiftInner, 0);
	motorSet(leftLiftOuter, 0);
	motorSet(rightLiftInner, 0);
	motorSet(rightLiftOuter, 0);
}
