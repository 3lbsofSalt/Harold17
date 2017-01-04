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

//Encoder Globals
Encoder rEnc;
Encoder lEnc;
Encoder liftEnc;
Encoder clawEnc;
//Limit Switch Constant
const int launcherPosIn = 1;

//Motor Port Constants
const int leftBack = 9;
const int rightBack = 10;
const int leftCatapultTop = 3;
const int leftCatapultBottom = 4;
const int leftLift = 5;
const int rightLift = 6;
const int rightCatapultBottom = 7;
const int rightCatapultTop = 8;
const int rightFront = 1;
const int leftFront = 2;

const int leftSolenoid = 9;
const int rightSolenoid = 8;

const int shotLimit = 1;

//////////////////////////
// Function Prototypes	//
//////////////////////////
void move();
void turn();
void lift();
void launch();
void claw();


void autonomous() {


	//Initalize encoders if not already set
	if(!lEnc){
		lEnc = encoderInit(2, 3, 1);
	}
	if(!rEnc){
		rEnc = encoderInit(4, 5, 1);
	}
	if(!liftEnc) {
		liftEnc = encoderInit(6, 7, 0);
	}
	if(!clawEnc){
		clawEnc = encoderInit(8, 9, 0);
	}

	//Reset after initialization
	encoderReset(lEnc);
	encoderReset(rEnc);


	lift(120);
	motorSet(leftLift, 50);
	motorSet(rightLift, -50);
	move(760, 1);
	delay(3000);
	motorSet(leftLift, 0);
	motorSet(rightLift, 0);
}

void claw(int posit) {
	if(posit == 1){
		digitalWrite(leftSolenoid, LOW);
		digitalWrite(rightSolenoid, LOW);
	} else if(posit == 0) {
		digitalWrite(leftSolenoid, HIGH);
		digitalWrite(rightSolenoid, HIGH);
	}
}

void launch() {
	motorSet(leftCatapultTop, -127);
	motorSet(leftCatapultBottom, -127);
	motorSet(rightCatapultTop, 127);
	motorSet(rightCatapultBottom, 127);

	while(1){
		if(digitalRead(shotLimit) == LOW){
			break;
		}
	}

	motorSet(leftCatapultTop, 0);
	motorSet(leftCatapultBottom, 0);
	motorSet(rightCatapultTop, 0);
	motorSet(rightCatapultBottom, 0);
}

/**
 * Turns Robot
 *
 * @param dist is distance in turn
 * @param dir is direction 1 is left and 0 right default right
 */
void turn(int dist, int dir){
	lcdPrint(uart1, 1, "DA!");
	encoderReset(lEnc);
	encoderReset(rEnc);
	lcdPrint(uart1, 1, "%d", dir);
	if(dir == 1){
		motorSet(leftBack, 110);
		motorSet(rightBack, 110);
		motorSet(leftFront, 110);
		motorSet(rightFront, 110);
	} else if(dir !=  1) {
		motorSet(leftBack, -110);
		motorSet(rightBack, -110);
		motorSet(leftFront, -110);
		motorSet(rightFront, -110);
	}
	while(encoderGet(lEnc) <= dist && encoderGet(rEnc) <= dist){
		lcdPrint(uart1, 1, "B:%d", encoderGet(lEnc));
	}
	motorSet(leftBack, 0);
	motorSet(rightBack, 0);
	motorSet(leftFront, 0);
	motorSet(rightFront, 0);
}

void lift(int height) {
	if(encoderGet(liftEnc) < height){
		motorSet(leftLift, 127);
		motorSet(rightLift, -127);
		while(encoderGet(liftEnc) < height){

		}
	} else if(encoderGet(liftEnc) > height){
		motorSet(leftLift, -127);
		motorSet(rightLift, 127);
		while(encoderGet(liftEnc) > height){

		}
	}
	motorSet(leftLift, 0);
	motorSet(rightLift, 0);
}



/**
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
		motorSet(leftBack, -110);
		motorSet(rightBack, 110);
		motorSet(leftFront, -110);
		motorSet(rightFront, 110);
	}else {				//Forward
		motorSet(leftBack, 110);
		motorSet(rightBack, -110);
		motorSet(leftFront, 110);
		motorSet(rightFront, -110);
	}

	//Run while distance is being traveled
	while(1) {
		//Break waiting Loop when distance is reached
		if(abs(encoderGet(lEnc)) >= dist || abs(encoderGet(rEnc)) >= dist){
			break;
		}
	}

	//Stop
	motorSet(leftBack, 0);
	motorSet(rightBack, 0);
	motorSet(leftFront, 0);
	motorSet(rightFront, 0);
}
