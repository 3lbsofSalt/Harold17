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


Encoder fEnc;
Encoder bEnc;

const int launcherPosIn = 1;

const int leftBack = 1;     	//+ Forward
const int rightBack = 2;    	//+ Forward
const int leftLaunch1 = 3;
const int leftLaunch2 = 4;
const int leftLaunch3 = 5;
const int rightLaunch3 = 6;
const int rightLaunch2 = 7;
const int rightLaunch1 = 8;
const int rightFront = 9;		//- Forward
const int leftFront = 10;		//- Forward

void move();
void launchMove();

void autonomous() {


	if(!bEnc){
		bEnc = encoderInit(2, 3, 1);
	}
	if(!fEnc){
		fEnc = encoderInit(4, 5, 1);
	}

	encoderReset(bEnc);
	encoderReset(fEnc);

	move(250);
	launchMove(100, 0);
}


/**
 * Moves the robot forward or reverse for a set distance
 *
 * @param dist the distance in encoder ticks that the robot will travel
 * @param reverse if set to 1, the robot reverses, otherwise it moves forward;
 */
void move(int dist, int reverse){
	encoderReset(bEnc);
	encoderReset(bEnc);

	if(reverse == 1){
		motorSet(leftBack, 110);
		motorSet(rightBack, 110);
		motorSet(leftFront, -110);
		motorSet(rightFront, -110);
	}else {
		motorSet(leftBack, 110);
		motorSet(rightBack, 110);
		motorSet(leftFront, -110);
		motorSet(rightFront, -110);
	}
	while(1) {
		if(abs(encoderGet(bEnc)) >= dist && abs(encoderGet(fEnc)) >= dist){
			break;
		}
	}
	motorSet(leftBack, 0);
	motorSet(rightBack, 0);
	motorSet(leftFront, 0);
	motorSet(rightFront, 0);
}

/*
 * Launches the launcher once, robot is able to move while reloading as well
 *
 * @param dist how far you want the robot to move after launch, any positive number, if not moving put 0
 * @param reverse if you wish the robot to have a reverse move after launch, insert any value if not moving
 */
void launchMove(int dist, int reverse){
	encoderReset(bEnc);
	encoderReset(bEnc);
	int motorRun = 0;
	if(dist != 0){
		motorRun = 1;
	}

	motorSet(rightLaunch1, -127);
	motorSet(rightLaunch2, 127);
	motorSet(rightLaunch3, -127);
	motorSet(leftLaunch1, 127);
	motorSet(leftLaunch2, -127);
	motorSet(leftLaunch3, 127);
	int limit = 1;

	while(1){
		if(reverse == 1 && dist != 0 && limit != 1){
			motorSet(leftBack, 110);
			motorSet(rightBack, 110);
			motorSet(leftFront, -110);
			motorSet(rightFront, -110);
			motorRun = 1;
		} else if(reverse == 0 && dist != 0 && limit != 1) {
			motorSet(leftBack, 110);
			motorSet(rightBack, 110);
			motorSet(leftFront, -110);
			motorSet(rightFront, -110);
			motorRun = 1;
		}

		if(abs(encoderGet(bEnc)) >= dist && abs(encoderGet(fEnc))){
			motorSet(leftBack, 0);
			motorSet(rightBack, 0);
			motorSet(leftFront, 0);
			motorSet(rightFront, 0);
			motorRun = 0;
			lcdPrint(uart1, 1, "Yes");
		}
		if(digitalRead(launcherPosIn) != LOW || limit != 1){ //Limit Switch is not pressed or limit != 1
			if(digitalRead(launcherPosIn) == LOW){ //Limit Switch is pressed
				motorSet(rightLaunch1, 0);
				motorSet(rightLaunch2, 0);
				motorSet(rightLaunch3, 0);
				motorSet(leftLaunch1, 0);
				motorSet(leftLaunch2, 0);
				motorSet(leftLaunch3, 0);
			}
			limit = 0;
		}
		if(motorRun != 1 && limit != 1){
			break;
		}
	}
}
