/** @file opcontrol.c
 * @brief File for operator control code
 *
 * This file should contain the user operatorControl() function and any functions related to it.
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
#include "math.h"

/*
 * Runs the user operator control code. This function will be started in its own task with the
 * default priority and stack size whenever the robot is enabled via the Field Management System
 * or the VEX Competition Switch in the operator control mode. If the robot is disabled or
 * communications is lost, the operator control task will be stopped by the kernel. Re-enabling
 * the robot will restart the task, not resume it from where it left off.
 *
 * If no VEX Competition Switch or Field Management system is plugged in, the VEX Cortex will
 * run the operator control task. Be warned that this will also occur if the VEX Cortex is
 * tethered directly to a computer via the USB A to A cable without any VEX Joystick attached.
 *
 * Code running in this task can take almost any action, as the VEX Joystick is available and
 * the scheduler is operational. However, proper use of delay() or taskDelayUntil() is highly
 * recommended to give other tasks (including system tasks such as updating LCDs) time to run.
 *
 * This task should never exit; it should end with some kind of infinite loop, even if empty.
 */
Encoder rEnc;
Encoder lEnc;
Encoder liftEnc;
Encoder clawEnc;


void operatorControl() {
	lcdInit(uart1);
	lcdSetBacklight(uart1, true);

	if(!liftEnc) {
		liftEnc = encoderInit(6, 7, 1);
	}
	if(!lEnc){							//If autonomous did not initialize back encoder
		lEnc = encoderInit(2, 3, 1);	//Initialize back Encoder
	}
	if(!rEnc){							//If autonomous did not initialize front encoder
		rEnc = encoderInit(4, 5, 1);	//Initialize front Encoder
	}
	encoderReset(liftEnc);

	int deadzone = 20;					//Joystick deadzone
	int xAxis;
	int yAxis;
	int liftYAxis;
	int clawPos = 0;

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

	int shooting = 0;
	int liftMove = 0;
	int liftInitial = encoderGet(liftEnc);
	int liftValue = liftInitial;

	/*
	 * Joy Control Values are A, F, S defaulting to S
	 * A is for Arcade mode
	 * F is for a Forward Strafe Mode
	 * S is for a Sidways Strafe Mode
	 */
	while (1) {
		delay(20);

		//Gets different xAxis and yAxis values for different drive modes

		yAxis = -(joystickGetAnalog(1, 1));
		xAxis = -(joystickGetAnalog(1, 2));
		//////////////////////////////////////////////
		//											//
		//		   Drive Control Statements			//
		//											//
		//////////////////////////////////////////////

		//If controller not out of deadzone stop motors
		if(abs(xAxis) > deadzone || abs(yAxis) > deadzone) {
			motorSet(leftBack, -xAxis - yAxis);
			motorSet(rightBack, xAxis - yAxis);
			motorSet(leftFront, -xAxis - yAxis);
			motorSet(rightFront, xAxis - yAxis);
		} else {
			motorSet(leftBack, 0);
			motorSet(rightBack, 0);
			motorSet(leftFront, 0);
			motorSet(rightFront, 0);
		}

		///Catapult

		if(joystickGetDigital(1, 6, JOY_UP)){
			shooting = 1;
		}

		if(shooting == 1 && !joystickGetDigital(1, 6, JOY_DOWN)){
			motorSet(leftCatapultTop, 127);
			motorSet(leftCatapultBottom, 127);
			motorSet(rightCatapultTop, -127);
			motorSet(rightCatapultBottom, -127);
			if(digitalRead(shotLimit) == LOW){
				shooting = 0;
			}
		} else {
			shooting = 0;
			if(joystickGetDigital(1, 8, JOY_UP)){
				motorSet(leftCatapultTop, 127);
				motorSet(leftCatapultBottom, 127);
				motorSet(rightCatapultTop, -127);
				motorSet(rightCatapultBottom, -127);
			} else if(joystickGetDigital(1, 8, JOY_RIGHT)){
				motorSet(leftCatapultTop, -127);
				motorSet(leftCatapultBottom, -127);
				motorSet(rightCatapultTop, 127);
				motorSet(rightCatapultBottom, 127);
			} else {
				motorSet(leftCatapultTop, 0);
				motorSet(leftCatapultBottom, 0);
				motorSet(rightCatapultTop, 0);
				motorSet(rightCatapultBottom, 0);
			}
		}

		///////////////LIFT
		if(joystickGetDigital(1, 7, JOY_UP)){
			liftMove = 1;
			liftValue = 80;
		} else if(joystickGetDigital(1, 7, JOY_LEFT)){
			liftMove = 1;
			liftValue = 60; //Barely below highest lifting value;
		} else if(joystickGetDigital(1, 7, JOY_DOWN)){
			liftMove = 1;
			liftValue = liftInitial;
		}

		liftYAxis = joystickGetAnalog(1, 3);

		if(liftMove == 1 && !(abs(liftYAxis) > deadzone)){
			if(encoderGet(liftEnc) < liftValue+3){
				motorSet(leftLift, 127);
				motorSet(rightLift, -127);
			} else if(encoderGet(liftEnc) > liftValue-3){
				motorSet(leftLift, -127);
				motorSet(rightLift, 127);
			} else {
				motorSet(leftLift, 0);
				motorSet(rightLift, 0);
			}
		} else {
			liftMove = 0;
			if(abs(liftYAxis) > deadzone){
				motorSet(leftLift, liftYAxis);
				motorSet(rightLift, -liftYAxis);
			} else {
				motorSet(leftLift, 0);
				motorSet(rightLift, 0);
			}
		}

		lcdPrint(uart1, 2, "BLEH");
		lcdPrint(uart1, 1, "Lift: %d", encoderGet(liftEnc));

		if(joystickGetDigital(1, 5, JOY_UP) && clawPos == 0){
			clawPos = 1;
			digitalWrite(leftSolenoid, LOW);
			digitalWrite(rightSolenoid, LOW);
			delay(200);
		} else if(joystickGetDigital(1, 5, JOY_DOWN) ||(joystickGetDigital(1, 5, JOY_UP) && clawPos == 1)){
			clawPos = 0;
			digitalWrite(leftSolenoid, HIGH);
			digitalWrite(rightSolenoid, HIGH);
			delay(200);
		}

		//////////////////////////////////////
		//									//
		//			Extra Features			//
		//									//
		//////////////////////////////////////
		if(joystickGetDigital(1, 8, JOY_LEFT)){
			if(joystickGetDigital(1, 8, JOY_UP)){ //Press up and right on left buttons
				//Reset encoders
				encoderReset(rEnc);
				encoderReset(lEnc);
			}
			if(joystickGetDigital(1, 8, JOY_DOWN)){ //Press up and left on left buttons
				//Start autonomous
				autonomous();
			}
		}

	}
}
