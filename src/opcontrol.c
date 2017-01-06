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

void operatorControl() {

	lcdInit(uart1);
	lcdSetBacklight(uart1, true);

	if(!liftEnc) {
		liftEnc = encoderInit(5, 6, 0);
	}
	if(!lEnc){							//If autonomous did not initialize back encoder
		lEnc = encoderInit(1, 2, 1);	//Initialize back Encoder
	}
	if(!rEnc){							//If autonomous did not initialize front encoder
		rEnc = encoderInit(3, 4, 1);	//Initialize front Encoder
	}
	encoderReset(liftEnc);

	//Drive Variables
	int deadzone = 20;					//Joystick deadzone
	int xAxis;
	int yAxis;

	//Lift Variables
	int liftYAxis;
	int liftPos = encoderGet(liftEnc);
	int liftSpeed = 33;
	int liftCounter = 0;


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

	while (1) {
		delay(20);

		//Gets different xAxis and yAxis values for different drive modes

		yAxis = -(joystickGetAnalog(1, 2));
		xAxis = -(joystickGetAnalog(1, 1));
		/////////p/////////////////////////////////////
		//											//
		//		   Drive Control Statements			//
		//											//
		//////////////////////////////////////////////

		//If controller not out of deadzone stop motors
		if(abs(xAxis) > deadzone || abs(yAxis) > deadzone) {
			motorSet(rightFrontDrive, -xAxis + yAxis);
			motorSet(rightBackDrive, -xAxis + yAxis);
			motorSet(leftFrontDrive, -xAxis - yAxis);
			motorSet(leftBackDrive, -xAxis - yAxis);
		} else {
			motorSet(rightFrontDrive, 0);
			motorSet(rightBackDrive, 0);
			motorSet(leftFrontDrive, 0);
			motorSet(leftBackDrive, 0);
		}

		liftYAxis = joystickGetAnalog(1, 3);

		if(abs(liftYAxis) > deadzone) {
			motorSet(leftLiftInner, liftYAxis);
			motorSet(leftLiftOuter, -liftYAxis);
			motorSet(rightLiftInner, -liftYAxis);
			motorSet(rightLiftOuter, liftYAxis);
			liftPos = encoderGet(liftEnc);
		} else {
			/*lcdPrint(uart1, 2, "Count: %d", encoderGet(liftEnc));
			if (encoderGet(liftEnc) + 10 > liftPos) {
				if(liftCounter % 20 == 0) {
					liftSpeed -= 2;
				}
				liftCounter++;
			} else if(encoderGet(liftEnc) - 10 < liftPos) {
				if(liftCounter % 20 == 0){
					liftSpeed += 2;
				}
				liftCounter++;
			} else {
				if(liftCounter != 0){
					if(liftSpeed < 33) {
						liftSpeed += 25;
					} else if (liftSpeed > 33) {
						liftSpeed -= 15;
					}
				}
				liftCounter = 0;
			}*/
			motorSet(leftLiftInner, 0);
			motorSet(leftLiftOuter, 0);
			motorSet(rightLiftInner, 0);
			motorSet(rightLiftOuter, 0);
		}

		if(joystickGetDigital(1, 6, JOY_UP)){
			motorSet(leftClaw, 127);
			motorSet(rightClaw, -127);
		} else if(joystickGetDigital(1, 6, JOY_DOWN)){
			motorSet(leftClaw, -127);
			motorSet(rightClaw, 127);
		} else {
			motorSet(leftClaw, 0);
			motorSet(rightClaw, 0);
		}

		lcdPrint(uart1, 1, "Lift: %d", encoderGet(rEnc));



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
				lcdPrint(uart1, 1, "HIA");
				autonomous();
			}
		}

	}
}
