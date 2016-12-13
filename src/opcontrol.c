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
Encoder fEnc;
Encoder bEnc;
Encoder liftEnc;
Encoder clawEnc;


void operatorControl() {
	lcdInit(uart1);
	lcdSetBacklight(uart1, true);

	if(!liftEnc) {
		liftEnc = encoderInit(6, 7, 0);
	}
	if(!bEnc){							//If autonomous did not initialize back encoder
		bEnc = encoderInit(2, 3, 1);	//Initialize back Encoder
	}
	if(!fEnc){							//If autonomous did not initialize front encoder
		fEnc = encoderInit(4, 5, 1);	//Initialize front Encoder
	}
	if(!clawEnc){
		clawEnc = encoderInit(8, 9, 0);
	}

	encoderReset(liftEnc);

	int deadzone = 20;					//Joystick deadzone
	int xAxis;
	int yAxis;
	int death = 0;
	int liftspeed = 0;
	//Motor Port Constants
	const int leftBack = 1;
	const int rightBack = 10;
	const int leftLiftBot = 3;
	const int leftLiftTop = 4;
	const int claw = 5;
	const int farClaw = 6;
	const int rightLiftTop = 7;
	const int rightLiftBot = 8;
	const int rightFront = 9;
	const int leftFront = 2;

	float liftHeight = encoderGet(liftEnc);
	float lift;
	/*
	 * Joy Control Values are A, F, S defaulting to S
	 * A is for Arcade mode
	 * F is for a Forward Strafe Mode
	 * S is for a Sidways Strafe Mode
	 */
	int joyControl = 1;
	//Driver Control Loop
	while (1) {
		delay(20);

		//Gets different xAxis and yAxis values for different drive modes

		if(joyControl == 1){		//Arcade Mode
			yAxis = -(joystickGetAnalog(1, 1));
			xAxis = -(joystickGetAnalog(1, 2));
		}
		//////////////////////////////////////////////
		//																					//
		//		   Drive Control Statements						//
		//																					//
		//////////////////////////////////////////////
		if(joyControl == 3 || joyControl == 2 || joyControl == 1 || joyControl != 0){
			//If joystick is pressed out of deadzone
			if(abs(xAxis) >= deadzone || abs(yAxis) >= deadzone){
				//Runs Right in SS mode and Up in UD mode
				if(xAxis > deadzone && !(joyControl == 1)){
					motorSet(leftBack, -xAxis - yAxis);
					motorSet(rightBack, -xAxis - yAxis);
					motorSet(leftFront, xAxis - yAxis);
					motorSet(rightFront, xAxis - yAxis);
				//Runs Left in SS mode and Down in UD mode
				} else if(xAxis < -deadzone && !(joyControl == 1)){
					motorSet(leftBack, -xAxis + yAxis);
					motorSet(rightBack, -xAxis + yAxis);
					motorSet(leftFront, xAxis + yAxis);
					motorSet(rightFront, xAxis + yAxis);
				//Runs arcade mode
				} else if(((abs(xAxis) > deadzone || abs(yAxis) > deadzone) && joyControl == 1)) {
					motorSet(leftBack, -xAxis - yAxis);
					motorSet(rightBack, xAxis - yAxis);
					motorSet(leftFront, -xAxis - yAxis);
					motorSet(rightFront, xAxis - yAxis);
				//Runs Up/Down in SS and Left/Right in UD
				} else {
					if(!(joyControl == 1)){
						motorSet(leftBack, yAxis);
						motorSet(rightBack, yAxis);
						motorSet(leftFront, yAxis);
						motorSet(rightFront, yAxis);
					}
				}
			//If controller not out of deadzone stop motors
			} else {
				motorSet(leftBack, 0);
				motorSet(rightBack, 0);
				motorSet(leftFront, 0);
				motorSet(rightFront, 0);
			}
		//Extra slot for another drive mode
		} else if (joyControl == 0) {

		}


		//////////////////////////////////////
		//									//
		//	 	 Lift Statements 			//
		//									//
		//////////////////////////////////////

		lift = joystickGetAnalog(1, 3);
		if(abs(lift) > deadzone) {
			motorSet(leftLiftBot, lift);
			motorSet(leftLiftTop, lift);
			motorSet(rightLiftBot, -(lift));
			motorSet(rightLiftTop, -(lift));
			liftHeight = encoderGet(liftEnc) -2;
		} else if(encoderGet(liftEnc) < liftHeight) {
			motorSet(leftLiftBot, liftspeed);
			motorSet(leftLiftTop, liftspeed);
			motorSet(rightLiftBot, -liftspeed);
			motorSet(rightLiftTop, -liftspeed);
			death++;
			if((death % 50 == 0) && liftspeed < 110){
				liftspeed += 20;
			}
		} else {
			death = 0;
			liftspeed = 30;
			motorSet(leftLiftBot, 0);
			motorSet(leftLiftTop, 0);
			motorSet(rightLiftBot, 0);
			motorSet(rightLiftTop, 0);
		}


		if(joystickGetDigital(1, 6, JOY_UP)){ //Open
			motorSet(claw, 127);
		} else if(joystickGetDigital(1, 6, JOY_DOWN)){
			motorSet(claw, -127);
		} else {
			motorSet(claw, 0);
		}

		if(joystickGetDigital(1, 5, JOY_UP)){
			motorSet(farClaw, -127);
		} else if(joystickGetDigital(1, 5, JOY_DOWN)) {
			motorSet(farClaw, 127);
		} else {
			motorSet(farClaw, 0);
		}

		lcdPrint(uart1, 1, "liftspeed: %d", liftspeed);
		lcdPrint(uart1, 2, "Claw %d", encoderGet(clawEnc));

		//lift is joystick value
		//liftHeight is where the joystick sets the lift to be
		//liftspeed is how fast it is lifting
		//death is a counter for the if statement
		/*lift = joystickGetAnalog(1, 3);



		if(abs(lift) > deadzone) {
			liftHeight += floor((lift/10));
		}
1
		if (encoderGet(liftEnc) < liftHeight) {
			if(death == 0){
				death = 1;
			}
			motorSet(leftLiftBot, liftspeed);
			motorSet(leftLiftTop, liftspeed);
			motorSet(rightLiftBot, -liftspeed);
			motorSet(rightLiftTop, -liftspeed);
		} else if(encoderGet(liftEnc) > liftHeight) {
			if(death == 0){
				death = 1;
			}
			motorSet(leftLiftBot, -liftspeed);
			motorSet(leftLiftTop, -liftspeed);
			motorSet(rightLiftBot, liftspeed);
			motorSet(rightLiftTop, liftspeed);
		} else {
			death = 0;
			liftspeed = 0;
			motorSet(leftLiftBot, 0);
			motorSet(leftLiftTop, 0);
			motorSet(rightLiftBot, 0);
			motorSet(rightLiftTop, 0);
		}

		if(death != 0){
			if((death % 100 == 0) && liftspeed < 130){
				liftspeed += 10;
			}
			death++;
		}



		lcdPrint(uart1, 1, "liftspeed: %d", liftHeight);
		lcdPrint(uart1, 2, "encoderget %d", encoderGet(liftEnc));

		lift = joystickGetAnalog(1, 3);
		if(abs(lift) > deadzone){
			motorSet(leftLiftBot, lift/2);
			motorSet(leftLiftTop, lift/2);
			motorSet(rightLiftBot, -(lift/2));
			motorSet(rightLiftTop, -(lift/ 2));
		} else {
			motorSet(leftLiftBot, 0);
			motorSet(leftLiftTop, 0);
			motorSet(rightLiftBot, 0);
			motorSet(rightLiftTop, 0);
		}
		*/
		//////////////////////////////////////
		//									//
		//			Extra Features			//
		//									//
		//////////////////////////////////////
		if(joystickGetDigital(1, 7, JOY_UP)){
			if(joystickGetDigital(1, 7, JOY_RIGHT)){ //Press up and right on left buttons
				//Reset encoders
				encoderReset(fEnc);
				encoderReset(bEnc);
			}
			if(joystickGetDigital(1, 7, JOY_LEFT)){ //Press up and left on left buttons
				//Start autonomous
				autonomous();
			}
		}


	}
}
