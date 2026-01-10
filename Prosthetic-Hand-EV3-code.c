/***************************************************************************************
 * Project:      EV3 Prosthetic Hand (LEGO Mindstorms)
 * File:         Prosthetic-Hand-EV3-code.c
 * Team/Group:   Group 8-5
 * Course:       MTE 121 - Digital Computation
 * Version:      v8.0
 * Date:         December 5, 2023
 *
 * Authors:
 *   -Yaseen Mohamed
 *   -Sara Alrifai
 *   -Jack Doehler
 *   -Evan Janakievski
 *
 * Description:
 *   EV3 prosthetic hand control program written in RobotC. Implements:
 *   - Auto-grab mode using Ultrasonic + Touch sensors
 *   - Grip detection via motor encoder stall behavior
 *   - Movement-based release lockout using HiTechnic Accelerometer
 *   - Manual control mode + waving demo mode
 *
 * Hardware/Ports (edit to match your build):
 *   Motors:
 *     - motorA: Main finger 1
 *     - motorD: Main finger 2
 *     - motorC: Thumb
 *   Sensors:
 *     - S1: EV3 Touch sensor
 *     - S2: EV3 Ultrasonic sensor
 *     - S4: HiTechnic Accelerometer (I2C)
 *
 * Notes:
 *   - Encoder thresholds and distance thresholds are tuned for the current mechanism.
 *   - Emergency shutdown opens the hand at high speed.
 ***************************************************************************************/

#pragma config(Sensor, S1, HTAC, sensorI2CCustom)
#include "hitechnic-accelerometer.h"

const int ALL = -1;

void stopFinger(tMotor finger_motor);
bool checkForObj(bool&closed);
bool isMoving(tHTAC & accelerometer);
void gripObject();
void openFinger(tMotor finger_motor, int motor_speed);
void closeFinger(tMotor finger_motor, int motor_speed);
void initializeFinger(tMotor finger_motor);
void control (int&mode, bool&closed);
void wave (int&mode, bool&closed);

/************************************** MAIN **************************************/
/**
 * task main
 * Purpose:
 *   High-level state machine for the EV3 prosthetic hand demo.
 *   - Waits for user to start
 *   - Auto-detects object proximity and touch input
 *   - Grips object and prevents release while moving (accelerometer check)
 *   - Provides manual control and "wave" modes
 *
 * Key Inputs:
 *   S1: EV3 Touch sensor
 *   S2: EV3 Ultrasonic sensor (distance in cm)
 *   S4: HiTechnic accelerometer (I2C)
 *
 * Key Outputs:
 *   motorA, motorD: main finger motors
 *   motorC: thumb motor
 */
task main()
{
	SensorType[S1] = sensorEV3_Touch;
	SensorType[S2] = sensorEV3_Ultrasonic;

	tHTAC accelerometer;
  initSensor(&accelerometer, S4);

	displayCenteredBigTextLine(6, "PRESS ENTER");
	displayCenteredBigTextLine(8, "TO START");

	// Wait for a clean "Enter" press
	while (!getButtonPress(buttonEnter))
	{}
	while (getButtonPress(buttonEnter))
	{}

	eraseDisplay();
	displayCenteredBigTextLine(6, "BOOTING UP");

	// Zero motors so encoder positions are repeatable
	initializeFinger(motorA);
	initializeFinger(motorD);
	initializeFinger(motorC);

	int acc_x = 0, acc_y = 0, acc_z = 0;
	int mode = 1;
	bool closed = false;

	while (mode == 1)
	{
		eraseDisplay();
		displayCenteredBigTextLine(5, "Checking for");
		displayCenteredBigTextLine(7, "object to grab...");

		// Auto mode: check for object proximity + touch trigger
		while (!checkForObj(closed) && !getButtonPress(buttonAny))
		{}

		// Exit conditions
		if (getButtonPress(buttonUp) || getButtonPress(buttonDown))
				mode = 0;

		// Manual control mode
		if (getButtonPress(buttonLeft))
		{
			mode = 3;
			control(mode, closed);
		}

		// Wave mode
		if (getButtonPress(buttonRight))
		{
			mode = 4;
			wave(mode, closed);
		}

		// If we found an object and we're in auto mode, close to pick it up
		if (!closed && mode == 1)
		{
			closeFinger(ALL, 20);

			// Capture tilt/pose after grabbing (stored but not used elsewhere)
			readSensor(&accelerometer);
			acc_x = accelerometer.x;
			acc_y = accelerometer.y;
			acc_z = accelerometer.z;

			mode = 2;
		}

		while (mode == 2)
		{
			gripObject();

			if (getButtonPress(buttonUp))
				mode = 0;

			// Attempt to release: block if accelerometer indicates movement
			if (getButtonPress(buttonEnter))
			{
				while (getButtonPress(buttonEnter))
				{}

				if (!isMoving(accelerometer))
				{
					openFinger(ALL, 20);
					mode = 1;
				}
				else
				{
					eraseDisplay();
					displayCenteredBigTextLine(4, "Cannot release");
					displayCenteredBigTextLine(6, "while hand");
					displayCenteredBigTextLine(8, "is moving.");
					wait1Msec(750);
				}
			}
		}
	}

	eraseDisplay();

	// Regular shutdown
	if (getButtonPress(buttonDown))
	{
		while (getButtonPress(buttonDown))
		{}

		displayCenteredBigTextLine(5, "SHUTTING DOWN...");
		closeFinger(ALL, 10);
	}
	// Emergency shutdown
	else
	{
		displayCenteredBigTextLine(5, "EMERGENCY SHUTDOWN INITIALIZED");
		openFinger(ALL, 50);
	}

	wait1Msec(1000);
}


/***************************************************************************************
*************************************** FUNCTIONS **************************************
***************************************************************************************/

/**
 * initializeFinger
 * What:
 *   Moves the specified motor briefly to create a consistent starting pose,
 *   then stops and zeros its encoder.
 *
 * How:
 *   - Applies a short motor pulse (direction depends on motor)
 *   - Calls stopFinger() to halt motion
 *   - Sets nMotorEncoder[...] = 0
 *
 * Why:
 *   Ensures later open/close thresholds use repeatable encoder references.
 *
 * Params:
 *   finger_motor: EV3 motor port (motorA/motorC/motorD)
 */
void initializeFinger(tMotor finger_motor)
{
	if (finger_motor == motorA || finger_motor == motorD)
		motor[finger_motor] = -15;
	else
		motor[finger_motor] = 8;

	wait1Msec(175);

	stopFinger(finger_motor);
	nMotorEncoder[finger_motor] = 0;
}

/**
 * closeFinger
 * What:
 *   Closes one finger motor or all motors (fingers + thumb).
 *
 * How:
 *   - For motorA/motorD: positive closes
 *   - For motorC (thumb): negative closes (opposite direction)
 *   - For ALL: closes A and D, then closes C with a scaled speed
 *
 * Why:
 *   Performs a controlled closing action and then stops motors to avoid stalls.
 *
 * Params:
 *   finger_motor: motorA, motorC, motorD, or ALL (-1)
 *   motor_speed: base speed (thumb may be scaled when ALL)
 */
void closeFinger (tMotor finger_motor, int motor_speed)
{
	if (finger_motor == motorA || finger_motor == motorD)
		motor[finger_motor] = motor_speed;

	else if (finger_motor == motorC)
		motor[finger_motor] = -motor_speed;

	else if (finger_motor == ALL)
	{
		motor[motorA] = motor_speed;
		motor[motorD] = motor_speed;
		wait1Msec(250);
		motor[motorC] = -motor_speed*1.5;
	}

	wait1Msec(250);

	if (finger_motor != ALL)
		stopFinger(finger_motor);
	else
	{
		stopFinger(motorC);
		stopFinger(motorA);
		stopFinger(motorD);
	}
}

/**
 * openFinger
 * What:
 *   Opens one motor or all motors until a target encoder position is reached.
 *
 * How:
 *   - Drives motor(s) in the opening direction
 *   - Uses encoder thresholds to stop at a repeatable "open" position
 *   - Aborts if any button is pressed
 *
 * Why:
 *   Returns fingers/thumb to a known open state without overdriving the mechanism.
 *
 * Params:
 *   finger_motor: motorA, motorC, motorD, or ALL (-1)
 *   motor_speed: speed used for opening (thumb may be scaled when ALL)
 */
void openFinger (tMotor finger_motor, int motor_speed)
{
	// NOTE: "(finger_motor == (motorA || motorD))" is preserved as-is to avoid behavior changes.
	if (finger_motor == (motorA || motorD))
	{
		motor[finger_motor] = -motor_speed;
		while (nMotorEncoder[finger_motor] > 10 && !getButtonPress(buttonAny))
		{}
		motor[finger_motor] = 0;
	}
	else if (finger_motor == motorC)
	{
		motor[finger_motor] = motor_speed;
		while (nMotorEncoder[finger_motor] > 40 && !getButtonPress(buttonAny))
		{}
		motor[finger_motor] = 0;
	}
	else
	{
		motor[motorA] = -motor_speed;
		motor[motorD] = -motor_speed;
		wait1Msec(350);
		motor[motorC] = motor_speed*2;

		while(nMotorEncoder[motorA] > 10 && !getButtonPress(buttonAny))
		{}
		motor[motorA] = 0;

		while(nMotorEncoder[motorD] > 10 && !getButtonPress(buttonAny))
		{}
		motor[motorD] = 0;

		while(nMotorEncoder[motorC] > 40 && !getButtonPress(buttonAny))
		{}
		motor[motorC] = 0;
	}
}

/**
 * stopFinger
 * What:
 *   Stops a motor once it is no longer changing encoder position (i.e., has settled).
 *
 * How:
 *   - Samples encoder position, waits a short time window, samples again
 *   - Repeats until no change is detected (or a button is pressed)
 *   - Sets motor power to 0
 *
 * Why:
 *   Prevents overshoot and ensures the motor actually finishes moving before actions continue.
 *
 * Params:
 *   finger_motor: motor port to stop
 */
void stopFinger(tMotor finger_motor)
{
	int init_pos = 0;
	int new_pos = 0;
	int change_in_pos = 1000;

	while (change_in_pos > 0 && !getButtonPress(buttonAny))
	{
		clearTimer(T1);
		init_pos = nMotorEncoder[finger_motor];

		if (!getButtonPress(buttonAny))
		{
			while (time1[T1] <= 350)
			{}
			new_pos = nMotorEncoder[finger_motor];
			change_in_pos = abs(new_pos - init_pos);
		}
	}

	motor[finger_motor] = 0;
}

/**
 * gripObject
 * What:
 *   Actively closes the hand until it detects a "grip" condition.
 *
 * How:
 *   - Drives A/D (fingers) and C (thumb) at higher speeds
 *   - Monitors encoder change; if the finger encoder stops changing, assumes contact/stall = gripped
 *
 * Why:
 *   Produces a simple “close until contact” behavior without force sensors.
 *
 * Notes:
 *   This approach assumes "no encoder movement" means an object is resisting motion.
 */
void gripObject()
{
	while (!getButtonPress(buttonUp) && !getButtonPress(buttonEnter))
	{
		int init_pos = 0;
		bool gripped = false;

		init_pos = nMotorEncoder[motorA];
		clearTimer(T1);

		motor[motorA] = motor[motorD] = 50;
		motor[motorC] = -30;

		while (!gripped)
		{
			init_pos = nMotorEncoder[motorA];
			clearTimer(T1);

			while (time1[T1] <= 100)
			{}

			if (init_pos - nMotorEncoder[motorA] == 0)
			{
				gripped = true;
				eraseDisplay();
				displayBigTextLine(6, "GRIPPED");
			}
		}

		motor[motorA] = motor[motorD] = motor[motorC] = 0;
	}
}

/**
 * checkForObj
 * What:
 *   Uses ultrasonic proximity + touch input to decide when the system should grab.
 *
 * How:
 *   - If an object is within 20 cm, open the hand and wait up to ~3 seconds for a touch trigger
 *   - If object remains within 20 cm, continue waiting for touch or button exit
 *   - If object leaves range and hand is open, close the hand back up (idle posture)
 *
 * Why:
 *   Enables an “auto grab” flow: open when object is near, grab when touch is pressed.
 *
 * Params:
 *   closed (in/out): tracks whether the hand is currently closed
 *
 * Returns:
 *   true  -> trigger condition met (ready to grab or user confirms)
 *   false -> no trigger / user exit
 */
bool checkForObj(bool&closed)
{
	if (SensorValue[S2] <= 20)
	{
		openFinger(ALL, 20);
		closed = false;

		clearTimer(T1);

		while (time1[T1] < 3000 && !getButtonPress(buttonAny))
		{
			if ((SensorValue[S1] && !closed) || getButtonPress(buttonAny))
				return true;
		}
	}

	while (SensorValue[S2] <= 20)
	{
		if (SensorValue[S1] && closed == false)
			return true;

		else if (getButtonPress(buttonAny))
		{
			if (getButtonPress(buttonEnter))
				return true;
			else
				return false;
		}
	}

	if (!closed)
	{
		closeFinger(ALL, 20);
		closed = true;
	}

	return false;
}

/**
 * isMoving
 * What:
 *   Detects whether the hand is moving by comparing accelerometer samples.
 *
 * How:
 *   - Reads accelerometer (x/y/z)
 *   - Waits 250 ms and reads again
 *   - If any axis changes beyond a threshold, considers the hand "moving"
 *
 * Why:
 *   Prevents releasing the grip when the hand is in motion.
 *
 * Params:
 *   accelerometer: reference to the initialized accelerometer sensor object
 *
 * Returns:
 *   true  -> movement detected
 *   false -> hand appears stationary
 */
bool isMoving(tHTAC & accelerometer)
{
	readSensor(&accelerometer);
  	clearTimer(T1);

	int x_before = accelerometer.x;
	int y_before = accelerometer.y;
	int z_before = accelerometer.z;

	wait1Msec(250);

	readSensor(&accelerometer);
	int x_current = accelerometer.x;
	int y_current = accelerometer.y;
	int z_current = accelerometer.z;

	int x_difference = x_before - x_current;
	int y_difference = y_before - y_current;
	int z_difference = z_before - z_current;

	if (abs(x_difference) > 30 || abs(y_difference) > 30 || abs(z_difference) > 30)
		return true;
	else
		return false;
}

/**
 * control
 * What:
 *   Manual control mode for opening/closing the hand while buttons are held.
 *
 * How:
 *   - Right button: close (A/D positive, C negative)
 *   - Left button: open (A/D negative, C positive)
 *   - Enter: return to auto mode and set a known closed posture
 *   - Up/Down: exit program
 *
 * Why:
 *   Provides direct manual override for testing and demos.
 *
 * Params:
 *   mode (in/out): state machine mode; exits when changed
 *   closed (in/out): updated to keep consistency with auto logic
 */
void control (int&mode, bool&closed)
{
	eraseDisplay();
	displayCenteredBigTextLine(6, "Manual Control Mode");

	while (mode == 3)
	{
		if (getButtonPress(buttonRight))
		{
			motor[motorA] = motor[motorD]=20;
			motor[motorC] = -10;
			while (getButtonPress(buttonRight) && !getButtonPress(buttonEnter) && !getButtonPress(buttonDown) && !getButtonPress(buttonUp))
			{}
			motor[motorA] = motor[motorD] = motor[motorC] = 0;
		}
		else if (getButtonPress(buttonLeft))
		{
			motor[motorA] = motor[motorD] = -20;
			motor[motorC] = 10;
			while (getButtonPress(buttonLeft) && !getButtonPress(buttonEnter) && !getButtonPress(buttonDown) && !getButtonPress(buttonUp))
			{}
			motor[motorA] = motor[motorD] = motor[motorC]=0;
		}
		else if (getButtonPress(buttonEnter))
		{
			mode = 1;
			openFinger(ALL, 20);
			closeFinger(ALL, 20);
			closed = true;
		}
		else if (getButtonPress(buttonUp) || getButtonPress(buttonDown))
			mode = 0;
	}
	eraseDisplay();
}

/**
 * wave
 * What:
 *   Runs a simple “wave” animation using finger motors A and D.
 *
 * How:
 *   - Opens hand slightly
 *   - Repeats a back-and-forth motion using encoder thresholds
 *   - Returns to auto mode with a closed posture
 *
 * Why:
 *   Provides a demo-friendly gesture mode.
 *
 * Params:
 *   mode (in/out): wave mode runs while mode==4; updates to exit/return to auto
 *   closed (in/out): updated when returning to auto mode
 */
void wave (int&mode, bool&closed)
{
	eraseDisplay();
	displayCenteredBigTextLine(6, "WAVING");

	if (mode != 0 && mode == 4)
	{
		openFinger(ALL, 15);
		wait1Msec(100);

		for (int count=0; count<5 && !getButtonPress(buttonEnter) && !getButtonPress(buttonDown) && !getButtonPress(buttonUp); count++)
		{
			motor[motorA] = motor[motorD] = 25;
			while (nMotorEncoder(motorA) < 175 && nMotorEncoder(motorD) < 175 && !getButtonPress(buttonEnter) && !getButtonPress(buttonDown) && !getButtonPress(buttonUp))
			{}
			motor[motorA] = motor[motorD] = 0;
			wait1Msec(100);

			motor[motorA] = motor[motorD] = -25;
			while (nMotorEncoder(motorA) > 100 && nMotorEncoder(motorD) > 100 && !getButtonPress(buttonEnter) && !getButtonPress(buttonDown) && !getButtonPress(buttonUp))
			{}
			motor[motorA] = motor[motorD] = 0;
			wait1Msec(100);
		}

		if (getButtonPress(buttonUp) || getButtonPress(buttonDown))
			mode = 0;
		else
		{
			openFinger(ALL, 20);
			closeFinger(ALL, 20);
			closed = true;
			mode = 1;
		}
	}
	eraseDisplay();
}
