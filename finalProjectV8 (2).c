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

task main()
{
	SensorType[S1] = sensorEV3_Touch;
	SensorType[S2] = sensorEV3_Ultrasonic;
	tHTAC accelerometer;
  initSensor(&accelerometer, S4);

	displayCenteredBigTextLine(6, "PRESS ENTER");
	displayCenteredBigTextLine(8, "TO START");

	while (!getButtonPress(buttonEnter))
	{}
	while (getButtonPress(buttonEnter))
	{}

	eraseDisplay();
	displayCenteredBigTextLine(6, "BOOTING UP");

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

		while (!checkForObj(closed) && !getButtonPress(buttonAny))
		{}

		if (getButtonPress(buttonUp) || getButtonPress(buttonDown))
				mode = 0;

		//starts manual control function
		if (getButtonPress(buttonLeft))
		{
			mode = 3;
			control(mode, closed);
		}

		if (getButtonPress(buttonRight))
		{
			mode = 4;
			wave(mode, closed);
		}

		if (!closed && mode == 1)
		{
			closeFinger(ALL, 20);

			//check the tilt of hand when object is picked up
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

	// regular shutdown
	if (getButtonPress(buttonDown))
	{
		while (getButtonPress(buttonDown))
		{}

		displayCenteredBigTextLine(5, "SHUTTING DOWN...");
		closeFinger(ALL, 10);
	}

	// emergency shutdown
	else {
		displayCenteredBigTextLine(5, "EMERGENCY SHUTDOWN INITIALIZED");
		openFinger(ALL, 50);
	}

	wait1Msec(1000);
}


/***************************************************************************************
*************************************** FUNCTIONS **************************************
***************************************************************************************/


/************************************** initializeFinger **************************************/

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

/************************************** closeFinger **************************************/

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

/************************************** openFinger **************************************/

void openFinger (tMotor finger_motor, int motor_speed)
{
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

/************************************** stopFinger **************************************/

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

/************************************** gripObject **************************************/

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

/************************************** checkForObj **************************************/

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

/************************************** isMoving **************************************/

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

/************************************** control **************************************/

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

/************************************** wave **************************************/


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
