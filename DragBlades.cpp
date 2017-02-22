#include "RCRClasses.h"




void DragBladesClass::init() {
	//setup motor pins
	pinMode(MOTOR_A, OUTPUT);
	pinMode(MOTOR_B, OUTPUT);
	pinMode(MOTOR_PWM, OUTPUT);
	//setup encoder pins
	pinMode(ENC_A, INPUT);
	pinMode(ENC_B, INPUT);
	//setup limit switch pins
	pinMode(LIM_OUT, INPUT);
	pinMode(LIM_IN, INPUT);
}

/**************************************************************************/
/*!
@brief  Prints status info on the limit switches, encoder position, etc
Author: Ben
*/
/**************************************************************************/
void DragBladesClass::dragBladesCheck() {
	Serial.println("\r\n-----Drag Blades Check----");	
	Serial.printf("encMin: %d\r\nencMax: %d\r\nencPos: %d\r\nInner limit pressed: %d\r\nOutter limit pressed: %d\r\n\r\n", encMin, encMax, encPos, !digitalRead(LIM_IN), !digitalRead(LIM_OUT) );
}

/**************************************************************************/
/*!
@brief  Returns the encoder position that the airbrdakes should deploy based on how
fast the vehicle is moving and how fast the SPP thinks it should be moving
Author: Ben
*/
/**************************************************************************/
int DragBladesClass::airBrakesGoToEncPos(float vehVel, float sppVel)
{
	float returnVal;
	returnVal = -(sppVel - vehVel) * AIRBRAKES_GAIN;
	if (returnVal >= 100) returnVal = 100;
	else if (returnVal <= 0) returnVal = 0;
	return (int)map((long)returnVal, 0, 100, encMin, encMax);
}

/**************************************************************************/
/*!
@brief  Makes the DC motor spin with a given speed and direction
Also ensures that the drag blades don't extend past their limits.
Author: Ben
*/
/**************************************************************************/
void DragBladesClass::motorDo(bool direction, uint8_t speed) {
	bool limit_in, limit_out;
	int range;
	if (direction) {
		digitalWrite(MOTOR_A, HIGH);
		digitalWrite(MOTOR_B, LOW);
	}
	else {
		digitalWrite(MOTOR_A, LOW);
		digitalWrite(MOTOR_B, HIGH);
	}
	limit_in = digitalRead(LIM_IN);
	limit_out = digitalRead(LIM_OUT);
	DataLog.supStat.encPos = encPos;
	DataLog.supStat.encPosCmd = encPosCmd;
	DataLog.supStat.limit_in = limit_in;
	DataLog.supStat.limit_out = limit_out;
	DataLog.supStat.encMax = encMax;
	DataLog.supStat.encMin = encMin;
	DataLog.supStat.mtrSpdCmd = mtrSpdCmd;
	if (!limit_in && (direction == INWARD)) {
		analogWrite(MOTOR_PWM, 0);
		range = myAbs(encMax - encMin);
		encPos = 0;	
		encMin = 0;
		encMax = range;
	}
	else if (!limit_out && (direction == OUTWARD)) {
		analogWrite(MOTOR_PWM, 0);
		range = myAbs(encMax - encMin);
		encPos = range;
		encMin = 0;
		encMax = range;
	}
	else {
		analogWrite(MOTOR_PWM, speed);
	}
	if ((encMax - encMin) < 9 * ENC_RANGE / 10) {
		encMin = 0;
		encMax = ENC_RANGE;
		DataLog.logError(ENC_RANGE_ERROR);
	}
}

void DragBladesClass::motorDont() {
	digitalWrite(MOTOR_A, LOW);
	digitalWrite(MOTOR_B, LOW);
	analogWrite(MOTOR_PWM, 0);
}

/**************************************************************************/
/*!
@brief  This function returns whether or not the dragblades are at the given
encoder position. If false, motorGoTo() calculates the speed and direction required
for the dragblades to reach their target. To return true, the blades must be at 
their target for at least SETPOINT_INAROW succesive calls of motorGoTo()
Author: Ben
*/
/**************************************************************************/
bool DragBladesClass::motorGoTo(int16_t goTo)
{
	static uint8_t count = 0;
	encPosCmd = goTo;
	motorPID.Compute();
	if (mtrSpdCmd >= 0) {
		DragBlades.motorDo(OUTWARD, mtrSpdCmd);
	}
	else if (mtrSpdCmd < 0) {
		DragBlades.motorDo(INWARD, -1 * mtrSpdCmd);
	}
	if ((myAbs(DragBlades.encPos - encPosCmd) <= SETPOINT_TOLERANCE)) {
		count++;
	}
	else {
		count = 0;
	}
#if DEBUG_MOTORGOTO
	Serial.println("");
	Serial.println("MOTORGOTO----------------");
	Serial.printf("goTo: %d\r\n", goTo);
	Serial.printf("count: %d\r\n", count);
	dragBladesCheck();
#endif
	if (count >= SETPOINT_INAROW) {
		count = 0;		
		return true;
	}
	else {
		return false;
	}
}

/**************************************************************************/
/*!
@brief  First calibrates the drag blades by finding the limit switches. 
When a limit switch is hit, remember what encoder position it was hit at. 
Then exercise motorGoTo by extending and retracting the blades in 1/4 turns
Author: Ben
*/
/**************************************************************************/
void DragBladesClass::motorTest()
{
	int timer = 0;
	DataLog.sd.remove(MOTOR_FILENAME);                                 //Removes prior error file

	File data = DataLog.sd.open(MOTOR_FILENAME, FILE_WRITE);       //Creates new data file
	if (!data) {                                                    //If unable to be initiated, throw error statement.  Do nothing
		Serial.println("Data file unable to initiated - motorTest");
		SD_GO = false;
		return;
	}
	else {                                             //Adds unique header depending on if VDS is in test or flight mode
		data.println("times, encPos, encPosCmd, limit_out, limit_in, encMax, encMin, mtrSpdCmd");
		data.close();                                               //Closes data file after use.
	}
	timer = millis();
	while (digitalRead(LIM_OUT)) {
		motorDo(OUTWARD, DEADZONE_MAX);
		if ((Serial.available() > 0) || ((millis() - timer) >4000)) {
			motorDont();
			return;
		}
	}
	timer = millis();
	while (digitalRead(LIM_IN)) {
		motorDo(INWARD, DEADZONE_MAX);
		if ((Serial.available() > 0) || ((millis() - timer) >4000)) {
			motorDont();
			return;
		}
	}	
	DragBlades_GO = true;

	motorGoToPersistent(0);
	motorDont();
	delay(300);
	motorGoToPersistent(0);
	motorGoToPersistent(25);
	motorDont();
	delay(300);
	motorGoToPersistent(25);
	motorGoToPersistent(50);
	motorDont();
	delay(300);
	motorGoToPersistent(50);
	motorGoToPersistent(75);
	motorDont();
	delay(300);
	motorGoToPersistent(75);
	motorGoToPersistent(100);
	motorDont();
	delay(300);
	motorGoToPersistent(100);
	motorGoToPersistent(75);
	motorDont();
	delay(300);
	motorGoToPersistent(75);
	motorGoToPersistent(50);
	motorDont();
	delay(300);
	motorGoToPersistent(50);
	motorGoToPersistent(25);
	motorDont();
	delay(300);
	motorGoToPersistent(25);
	motorGoToPersistent(0);
	motorDont();
	motorGoToPersistent(0);
	motorDont();
}

/**************************************************************************/
/*!
@brief  This is a quick exercise meant to spin the motor in different ways.
Data collected from this function was used to build a transfer function for the DC motor
WARNING: Do not execute this function if the motor is installed in the VDS as it will try
to spin past its physical limits and the motor will stall.
Author: Ben
*/
/**************************************************************************/
void DragBladesClass::motorExercise()
{
	int deadZoneSpeed = 63;
	unsigned long t = 0;
	unsigned long t0 = 0;
	bool dir = OUTWARD;
	float derp;
	uint8_t spd = 0;
	DataLog.sd.remove("motorExercise.dat");
	File myFile = DataLog.sd.open("motorExercise.dat", FILE_WRITE);
	myFile.println("times,spd,dir");
	myFile.close();

	t0 = micros();
	while (t<8000000) {
		myFile = DataLog.sd.open("motorExercise.dat", FILE_WRITE);
		t = micros() - t0;
		if (t < 1000000) {
			dir = OUTWARD;
			spd = 255;
		}
		else if (t < 2000000) {
			dir = OUTWARD;
			derp = (float)(t - 1000000) / 1000000;
			spd = derp * 255;
			//Serial.print("derp = ");
			//Serial.println(derp);
		}
		else if (t < 3000000) {
			dir = OUTWARD;
			spd = 0;
		}
		else if (t < 4000000) {
			dir = OUTWARD;
			spd = deadZoneSpeed;
		}
		else if (t < 5000000) {
			dir = OUTWARD;
			derp = (float)(t - 4000000) / 1000000;
			spd = deadZoneSpeed + derp * (255 - deadZoneSpeed);
			//Serial.print("derp = ");
			//Serial.println(derp);
		}
		else if (t < 6000000) {
			dir = INWARD;
			spd = 255;
		}
		else if (t < 7000000) {
			dir = OUTWARD;
			spd = 0;
		}
		//Serial.print(t);
		//Serial.print(",\t");
		//Serial.println(spd);
		DragBlades.motorDo(dir, spd);
		myFile.printf("%lu,%u,%d,%d", t, spd, dir, DragBlades.encPos);
		myFile.println("");
		myFile.close();
	}
}

/**************************************************************************/
/*!
@brief  Moves the blades in and out until a serial command is recieved
Author: Ben
*/
/**************************************************************************/
void DragBladesClass::powerTest() {
	while (!(Serial.available() > 0)) {
		while (!motorGoTo(encMin)) {
			delay(MOTORTEST_DELAY_MS);
		}
		while (!motorGoTo(encMax)) {
			delay(MOTORTEST_DELAY_MS);
		}
	}
	GUI.eatYourBreakfast();
	while (!(Serial.available() > 0)) {
		while ( !motorGoTo(encMin)) {
			delay(MOTORTEST_DELAY_MS);
		}
		motorDont();
		delay(500);
		while (!motorGoTo(map(33, 0, 100, encMin, encMax))) {
			delay(MOTORTEST_DELAY_MS);
		}
		motorDont();
		delay(500);
		while (!motorGoTo(map(66, 0, 100, encMin, encMax))) {
			delay(MOTORTEST_DELAY_MS);
		}
		motorDont();
		delay(500);
		while ( !motorGoTo(encMax)) {
			delay(MOTORTEST_DELAY_MS);
		}
		motorDont();
		delay(500);
		while (!motorGoTo(map(66, 0, 100, encMin, encMax))) {
			delay(MOTORTEST_DELAY_MS);
		}
		motorDont();
		delay(500);
		while (!motorGoTo(map(33, 0, 100, encMin, encMax))) {
			delay(MOTORTEST_DELAY_MS);
		}
		motorDont();
		delay(500);
	}
}

/**************************************************************************/
/*!
@brief  For whatever reason the abs function wasn't working for me so I made one
Author: Ben
*/
/**************************************************************************/
int DragBladesClass::myAbs(int x) {
	if (x >= 0) {
		return x;
	}
	else {
		return -x;
	}
}

/**************************************************************************/
/*!
@brief  A function that will not return until the motor reaches its destination
Also takes an argument as a percent and not as an encoder position.
Also records data on the sd card.
Author: Ben
*/
/**************************************************************************/
void DragBladesClass::motorGoToPersistent(uint16_t goToPercent)
{
	File data = DataLog.sd.open(MOTOR_FILENAME, FILE_WRITE);       //Creates new data file
	while (!motorGoTo(map(goToPercent, 0, 100, encMin, encMax))) {
		data.open(MOTOR_FILENAME, FILE_WRITE);
		if (data) {
			data.printf("%lu,%d,%d,%d,%d,%d,%d,%d", millis(), encPos, encPosCmd, DataLog.supStat.limit_out, DataLog.supStat.limit_in, DataLog.supStat.encMax, DataLog.supStat.encMin, DataLog.supStat.mtrSpdCmd);
			data.println("");
			data.close();
		}
		delay(MOTORTEST_DELAY_MS);
	}
}


DragBladesClass DragBlades;