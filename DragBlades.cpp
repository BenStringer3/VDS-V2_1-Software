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

void DragBladesClass::motorDo(bool direction, uint8_t speed) {
	bool limit_in, limit_out;
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
	DataLog.supStat.limit_in = limit_in;
	DataLog.supStat.limit_out = limit_out;
	if (!LIMITSWITCHES_DETATCHED && !limit_in && (direction == INWARD)) {
		Serial.println("in");
		analogWrite(MOTOR_PWM, 0);
		encMin = encPos;
	}
	else if (!LIMITSWITCHES_DETATCHED && !limit_out && (direction == OUTWARD)) {
		Serial.println("out");
		analogWrite(MOTOR_PWM, 0);
		encMax = encPos;
	}
	else {
		analogWrite(MOTOR_PWM, speed);
	}
}

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
	Serial.print("encPos: ");
	Serial.println(DragBlades.encPos);
	Serial.printf("Encoder max: %d\nEncoder min: %d\n", encMax, encMin);
	Serial.print("count: ");
	Serial.println(count);
#endif
	if (count >= SETPOINT_INAROW) {
		count = 0;
		return true;
	}
	else {
		return false;
	}
}

void DragBladesClass::motorTest()
{
#if !LIMITSWITCHES_DETATCHED
	while (digitalRead(LIM_OUT)) {
		motorDo(OUTWARD, DEADZONE_MAX);
	}
	Serial.println("OUT!");
	encMax = encPos;
	while (digitalRead(LIM_IN)) {
		motorDo(INWARD, DEADZONE_MAX);
	}	
	Serial.println("IN!");
	encMin = encPos;

#endif
	
	while ((Serial.available() == 0) && !motorGoTo(encMin)) {
		delay(MOTORTEST_DELAY_MS);
	}
	motorDo(OUTWARD, 0);

	GUI.eatYourBreakfast();
	delay(300);
	while ((Serial.available() == 0) && !motorGoTo(map(25, 0, 100, encMin, encMax))) {
		delay(MOTORTEST_DELAY_MS);
	}
	motorDo(OUTWARD, 0);
	GUI.eatYourBreakfast();
	delay(300);
	while ((Serial.available() == 0) && !motorGoTo(map(50, 0, 100, encMin, encMax))) {
		delay(MOTORTEST_DELAY_MS);
	}
	motorDo(OUTWARD, 0);
	GUI.eatYourBreakfast();
	delay(300);
	while ((Serial.available() == 0) && !motorGoTo(map(75, 0, 100, encMin, encMax))) {
		delay(MOTORTEST_DELAY_MS);
	}
	motorDo(OUTWARD, 0);
	GUI.eatYourBreakfast();
	delay(200);
	while ((Serial.available() == 0) && !motorGoTo(map(100, 0, 100, encMin, encMax))) {
		delay(MOTORTEST_DELAY_MS);
	}
	motorDo(OUTWARD, 0);
	GUI.eatYourBreakfast();
	delay(300);
	while ((Serial.available() == 0) && !motorGoTo(map(75, 0, 100, encMin, encMax))) {
		delay(MOTORTEST_DELAY_MS);
	}
	motorDo(OUTWARD, 0);
	GUI.eatYourBreakfast();
	delay(300);
	while ((Serial.available() == 0) && !motorGoTo(map(50, 0, 100, encMin, encMax))) {
		delay(MOTORTEST_DELAY_MS);
	}
	motorDo(OUTWARD, 0);
	GUI.eatYourBreakfast();
	delay(300);
	while ((Serial.available() == 0) && !motorGoTo(map(25, 0, 100, encMin, encMax))) {
		delay(MOTORTEST_DELAY_MS);
	}
	motorDo(OUTWARD, 0);
	GUI.eatYourBreakfast();
	delay(300);
	while ((Serial.available() == 0) && !motorGoTo(encMin)) {
		delay(MOTORTEST_DELAY_MS);
	}
	motorDo(OUTWARD, 0);
	GUI.eatYourBreakfast();
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
	bool dir;
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


void DragBladesClass::powerTest() {
	while (!(Serial.available() > 0)) {
		while ((Serial.available() == 0) && !motorGoTo(encMin)) {
			delay(MOTORTEST_DELAY_MS);
		}
		while ((Serial.available() == 0) && !motorGoTo(encMax)) {
			delay(MOTORTEST_DELAY_MS);
		}
	}
}

int DragBladesClass::myAbs(int x) {
	if (x >= 0) {
		return x;
	}
	else {
		return -x;
	}
}


DragBladesClass DragBlades;