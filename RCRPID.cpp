#include "RCRPID.h"



RCRPID::RCRPID(volatile int* input, int* output, int* setpoint, float Kp, float Ki, float Kd, float N, int lower, int upper)
{
	myOutput = output;
	myInput = input;
	mySetpoint = setpoint;

	kp = Kp;
	ki = Ki;
	kd = Kd;
	n = N;

	outMin = lower;
	outMax = upper;
}


void RCRPID::Compute() {
	//error variables
	int input, error;
	float output, dError, dt, DTerm=0;
	unsigned long now;

	input = *myInput;
	error = *mySetpoint - input;
	now = micros();
	dt = (float)(now - lastTime) / 1000000;
	dError =  (error - lastError);
	ITerm += (ki * (float)dError * dt);
	DTerm = (float)dError / dt;
	DTerm = kd * DTerm;
	//DTerm = (float)(DTerm - lastDTerm)*dt;
	//DTerm = n*(error*kd - DTerm);
	if (ITerm > outMax) ITerm = outMax;
	else if (ITerm < outMin) ITerm = outMin;

	/*Compute PID Output*/
	if (dt > PID_RESET_TIME_S) {
		output = kp*error;
		ITerm = 0;
	}
	else {
		output = kp * error + ITerm + DTerm;
	}

	if (output > outMax) output = outMax;
	else if (output < outMin) output = outMin;
	if ((output < DEADZONE_MAX) && (output > DEADZONE_MIN)) { 
		output = DEADZONE_MAX+DEADZONE_BOOST; 
	}
	else if ((output > -DEADZONE_MAX) && (output < -DEADZONE_MIN)) {
		output = -DEADZONE_MAX- DEADZONE_BOOST;
	}
	*myOutput = output;

#if DEBUG_PIDCOMPUTE
	Serial.println();
	Serial.println("PID_COMPUTE--------------");
	Serial.print("mtrSpdCmd: ");
	Serial.println(output);
	Serial.print("kp*error: ");
	Serial.println(kp*error);
	Serial.print("Iterm: ");
	Serial.println(ITerm,5);
	Serial.print("DTerm: ");
	Serial.println(DTerm);
	Serial.print("dt: ");
	Serial.println(dt,5);
#endif

	/*Remember some variables for next time*/
	lastDTerm = DTerm;
	lastError = error;
	lastTime = now;
}

