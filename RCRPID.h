#pragma once
#include "GlobVars.h"

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

class RCRPID
{
public:
	RCRPID(volatile int*, int*, int*, float, float, float, float, int, int);//constructor
	void Compute();
private:
	float kp;                  // * (P)roportional Tuning Parameter
	float ki;                  // * (I)ntegral Tuning Parameter
	float kd;                  // * (D)erivative Tuning Parameter
	float n;

	volatile int *myInput; 
	int *myOutput;  
	int *mySetpoint; 

	float ITerm, lastError, lastDTerm=0;
	unsigned long lastTime;
	int outMin, outMax;

};

