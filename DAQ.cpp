// 
// 
// 

#include "RCRClasses.h"


void DAQClass::init(bool bnoToo)
{
	Serial.println("\r\n-------DAQ.init-------");
	/********************INITIALIZE OR TEST pressure sensor********************/
#if !BMP280
	Serial.println("Initializing BMP180");
	if (!bmp.begin()) {                                           //Determine if BMP180 is initialized and ready to be used
		BMP_GO = false;
		Serial.println("NO Bmp180 DETECTED!");
	}
	else {
		BMP_GO = true;
		Serial.println("Bmp180 Initialized");
	}
#else
	Serial.println("Initializing BMP280");
	if (!bme.begin()) {
		BMP_GO = false;
		Serial.println("NO Bmp280 DETECTED!");
	}
	else {
		BMP_GO = true;
		Serial.println("Bmp280 Initialized");
	}
#endif
	/********************END TESTING OF pressure sensor********************/

	/********************INITIALIZE OR TEST BNO055********************/
	Serial.println("Initializing BNO055");
	if (bnoToo) {
		if (!bno.begin()) {                                           //Determine if BNO055 is initialized and ready to be used
			Serial.println("NO Bno055 DETECTED!");
		}
		else {
			bno.setExtCrystalUse(true);
			Serial.println("Bno055 Initialized");
		}
		BNO_GO = false;
	}
	/********************END TESTING OF BNO055********************/

	//initialize past raw states
	for (unsigned int i = 0; i < BUFF_N; i++) {
		pastRawStates[i].alt = (float)(0);
		pastRawStates[i].vel = (float)(0);
		pastRawStates[i].accel = (float)(0);
		pastRawStates[i].time = (unsigned long)(0);
	}
}


/**************************************************************************/
/*!
@brief  Gathers data from the desired source (Sensors or file).  Dependent on TEST_MODE
Author: Jacob & Ben
*/
/**************************************************************************/
bool DAQClass::getRawState(struct stateStruct* rawState, bool testMode) {
	bool returnVal;
	if (testMode) {                                                  //If file is in test mode, retrieve sensor data from data file with past flight data
		if (!DataLog.readCSV(rawState)) {
			Serial.println("end of flight");
			delay(1000);
			returnVal = false;
		}
		else {
			delay(MOTORTEST_DELAY_MS);
			returnVal = true;
		}
	}
	else {
		//get raw altitude
#if !BMP280
		rawState->alt = altitude_plz() - padAlt;
#else
		rawState->alt = bme.readAltitude(SEALVL_PRESS) - padAlt;
#endif
		if (timeOverflow) {
			rawState->time = millis() * 100;             //Retrieves time from millis() function, stores within rawState
		}
		else {
			rawState->time = micros()/10;
		}

		if (rawState->time > 420000000) {
			timeOverflow = true;
		}

		//get raw acceleration  
		rawState->accel = getAcceleration();                          //Retrieves acceleration from bno055 sensor, stores within rawState
		returnVal = true;
	}

	rawState->vel = calculateVelocity(*rawState);                 //Calculates velocity using algorithm.  Takes prior acceleration and velocity values from pastRawStates

#if DEBUG_RAWSTATE
	Serial.println();
	Serial.println("RAW STATE--------------------");
	GUI.printState(*rawState, "raw state");                            //If in DEBUG_RAWSTATE mode, prints raw state data for evaluation.
#endif
	return returnVal;
} // END getRawState()


/**************************************************************************/
/*!
@brief  Checks if Bmp180 has a reading ready, retrieves reading and requests a new readings if yes, returns false if not ready yet
Pronounced "altitude please".
Author: Ben
*/
/**************************************************************************/
#if !BMP280
float DAQClass::altitude_plz(void) {
	float returnVal = 0;
	float pressure_kPa;
	float pressure_; //units are Pa*10?

	if (bmp.RCR_readyYet()) {

		bmp.RCR_getPressure(&pressure_kPa);                         //picks up the pressure reading from the Bmp180, then puts in a request for a new one
#if DEBUG_ALTITUDEPLZ
		Serial.print("RCR_getPressure returned: ");
		Serial.print(pressure_kPa);
		Serial.print("  kPa at t = ");
		Serial.println(millis());
#endif
		pressure_ = pressure_kPa / 100.0F;
		lastAlt = bmp.pressureToAltitude(SENSORS_PRESSURE_SEALEVELHPA, pressure_);
		returnVal = lastAlt;
		//Serial.print("ready: ");
		//Serial.println(returnVal);
#if DEBUG_ALTITUDEPLZ
		Serial.print("RCR_getPressure returned: ");
		Serial.print(pressure_kPa);
		Serial.print("  kPa at t = ");
		Serial.println(millis());
		Serial.print("  altitude = ");
		Serial.println(returnVal);
#endif
	}
	else {
		returnVal = lastAlt;
	}
	return returnVal;
} // END altitude_plz()
#endif


  /**************************************************************************/
  /*!
  @brief  Returns the vertical acceleration as a floating point value
  Author: Jacob
  */
  /**************************************************************************/
float DAQClass::getAcceleration(void) {
	imu::Vector<3> gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);        //Creates vector to store acceleration from gravity components
	imu::Vector<3> linear = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);     //Creates vector to store linear acceleration components
	float linearDotGravity = 0,  verticalAcceleration = 0;
	float xG = 0, yG = 0, zG = 0, xL = 0, yL = 0, zL = 0;

	xG = (float)gravity.x();                                                        //Stores most recent x-component of acceleration by gravity
	yG = (float)gravity.y();                                                        //Stores most recent y-component of acceleration by gravity
	zG = (float)gravity.z();                                                        //Stores most recent z-component of acceleration by gravity

	xL = (float)linear.x();                                                         //Stores most recent x-component of linear acceleration
	yL = (float)linear.y();                                                         //Stores most recent y-component of linear acceleration
	zL = (float)linear.z();                                                         //Stores most recent z-component of linear acceleration

	linearDotGravity = (xG*xL) + (yG*yL) + (zG*zL);                                     //Calculates dot product of linear acceleration and acceleration from gravity vectors                                      //Calculates magnitude of acceleration from gravity vector.

	verticalAcceleration = linearDotGravity / 9.81;                                 //Finds the acceleration in the direction of gravity.


	DataLog.supStat.rollAxisGrav = xG;
	DataLog.supStat.yawAxisGrav = yG;
	DataLog.supStat.pitchAxisGrav = zG;

	DataLog.supStat.rollAxisLin = xL;
	DataLog.supStat.yawAxisLin = yL;
	DataLog.supStat.pitchAxisLin = zL;

	testCalibration();

	return verticalAcceleration;                                                    //Returns calculated vertical acceleration.
} // END getAcceleration();

  /*OVERLOADED VERSION.  TAKES IN THE TWO VECTORS AND RETURNS THE VERTICAL ACCELERATION :: USED FOR TESTING PURPOSES*/
float DAQClass::getAcceleration(imu::Vector<3> gravity, imu::Vector<3> linear) {
	float linearDotGravity = 0, theta = 0, defOfProduct = 0,  verticalAcceleration = 0, magL = 0, magG = 0;
	float xG = 0, yG = 0, zG = 0, xL = 0, yL = 0, zL = 0;

	xG = (float)gravity.x();                                                        //Stores most recent x-component of acceleration by gravity
	yG = (float)gravity.y();                                                        //Stores most recent y-component of acceleration by gravity
	zG = (float)gravity.z();                                                        //Stores most recent z-component of acceleration by gravity

	xL = (float)linear.x();                                                         //Stores most recent x-component of linear acceleration
	yL = (float)linear.y();                                                         //Stores most recent y-component of linear acceleration
	zL = (float)linear.z();                                                         //Stores most recent z-component of linear acceleration

	linearDotGravity = (xG*xL) + (yG*yL) + (zG*zL);                                     //Calculates dot product of linear acceleration and acceleration from gravity vectors

	magL = pow(((xL*xL) + (yL*yL) + (zL*zL)), 0.5);                                      //Calculates magnitude of linear acceleration vector.
	magG = pow(((xG*xG) + (yG*yG) + (zG*zG)), 0.5);                                      //Calculates magnitude of acceleration from gravity vector.

	defOfProduct = linearDotGravity / (magL*magG);                                  //Calculates the cosine value using the definition of a dot product.

	theta = acos(defOfProduct);                                                     //Calculates theta using the arc cosine of the previously calculated vosine value.
	theta = (theta * 180) / PI;                                                         //Converts theta from radians to degress.

	verticalAcceleration = linearDotGravity / magG;                                 //Finds the acceleration in the direction of gravity.

																					/* Display the used acceleration from gravity*/
	Serial.print("Gravity Used <x,y,z>: ");
	Serial.print("<");
	Serial.print(xG);                                //
	Serial.print(",");
	Serial.print(yG);                                //
	Serial.print(",");
	Serial.print(zG);                                //
	Serial.println(">;");

	/* Display the calculated theta*/
	Serial.print("Theta: ");
	Serial.println(theta);

	/* Display the calculated vertical acceleration*/
	Serial.print("verticalAcceleration: ");
	Serial.print(verticalAcceleration);

	Serial.println("");

	return verticalAcceleration;                                                    //Returns calculated vertical acceleration.
} // END getAcceleration();  OVERLOADED VERSION


  /**************************************************************************/
  /*!
  @brief  Calculates a velocity value using altitude data from BMP180 and acceleration data fromm BNO055.
  Author: Jacob & Ben
  - Algorithm developed by Ben Stringer, function written by Jacob Cassady
  */
  /**************************************************************************/
float DAQClass::calculateVelocity(struct stateStruct rawState) { //VARIABLES NEEDED FOR CALULATION
	float sumTimes = 0, sumTimes2 = 0, sumAlt = 0, sumAltTimes = 0, leftSide = 0;
	float rightSide = 0, numer = 0, denom = 0, velocity = 0, pastTime = 0, newTime = 0;

	//shift new readings into arrays   
	for (int i = BUFF_N; i > 0; i--) {
		copyState(&pastRawStates[i], &pastRawStates[i - 1]);           //copyState(1,2) deep copies information from struct 1 into struct 2.
	}
	rawState.buff_t = 0;
	copyState(&pastRawStates[0], &rawState);                      //Moves newest state into the 0 position of pastRawStates array.

																  //time relative to the current moment
	for (int i = BUFF_N; i > 0; i--) {
		pastTime = (float)pastRawStates[i - 1].time;
		newTime = (float)rawState.time;
		pastRawStates[i - 1].buff_t = (pastTime - newTime) / (float)TIME_DIVISOR;   //Calculates buff_t values for pastRawStates array (sec)
	}

#if DEBUG_VELOCITY && DEBUG_EMERGENCY
	Serial.println("");
	Serial.println("Past states post-shift");
	GUI.printPastStates(pastRawStates);                             //If in DEBUG_VELOCITY and DEBUG_EMERGENCY, print all pastRawStates for verification of function output
#endif

																//FIND SUMS FOR BMP
	for (unsigned int i = 0; i < BUFF_N; i++) {                   //Calculates sums for left side of velocity equation.
		sumTimes += (float)(pastRawStates[i].buff_t);
		sumTimes2 += (float)((pastRawStates[i].buff_t) * (pastRawStates[i].buff_t));
		sumAlt += pastRawStates[i].alt;
		sumAltTimes += ((float)pastRawStates[i].buff_t * pastRawStates[i].alt);
	}

	//CALCULATE LEFT SIDE OF EQUATION
	numer = ((sumTimes * sumAlt) - (BUFF_N * sumAltTimes));
	denom = ((sumTimes*sumTimes) - (BUFF_N * sumTimes2));
	leftSide = numer / denom;

	DataLog.supStat.leftVel = leftSide;                                         //Stores leftSide values for further post-flight analysis.

#if DEBUG_VELOCITY && DEBUG_EMERGENCY                         //Prints header for future rightSide values if in DEBUG_VELOCITY && DEBUG_EMERGENCY modes.
	Serial.println(" ----- rightSide values ----- ");
#endif

	//CALCULATE RIGHT SIDE OF EQUATION
	for (unsigned int i = 0; i <= (BUFF_N / 2); i++) {
		rightSide += 0.5 * (pastRawStates[i].accel + pastRawStates[i + 1].accel) * (pastRawStates[i].buff_t - pastRawStates[i + 1].buff_t);
#if DEBUG_VELOCITY //&& DEBUG_EMERGENCY                       //Reports rightSide values if in DEBUG_VELOCITY && DEBUG_EMERGENCY modes, final value is used for final velocity calculation.
		Serial.print(i);
		Serial.print(") rightSide = ");
		Serial.println(rightSide, 6);
#endif
	}

	DataLog.supStat.rightVel = rightSide;                                         //Stores rightSide values for further post-flight analysis.

#if DEBUG_VELOCITY                                              //Reports velocity equation pieces for debugging if in DEBUG_VELOCITY mode.
	Serial.println();
	Serial.println("VELOCITY--------------------;");
	Serial.print("leftSide: ");
	Serial.print(leftSide, 3);
	Serial.println(";");
	Serial.print("numer: ");
	Serial.print(numer, 3);
	Serial.println(";");
	Serial.print("denom: ");
	Serial.print(denom, 3);
	Serial.println(";");
	Serial.print("rightSide: ");
	Serial.print(rightSide, 6);
	Serial.println(";");
	Serial.print("sumTimes: ");
	Serial.print(sumTimes, 3);
	Serial.println(";");
	Serial.print("sumTimes2: ");
	Serial.print(sumTimes2, 3);
	Serial.println(";");
	Serial.print("sumAlt: ");
	Serial.print(sumAlt, 3);
	Serial.println(";");
	Serial.print("sumAltTimes: ");
	Serial.print(sumAltTimes, 3);
	Serial.println(";");
	Serial.print("Velocity ");
	Serial.print(rightSide + leftSide, 3);
	Serial.println(";");
#endif

	velocity = (leftSide + rightSide);                            //Calculates final velocity value by summing the left and right sides of the equation
	if isnan(velocity) {                                          //logs error if velocity value is given as nan
#if DEBUG_VELOCITY
		Serial.println("vel is nan!");
#endif
		DataLog.logError(NAN_VEL);
		velocity = 0;                                               //Sets returned velocity to zero to minimize damage from egregious reading.
	}
	if ((velocity > MAX_EXP_VEL) || (velocity < MIN_EXP_VEL)) {           //logs error if velocity value is egregiously too high or low.
#if DEBUG_VELOCITY
		Serial.print("Velocity non-nominal! = ");
		Serial.println(velocity);
#endif
		DataLog.logError(NONNOM_VEL);
		velocity = 0;                                               //Sets returned velocity to zero to minimize damage from egregious reading.
	}
	return velocity;
}// END calculateVelocity()


 /**************************************************************************/
 /*!
 @brief  Menu Function.  Displays altitude values from pressure sensor.
 Author: Jacob
 */
 /**************************************************************************/
void DAQClass::testBMP(void) {
	while (Serial.available() <= 0) {
		Serial.print("Current altitude: ");
#if !BMP280
		Serial.printf("%0.3f", altitude_plz());
#else
		Serial.printf("%0.3f", bme.readAltitude(SEALVL_PRESS));
#endif
		Serial.println(";");
	}
}


/**************************************************************************/
/*!
@brief  Menu Function.  Displays different sensor values from the BNO055 as well as the calculated vertical acceleration.
Author: Jacob
*/
/**************************************************************************/
void DAQClass::testAccelerometer(void) {
	while (Serial.available() <= 0) {
		// Possible vector values can be:
		// - VECTOR_ACCELEROMETER - m/s^2
		// - VECTOR_MAGNETOMETER  - uT
		// - VECTOR_GYROSCOPE     - rad/s
		// - VECTOR_EULER         - degrees
		// - VECTOR_LINEARACCEL   - m/s^2
		// - VECTOR_GRAVITY       - m/s^2
		imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);        //Creates a vector which stores orientation values.
		imu::Vector<3> linear = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL); //Creates a vector which stores linear acceleration values.
		imu::Vector<3> gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);      //Creates a vector which stores orientation values.

																					  /* Display the current acceleration from gravity*/
		Serial.printf("roll: %.3f pitch: %.3f yaw: %.3f\r\n", euler.x(), euler.y(), euler.z());

		Serial.printf("x grav: %.3f y grav: %.3f z grav: %.3f\r\n", gravity.x(), gravity.y(), gravity.z());

		Serial.printf("x lin: %.3f y lin: %.3f z lin: %.3f\r\n", linear.x(), linear.y(), linear.z());
		delay(2000);
	}
}

/**************************************************************************/
/*!
@brief  Menu Function.  Enters program into a calibration mode, requiring the BNO's acceleration calibration
value to reach 3 before exiting.
Author: Jacob
*/
/**************************************************************************/
void DAQClass::calibrateBNO(void) {
	uint8_t system, gyro, accel, mag = 0;
	int calibrationCount = 0;

	Serial.println("Calibrating BNO055...;");

	while ((calibrationCount < 5) && (Serial.available() == 0)) {                                                    //Waits until it recieves 5 confirmations that sensor's accelerometer is calibrated.
		bno.getCalibration(&system, &gyro, &accel, &mag);                             //Retrieves calibration values from sensor.
		Serial.print("CALIBRATION: Sys=");                                            //Prints calibration values to serial for use while calibrating.
		Serial.print(system, DEC);
		Serial.print(" Gyro=");
		Serial.print(gyro, DEC);
		Serial.print(" Accel=");
		Serial.print(accel, DEC);
		Serial.print(" Mag=");
		Serial.print(mag, DEC);
		Serial.println(";");

		if ((accel > 2) && (gyro > 2) && (mag > 2) && (system > 2)) {                                                                //If the calibration value for accel is 3 or greater than 2, count as confirmation sensor is calibrated.
			calibrationCount += 1;
		}
		else {                                                                      //If calibration value is too low, disregard all prior confirmations and restart count.
			calibrationCount = 0;
		}

		delay(300);
	}
	if (calibrationCount >= 5) {
		BNO_GO = true;
	}
} // END calibrateBNO()


  /**************************************************************************/
  /*!
  @brief  Checks if accelerometer is calibrated, logs error if not.
  Author: Jacob
  */
  /**************************************************************************/
void DAQClass::testCalibration(void) {
	uint8_t system, gyro, accel, mag = 0;
	bno.getCalibration(&system, &gyro, &accel, &mag);                               //Retrieves calibration values from sensor.
	
		if (system < 3) {                                                                  
			DataLog.logError(UNCALIBRATED_BNO);
		}
		if (gyro < 3) {
			DataLog.logError(UNCALIBRATED_GYRO);
		}
		if (accel < 3) {
			DataLog.logError(UNCALIBRATED_ACCEL);
		}
		if (mag < 3) {
			DataLog.logError(UNCALIBRATED_MAGN);
		}

} // END testCalibration


  /**************************************************************************/
  /*!
  @brief  grabs gyro and euler data, also puts state information into supStat for logging
  Author: Ben
  */
  /**************************************************************************/
void DAQClass::getAdditionalData(stateStruct rawState, stateStruct filteredState, bool testMode) {
	if (!testMode) {
		imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
		imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
		DataLog.supStat.roll = euler.x();
		DataLog.supStat.pitch = euler.y();
		DataLog.supStat.yaw = euler.z();
		DataLog.supStat.rollAxisGyro = gyro.x();
		DataLog.supStat.pitchAxisGyro = gyro.y();
		DataLog.supStat.yawAxisGyro = gyro.z();
	}
	DataLog.supStat.time = rawState.time;
	DataLog.supStat.alt = rawState.alt;
	DataLog.supStat.vel = rawState.vel;
	DataLog.supStat.accel = rawState.accel;
	DataLog.supStat.alt_k = filteredState.alt;
	DataLog.supStat.vel_k = filteredState.vel;
	DataLog.supStat.accel_k = filteredState.accel;
}


/**************************************************************************/
/*!
@brief  Deep copies one state to another
Author: Jacob
*/
/**************************************************************************/
void DAQClass::copyState(struct stateStruct* destination, struct stateStruct* original) {
	destination->alt = original->alt;
	destination->vel = original->vel;
	destination->accel = original->accel;
	destination->time = original->time;
	destination->buff_t = original->buff_t;
} // END copyState()


  /**************************************************************************/
  /*!
  @brief  sets the altitude of the launch pad
  Author: Ben
  */
  /**************************************************************************/
void DAQClass::setPadAlt(void) {
#if !BMP280
	padAlt = altitude_plz();//puts in a request for a reading
	delay(40);
	padAlt = altitude_plz();//retrieves reading
#else
	padAlt = bme.readAltitude(SEALVL_PRESS);
#endif
	Serial.print("Launch pad altitude = ");
	Serial.println(padAlt);
}

DAQClass DAQ;

