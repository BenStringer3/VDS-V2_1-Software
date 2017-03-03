
#include "Adafruit_BMP280.h"
#include <math.h>
#include "GlobVars.h"                                      //All the VDS settings and constants are here
#include "MatrixMath.h"
#include <SdFat.h>
#include <SPI.h>
#include "RCRClasses.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Wire.h>
#include "RCRPID.h"

/********************BEGIN GLOBAL VARIABLES********************/
/*General Variables*/
//char response;

/*Kalman variables*/
float q_k[3][3] = {                                             //Constants used in Kalman calculations
  { 1, 0, 0 },
  { 0, 0.02, 0 },
  { 0, 0, 0.2 }
};
float r_k[3][3] = {
  { 0.5, 0, 0 },
  { 0, 4, 0 },
  { 0, 0, 7 }
};
/*********************END GLOBAL VARIABLES*********************/


/* _____      _
 / ____|    | |
| (___   ___| |_ _   _ _ __
\___ \ / _ \ __| | | | '_ \
____) |  __/ |_| |_| | |_) |
|_____/ \___|\__|\__,_| .__/
					  | |
					  |_|*/
void setup(void) {
	bool begin = false;
	//turn on an LED to ensure the Teensy is getting power
	pinMode(LED, OUTPUT);
	digitalWrite(LED, HIGH);
	delay(1000);


	// start serial port at any baud rate (Baud rate doesn't matter to teensy)
	Serial.begin(38400);
	delay(1000);
	//wait for user to begin
	Serial.println("Welcome to VDS\r\nPress 's' to start");
	while (!begin) {
		if (Serial.available() > 0) {
			if (Serial.read() == 's') {
				begin = true;
				GUI.eatYourBreakfast();
			}
		}
	}

	//print out the title
	GUI.printTitle();

	//Initialize BNO055, pressure sensor, rocket settings, and microSD card
	DataLog.init();
	GUI.init();
	DAQ.init(true);

	DragBlades.init();
	attachInterrupt(digitalPinToInterrupt(ENC_A), doEncoder, RISING);
	DragBlades.dragBladesCheck();
	GUI.printMenu();
}  // END setup()
/********************END SETUP FUNCTION********************/


/*__  __       _         _
|  \/  |     (_)       | |
| \  / | __ _ _ _ __   | |     ___   ___  _ __
| |\/| |/ _` | | '_ \  | |    / _ \ / _ \| '_ \
| |  | | (_| | | | | | | |___| (_) | (_) | |_) |
|_|  |_|\__,_|_|_| |_| |______\___/ \___/| .__/
										 | |
										 |_| */
										 /*To add a menu item, add a case statement below and add a print statement in GUI.printMenu*/
void loop(void) {
	char response;
	bool testMode = false;
	bool fullBrakesTest = false;
	if (Serial.available() > 0) {
		switch (Serial.read()) {
		case 'S':
			DataLog.init();
			DAQ.init(false);
			GUI.init();
			DragBlades.dragBladesCheck();
			break;
		case 'D':
			DragBlades.dragBladesCheck();
			break;
		case 'P':
			Serial.println("Power test");
			GUI.eatYourBreakfast();
			DragBlades.powerTest();
			DragBlades.motorDont();
			break;
		case 'I':
			Serial.println("Inching Inward");
			DragBlades.motorDo(INWARD, DEADZONE_MAX + 15);
			delay(250);
			DragBlades.motorDont();
			break;
		case 'O':
			Serial.println("Inching Outward");
			DragBlades.motorDo(OUTWARD, DEADZONE_MAX + 15);
			delay(250);
			DragBlades.motorDont();
			break;
		case 'R':
			GUI.eatYourBreakfast();
			GUI.rocketMenu();
			break;
		case 'C':
			Serial.println("\n\n----- Calibrate BNO055 -----;");
			GUI.eatYourBreakfast();                                       //Flushes serial port
			DAQ.calibrateBNO();
			break;
		case 'A':
			Serial.println("\n\n----- Testing Accelerometer -----;");
			GUI.eatYourBreakfast();                                       //Flushes serial port
			DAQ.testAccelerometer();
			break;
		case 'M':
			GUI.eatYourBreakfast();                                       //Flushes serial port
			Serial.println("\n\n----- Calibrate Motor -----;");
			DragBlades.motorTest();
			break;
		case 'B':
			Serial.println("\n\n----- Testing Barometric Pressure Sensor -----;");
			GUI.eatYourBreakfast();                                       //Flushes serial port
			DAQ.testBMP();
			break;
		case 'F':
			GUI.eatYourBreakfast();                                       //Flushes serial port
			Serial.println("------Choose Flight Mode Settings-----");
			Serial.println("Would you like to enter test mode? (y/n)");
			GUI.eatYourBreakfast();
			while (!(Serial.available() > 0)) {
				//wait
			}
			Serial.readBytes(&response, 1);
			if (response == 'y') {
				DataLog.newFlight(true);
				testMode = true;
			}
			else if (response == 'n') {
				DataLog.newFlight(false);
				testMode = false;
			}
			else {
				Serial.println("Not a valid response");
				return;
			}
			Serial.println("Would you like to enter full-brake test mode? (y/n)");
			GUI.eatYourBreakfast();
			while (!(Serial.available() > 0)) {
				//wait
			}
			Serial.readBytes(&response, 1);
			if (response == 'y') {
				fullBrakesTest = true;
			}
			else if (response == 'n') {
				fullBrakesTest = false;
			}
			else {
				Serial.println("Not a valid response");
				return;
			}


			if (((!BMP_GO || !BNO_GO || !DragBlades_GO) && !testMode) || !SD_GO) {       //If sensors are not initialized, send error, do nothing
				Serial.println("Cannot enter flight mode. A sensor or sd card is not initialized.");
				DataLog.logError(SENSOR_UNIT);
			}
			else {
				Serial.println("Entering Flight Mode;");                //If sensors are initialized, begin flight mode

				if (!testMode) {                                          //If not in test mode, zero the pad altitude
					Serial.println("Test Mode: OFF");
					DAQ.setPadAlt();
				}
				else {
					Serial.println("Test Mode: ON");
				}
				if (fullBrakesTest) {
					Serial.println("Full-brakes Test: ON");
				}
				else {
					Serial.println("Full-brakes Test: OFF");
				}
				delay(2000);                                            //pause for dramatic effect....
				flightMode(testMode, fullBrakesTest);                                           //Initiate Flight Mode
			}
			break;
		default:
			Serial.println("Unkown code received - main menu");
			Serial.println(response);
			DataLog.logError(INVALID_MENU);
			break;
		}
		GUI.eatYourBreakfast();
		GUI.printMenu();
	}
} // END loop()
/*********************END LOOP FUNCTION*********************/


/********************BEGIN FUNCTION DEFINITIONS********************/


/**************************************************************************/
/*!
@brief  Launch and test sequence.
Author: Jacob & Ben
*/
/**************************************************************************/
void flightMode(bool testMode, bool fullBrakesTest) {
	struct stateStruct rawState, filteredState;
	int airBrakesEncPos_val = 0;
	float vSPP_val = 0;
	Serial.println("asdfasdf");
	GUI.eatYourBreakfast();
	while ((Serial.available() == 0) && DAQ.getRawState(&rawState, testMode)) {
		vSPP_val = vSPP(rawState.alt, rawState.vel);
		airBrakesEncPos_val = DragBlades.airBrakesGoToEncPos(rawState.vel, vSPP_val);
		if (!fullBrakesTest) {
			DragBlades.motorGoTo(airBrakesEncPos_val);
		}
		else {
			if ((rawState.accel < 0) && (rawState.alt > 150) && (rawState.vel > 0)) {
				DragBlades.motorGoTo(DragBlades.encMax);
			}
			else {
				DragBlades.motorGoTo(DragBlades.encMin);
			}
		}
		//kalman(0, rawState, &filteredState);                   //feeds raw state into kalman filter and retrieves new filtered state.

		//LOG DATA
		DAQ.getAdditionalData(rawState, filteredState, testMode);
		DataLog.supStat.vSPP = vSPP_val;
		DataLog.logData(testMode);
		if (!fullBrakesTest) {  //call motorGoTo again to make sure the blades didn't pass their setpoint 
			DragBlades.motorGoTo(airBrakesEncPos_val);
		}
		else {
			if ((rawState.accel < 0) && (rawState.alt > 150) && (rawState.vel > 0)) {
				DragBlades.motorGoTo(DragBlades.encMax);
			}
			else {
				DragBlades.motorGoTo(DragBlades.encMin);
			}
		}					

#if DEBUG_FLIGHTMODE
		Serial.println("");
		Serial.println("FLIGHTMODE-------------------");
		GUI.printState(rawState, "raw state");                          //If in DEBUG_FLIGHTMODE mode, prints raw state data for evaluation.
		Serial.println("");
		GUI.printState(filteredState, "filtered state");                //If in DEBUG_FLIGHTMODE mode, prints filtered state data for evaluation.
#endif
	}
	Serial.println("End of flight mode. Returning drag blades...");
	GUI.eatYourBreakfast();
	while (digitalRead(LIM_IN) && (Serial.available() == 0)) {
		delay(MOTORTEST_DELAY_MS);
		DragBlades.motorDo(INWARD, DEADZONE_MAX + 10);
	}
	DragBlades.motorDont();
} // END flightMode()


  /**************************************************************************/
  /*!
  @brief  Velocity of the Set Point Path (SPP). Returns a velocity at which the vehicle should be moving for a given
  altitude argument. The SPP is also piecewise.
  Author: Ben
  */
  /**************************************************************************/
float vSPP(float alt, float vel) {
	float returnVal, x;
	x = 1 - exp(-2 * rocket.Cmin *(rocket.targetAlt - alt));
	if (x < 0) {
		x = 0;
	}

	if (vel < rocket.interVel) {
		returnVal = velocity_h(rocket.Cmin, alt, 0, rocket.targetAlt);
	}
	else if (vel >= rocket.interVel) {
		if (alt < rocket.targetAlt) {
			returnVal = velocity_h(rocket.Cspp, alt, rocket.interVel, rocket.interAlt);
		}
		else {
			returnVal = 0;
		}
	}
	else {
		returnVal = 0;
	}
#if DEBUG_V_SPP
	Serial.println("");
	Serial.println("vSPP------------------");
	Serial.print("x: ");
	Serial.println(x);
	Serial.print("h0: ");
	Serial.println(h0);
	Serial.print("vSPP: ");
	Serial.println(returnVal);
#endif
	return returnVal;
		}


/**************************************************************************/
/*!
@brief  Calculates velocity as a function of altitude
Author: Ben
*/
/**************************************************************************/
float velocity_h(float c, float alt, float v0, float h0) {
	float K1, K2, x;
	K1 = -1 / sqrt(c*G)*atan(v0*sqrt(c / G));
	K2 = h0 - 1 / c*log(cos(sqrt(c*G)*K1));
	x = 1 - exp(-2 * c*(K2 - alt));
	if (x < 0) {
		x = 0;
	}
	return exp(c*(K2 - alt))*sqrt(G / c)*sqrt(x);
}


/*
_  __     _                         ______                _   _
| |/ /    | |                       |  ____|              | | (_)
| ' / __ _| |_ __ ___   __ _ _ __   | |__ _   _ _ __   ___| |_ _  ___  _ __  ___
|  < / _` | | '_ ` _ \ / _` | '_ \  |  __| | | | '_ \ / __| __| |/ _ \| '_ \/ __|
| . \ (_| | | | | | | | (_| | | | | | |  | |_| | | | | (__| |_| | (_) | | | \__ \
|_|\_\__,_|_|_| |_| |_|\__,_|_| |_| |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/
*/
/**************************************************************************/
/*!
@brief  Filters the state of the vehicle
Author: Ben, Denny, and Lydia
*/
/**************************************************************************/
void kalman(int16_t encPos, struct stateStruct rawState, struct stateStruct* filteredState) {
	static float x_k[3] = { 0, 0, 0 };
	static unsigned long lastTime;
	float delta_t;
	float z_k[3];
	static float p_k[3][3] = {
	  { 0, 0, 0 },
	  { 0, 0, 0 },
	  { 0, 0, 0 }
	};
	float k_gain[3][3];
	float placeHolder_3_1[3];
	float placeHolder_3_3_1[3][3];
	float placeHolder_3_3_2[3][3];
	float placeHolder_3_3_3[3][3];
	float u_k;
	float c_d;
	float area;
	float q;
	float b_k[3];
	float f_k[3][3] = {
	  { 1, 0, 0 },
	  { 0, 1, 0 },
	  { 0, 0, 0 }
	};

	//x_k[0] = filteredState->alt;
	//x_k[1] = filteredState->vel;
	//x_k[2] = filteredState->accel;

	z_k[0] = rawState.alt;
	z_k[1] = rawState.vel;
	z_k[2] = rawState.accel;

	delta_t = (float)(rawState.time - lastTime) / TIME_DIVISOR;
	lastTime = rawState.time;

	b_k[0] = delta_t*delta_t;
	b_k[0] = b_k[0] / 2;
	b_k[1] = delta_t;
	b_k[2] = 1;

	f_k[0][1] = delta_t;

#if DEBUG_KALMAN
	Serial.println("KALMAN---------------------");
	Serial.print("delta_t = "); //temp
	Serial.println(delta_t); //temp
	Serial.print("t = "); //temp
	Serial.println(rawState.time); //temp
	Matrix.Print(x_k, 3, 1, "x_k"); //temp
	Matrix.Print((float*)f_k, 3, 3, "f_k"); //temp
	Matrix.Print(b_k, 3, 1, "b_k"); //temp
	Matrix.Print(z_k, 3, 1, "z_k"); //temp
#endif

  //calculate what Kalman thinks the acceleration is
	c_d = rocket.Cd_r *(1 - encPos / ENC_RANGE) + encPos*rocket.Cd_b / ENC_RANGE;
	area = rocket.Ar*(1 - encPos / ENC_RANGE) + encPos*rocket.Ab / ENC_RANGE;
	q = RHO * rawState.vel * rawState.vel / 2;
	u_k = -9.81 - c_d * area * q / rocket.dryMass;

	// if acceleration > 10m/s^2 the motor is probably burning and we should add that in to u_k
	if (z_k[2] > 10) {
		//Serial.println("Burn Phase!"); //errorlog
		u_k += rocket.avgMotorThrust / (rocket.dryMass + rocket.propMass / 2);
	}
	else if ((z_k[0] < 20) && (z_k[0] > -20)) {
		u_k = 0;
		//Serial.print("On pad "); //errorlog
		//Serial.println(z_k[0]);
	}
	if isnan(u_k) { //caused by velocity being nan //errorlog
#if DEBUG_KALMAN
		Serial.println("u_k is nan!");
#endif
		DataLog.logError(NAN_UK);
		u_k = 0;
	}

#if DEBUG_KALMAN
	Serial.println("");
	Serial.print("u_k = ");
	Serial.println(u_k);
	Serial.print("c_d = "); //temp
	Serial.println(c_d); //temp
	Serial.print("area = "); //temp
	Serial.println(area); //temp
	Serial.print("q = "); //temp
	Serial.println(q); //temp
#endif

  //Predict----------------------------
  //x_k = u_k*b_k + f_k*x_k
	Matrix.Scale((float*)b_k, 3, 1, u_k);
	Matrix.Multiply((float *)f_k, (float *)x_k, 3, 3, 1, (float*)placeHolder_3_1);
	Matrix.Add((float*)b_k, (float*)placeHolder_3_1, 3, 1, (float*)x_k);

#if DEBUG_KALMAN
	Matrix.Print(b_k, 3, 1, "u_k*b_k");
	Matrix.Print(placeHolder_3_1, 3, 1, "f_k*x_k");
	Matrix.Print(x_k, 3, 1, "x_k predict");
#endif

	//p_k = q_k + f_k*p_k*T(f_k)
	Matrix.Multiply((float*)f_k, (float*)p_k, 3, 3, 3, (float*)placeHolder_3_3_1);
	Matrix.Transpose((float*)f_k, 3, 3, (float*)placeHolder_3_3_2);
	Matrix.Multiply((float*)placeHolder_3_3_1, (float*)placeHolder_3_3_2, 3, 3, 3, (float*)placeHolder_3_3_3);
	Matrix.Add((float*)placeHolder_3_3_3, (float*)q_k, 3, 3, (float*)p_k);

#if DEBUG_KALMAN
	Matrix.Print((float*)p_k, 3, 3, "p_k predict");
#endif

	//Kalman Gain------------------------
	//p_k*T(h_k) / (r_k + h_k * p_k * T(h_k)) ==
	//p_k / (r_k + p_k)    ..When h_k = eye(3)
	Matrix.Add((float*)r_k, (float*)p_k, 3, 3, (float*)placeHolder_3_3_1);
	Matrix.Invert((float*)placeHolder_3_3_1, 3);
	Matrix.Multiply((float*)p_k, (float*)placeHolder_3_3_1, 3, 3, 3, (float*)k_gain);

#if DEBUG_KALMAN
	Matrix.Print((float*)k_gain, 3, 3, "kalman gain");
	//Matrix.Print((float*)placeHolder_3_3_1, 3, 3, "1 / (r_k + p_k)");
#endif

  //Update-----------------------------
  //x_k = k_gain * (z_k - x_k) + x_k
	Matrix.Subtract((float*)z_k, (float*)x_k, 3, 1, (float*)placeHolder_3_3_1);
	Matrix.Multiply((float*)k_gain, (float*)placeHolder_3_3_1, 3, 3, 1, (float*)placeHolder_3_3_2);
	Matrix.Add((float*)x_k, (float*)placeHolder_3_3_2, 3, 1, x_k);

	//p_k = p_k - k_gain * p_k
	Matrix.Multiply((float*)k_gain, (float*)p_k, 3, 3, 1, (float*)placeHolder_3_3_1);
	Matrix.Subtract((float*)p_k, (float*)placeHolder_3_3_1, 3, 1, (float*)p_k);

	filteredState->alt = x_k[0];
	filteredState->vel = x_k[1];
	filteredState->accel = x_k[2];
	filteredState->time = rawState.time;
	} // END kalman()






	  /**************************************************************************/
	  /*!
	  @brief  This is an ISR! it is called when the pin belonging to ENC_A sees a rising edge
	  This functions purpose is to keep track of the encoder's position
	  Author: Ben
	  */
	  /**************************************************************************/
void doEncoder(void) {
	/* If pinA and pinB are both high or both low, it is spinning
	forward. If they're different, it's going backward.*/
	if (digitalRead(ENC_A) == digitalRead(ENC_B)) {
		DragBlades.encPos--;
	}
	else {
		DragBlades.encPos++;
	}
}

