#include <math.h>
#include "constants.h"                                      //All the VDS settings and constants are here
#include "MatrixMath.h"
#include <SdFat.h>
#include <SPI.h>
#include "RCRClasses.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Wire.h>
//#include "PID_v1.h"
#include "RCRPID.h"

/********************BEGIN GLOBAL VARIABLES********************/
/*General Variables*/
unsigned long timer = 0;                  
unsigned int stopWatch = 0;
char response;

//volatile int encPos = 0;                                    //Stores most recent position of encoder
int encMin = 0;
int encMax = ENC_RANGE;
int mtrSpdCmd = 0;											//motor speed command
int encPosCmd = 0;											//encoder position command
RCRPID motorPID(&DragBlades.encPos, &mtrSpdCmd, &encPosCmd, KP, KI, KD, KN, -255, 255);
//4.809, 1.8085, -0.45905, -255, 255);//neverrest60

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


/********************BEGIN FUNCTION PROTOTYPES********************/
/*General Functions*/
void flightMode(void);                                          //Begins flightMode sequence.  Dependent on TESTMODE.
void eatYourBreakfast(void);                                    //Clears the serial buffer.. This is helpful for carriage returns and things of that sort that
                                                                //hang around after you got what you wanted.

/*Kalman Functions*/
void kalman(int16_t, struct stateStruct, struct stateStruct*);  //Filters the state of the vehicle.

/*File IO Functions*/
/*********************END FUNCTION PROTOTYPES*********************/




/* _____      _
 / ____|    | |
| (___   ___| |_ _   _ _ __
\___ \ / _ \ __| | | | '_ \
____) |  __/ |_| |_| | |_) |
|_____/ \___|\__|\__,_| .__/
                      | |
                      |_|*/
void setup(void) {

  //turn on an LED to ensure the Teensy is getting power
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);


  // start serial port at any baud rate (Baud rate doesn't matter to teensy)
  Serial.begin(38400);
  delay(1000);
  Serial.println("Serial has begun:");
  Serial.println("...");
  Serial.println("...");

  //print out the title
  GUI.printTitle();

  //Initialize BNO055, BMP180, and microSD card
  systemCheck(true);
  DragBlades.init();
  attachInterrupt(digitalPinToInterrupt(ENC_A), doEncoder, RISING);

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
  if (Serial.available() > 0) {
    switch (Serial.read()) {
    case 'S':
		systemCheck(true);
      break;
    case 'C':
      Serial.println("\n\n----- Calibrate BNO055 -----;");
      eatYourBreakfast();                                       //Flushes serial port
      DAQ.calibrateBNO();
      break;
    case 'A':
      Serial.println("\n\n----- Testing Accelerometer -----;");
      eatYourBreakfast();                                       //Flushes serial port
      DAQ.testAccelerometer();
      break;
    case 'E': 
      eatYourBreakfast();                                       //Flushes serial port
	  Serial.println("\n\n----- Motor Exercise Test -----;");
	  motorExercise();
      break;
	case 'M':
		eatYourBreakfast();                                       //Flushes serial port
		Serial.println("\n\n----- Calibrate Motor -----;");
		motorTest();
		break;
    case 'B':
      Serial.println("\n\n----- Testing Barometric Pressure Sensor -----;");
      eatYourBreakfast();                                       //Flushes serial port
      DAQ.testBMP();
      break;
    case 'F':
      Serial.println("\n\n----- Entering Flight Mode -----;");
      eatYourBreakfast();                                       //Flushes serial port
	  systemCheck(false);
      DataLog.newFlight();
      
      if (((!DAQ.bmp180_init || !DAQ.bno055_init) && !TEST_MODE) || !DataLog.sd_init) {       //If sensors are not initialized, send error, do nothing
        Serial.println("Cannot enter flight mode. A sensor or sd card is not initialized.");
#if DATA_LOGGING
		DataLog.logError(SENSOR_UNIT);
#endif
      } else {
        Serial.println("Entering Flight Mode;");                //If sensors are initialized, begin flight mode
        
        #if !TEST_MODE                                          //If not in test mode, zero the pad altitude
		DAQ.setPadAlt();
        #endif
        
        delay(2000);                                            //pause for dramatic effect....
        flightMode();                                           //Initiate Flight Mode
      }
      break;
    default:
      Serial.println("Unkown code received;");
      Serial.println(response);
#if DATA_LOGGING
	  DataLog.logError(INVALID_MENU);
#endif
      break;
    }
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
void flightMode(void) {
	struct stateStruct rawState, filteredState;
	int airBrakesPercDep_val, airBrakesEncPos_val = 0;
	float vSPP_val = 0;
	while ((Serial.available() == 0) && DAQ.getRawState(&rawState)) {  
		vSPP_val = vSPP(rawState.alt, rawState.vel);
		airBrakesPercDep_val = airBrakesPercDep(rawState.vel, vSPP_val);
		airBrakesEncPos_val = map(airBrakesPercDep_val, 0, 100, encMin, encMax);
		motorGoTo(airBrakesEncPos_val);
		//kalman(0, rawState, &filteredState);                   //feeds raw state into kalman filter and retrieves new filtered state.

		//LOG DATA
		DAQ.getAdditionalData(rawState, filteredState);		
		DataLog.supStat.vSPP = vSPP_val;
		DataLog.supStat.encPos = DragBlades.encPos;
		DataLog.supStat.encPosCmd = encPosCmd;
		DataLog.logData();
		motorGoTo(airBrakesEncPos_val);					//call motorGoTo again to make sure the blades didn't pass their setpoint 

#if DEBUG_FLIGHTMODE
		Serial.println("");
		Serial.println("FLIGHTMODE-------------------");
		GUI.printState(rawState, "raw state");                          //If in DEBUG_FLIGHTMODE mode, prints raw state data for evaluation.
		Serial.println("");
		GUI.printState(filteredState, "filtered state");                //If in DEBUG_FLIGHTMODE mode, prints filtered state data for evaluation.
#endif
	}
	Serial.println("End of flight mode. Returning drag blades...");
	while (!motorGoTo(0)) {
		delay(MOTORTEST_DELAY_MS);
	}
	DragBlades.motorDo(CLOCKWISE, 0);
} // END flightMode()


  /**************************************************************************/
  /*!
  @brief  Returns the percentage that the airbrakes should deploy based on how 
  fast the vehicle is moving and how fast the SPP thinks it should be moving
  Author: Ben
  */
  /**************************************************************************/
float airBrakesPercDep(float vehVel, float sppVel) {
	float returnVal;
	returnVal = -(sppVel - vehVel) * AIRBRAKES_GAIN;
	if (returnVal >= 100) returnVal = 100;
	else if (returnVal <= 0) returnVal = 0;
	return returnVal;
}


  /**************************************************************************/
  /*!
  @brief  Velocity of the Set Point Path (SPP). Returns a velocity at which the vehicle should be moving for a given 
  altitude argument. The SPP is also piecewise.
  Author: Ben
  */
  /**************************************************************************/
float vSPP(float alt, float vel) {
	float returnVal, h0, x;
	x = 1 - exp(-2 * C_MIN*(TARGET_ALTITUDE - alt));
	if (x < 0) {
		x = 0;
	}
	if (vel < INTER_VEL) {
		returnVal = velocity_h(C_MIN, alt, 0, TARGET_ALTITUDE);
	}
	else if (vel >= INTER_VEL) {
		if (alt < TARGET_ALTITUDE) {
			returnVal = velocity_h(C_SPP, alt, INTER_VEL, INTER_ALT);
		}
		else {
			returnVal = 0;
		}
	}
	else{
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

  delta_t = (float)(rawState.time - lastTime)/1000000;
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
  c_d = CD_R*(1 - encPos / ENC_RANGE) + encPos*CD_B / ENC_RANGE;
  area = A_R*(1 - encPos / ENC_RANGE) + encPos*A_B / ENC_RANGE;
  q = RHO * rawState.vel * rawState.vel / 2;
  u_k = -9.81 - c_d * area * q / DRY_MASS;

  // if acceleration > 10m/s^2 the motor is probably burning and we should add that in to u_k
  if (z_k[2] > 10) {
    //Serial.println("Burn Phase!"); //errorlog
    u_k += AVG_MOTOR_THRUST / (DRY_MASS + PROP_MASS/2);
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
#if DATA_LOGGING
    DataLog.logError(NAN_UK);
#endif
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
@brief  Clears the serial buffer.. This
is helpful for carriage returns and things of that sort that
hang around after you got what you wanted.
Author: Ben
*/
/**************************************************************************/
void eatYourBreakfast() {
  while (Serial.available() > 0) {
    delay(2);
    Serial.read();
  }
} // END eatYourBreakfast()


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



/**************************************************************************/
/*!
@brief  This is a quick exercise meant to spin the motor in different ways.
Data collected from this function was used to build a transfer function for the DC motor
WARNING: Do not execute this function if the motor is installed in the VDS as it will try 
to spin past its physical limits and the motor will stall.
Author: Ben
*/
/**************************************************************************/
void motorExercise(void) {
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
			dir = CLOCKWISE;
			spd = 255;
		}
		else if (t < 2000000) {
			dir = CLOCKWISE;
			derp = (float)(t - 1000000) / 1000000;
			spd = derp * 255;
			//Serial.print("derp = ");
			//Serial.println(derp);
		}
		else if (t < 3000000) {
			dir = CLOCKWISE;
			spd = 0;
		}
		else if (t < 4000000) {
			dir = CLOCKWISE;
			spd = deadZoneSpeed;
		}
		else if (t < 5000000) {
			dir = CLOCKWISE;
			derp = (float)(t - 4000000) / 1000000;
			spd = deadZoneSpeed + derp * (255 - deadZoneSpeed);
			//Serial.print("derp = ");
			//Serial.println(derp);
		}
		else if (t < 6000000) {
			dir = COUNTERCLOCKWISE;
			spd = 255;
		}
		else if (t < 7000000) {
			dir = CLOCKWISE;
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


void motorTest(void) {
	while ((Serial.available() == 0) && !motorGoTo(70)) {
		delay(MOTORTEST_DELAY_MS);
	}
	DragBlades.motorDo(CLOCKWISE, 0);

	eatYourBreakfast();
	delay(300);
	while ((Serial.available() == 0) && !motorGoTo(140)) {
		delay(MOTORTEST_DELAY_MS);
	}
	DragBlades.motorDo(CLOCKWISE, 0);
	eatYourBreakfast();
	delay(300);
	while ((Serial.available() == 0) && !motorGoTo(210)) {
		delay(MOTORTEST_DELAY_MS);
	}
	DragBlades.motorDo(CLOCKWISE, 0);
	eatYourBreakfast();
	delay(300);
	while ((Serial.available() == 0) && !motorGoTo(280)) {
		delay(MOTORTEST_DELAY_MS);
	}
	DragBlades.motorDo(CLOCKWISE, 0);
	eatYourBreakfast();
	delay(200);
	while ((Serial.available() == 0) && !motorGoTo(210)) {
		delay(MOTORTEST_DELAY_MS);
	}
	DragBlades.motorDo(CLOCKWISE, 0);
	eatYourBreakfast();
	delay(300);
	while ((Serial.available() == 0) && !motorGoTo(140)) {
		delay(MOTORTEST_DELAY_MS);
	}
	DragBlades.motorDo(CLOCKWISE, 0);
	eatYourBreakfast();
	delay(300);
	while ((Serial.available() == 0) && !motorGoTo(70)) {
		delay(MOTORTEST_DELAY_MS);
	}
	DragBlades.motorDo(CLOCKWISE, 0);
	eatYourBreakfast();
	delay(300);
	while ((Serial.available() == 0) && !motorGoTo(0)) {
		delay(MOTORTEST_DELAY_MS);
	}
	DragBlades.motorDo(CLOCKWISE, 0);
}

int myAbs(int x) {
	if (x >= 0) {
		return x;
	}
	else {
		return -x;
	}
}

bool motorGoTo(int goTo) {
	static uint8_t count = 0;
	encPosCmd = goTo;
	motorPID.Compute();
	if (mtrSpdCmd >= 0) {
		DragBlades.motorDo(CLOCKWISE, mtrSpdCmd);
	}
	else if (mtrSpdCmd < 0) {
		DragBlades.motorDo(COUNTERCLOCKWISE, -1 * mtrSpdCmd);
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

void systemCheck(bool bnoToo) {
	Serial.println("\n\n----- System Check -----;");
	eatYourBreakfast();                                       //Flushes serial port
	DataLog.init();
	DAQ.init(bnoToo);
	
	Serial.print("Encoder Position: ");
	Serial.println(DragBlades.encPos);
	Serial.println();
	Serial.println("SPP Characteristics-----------");
	Serial.print("Target altitude = ");
	Serial.println(TARGET_ALTITUDE);
	Serial.print("Dry mass = ");
	Serial.println(DRY_MASS);
	Serial.print("Propellant mass = ");
	Serial.println(PROP_MASS);
	Serial.print("'Inter Vel' = ");
	Serial.println(INTER_VEL);
	Serial.print("'Inter Alt' = ");
	Serial.println(INTER_ALT);
	Serial.println();
}