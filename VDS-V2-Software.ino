//#include "GUI.h"
//#include "DataLog.h"
#include <math.h>
#include "hashTagDefines.h"                                      //All the VDS settings and constants are here
#include "MatrixMath.h"
#include <SdFat.h>
#include <SPI.h>
#include "RCRClasses.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Wire.h>


/********************BEGIN GLOBAL VARIABLES********************/
/*General Variables*/
unsigned long timer = 0;                  
unsigned int stopWatch = 0;
char response;

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

//encoder
volatile uint8_t encPos = 0;                                    //Stores most recent position of encoder

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
  DataLog.init();
  DAQ.init();
  
#if TEST_MODE
  Serial.println("TEST_MODE!;");
#endif

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
void loop(void) {
  if (Serial.available() > 0) {
    switch (Serial.read()) {
    case 'S':
      Serial.println("\n\n----- System Check -----;");
      eatYourBreakfast();                                       //Flushes serial port
	  DAQ.init();
	  DataLog.init();
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
    case 'M':                                                   //Case for printing option menu
      eatYourBreakfast();                                       //Flushes serial port
      break;
    case 'B':
      Serial.println("\n\n----- Testing Barometric Pressure Sensor -----;");
      eatYourBreakfast();                                       //Flushes serial port
      DAQ.testBMP();
      break;
    case 'F':
      Serial.println("\n\n----- Entering flight mode -----;");
      eatYourBreakfast();                                       //Flushes serial port

      DataLog.newFlight();
      
      if ((!DAQ.bmp180_init || !DAQ.bno055_init) && !TEST_MODE) {       //If sensors are not initialized, send error, do nothing
        Serial.println("Cannot enter flight mode. A sensor is not initialized.;");
		DataLog.logError(SENSOR_UNIT);
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
	  DataLog.logError(INVALID_MENU);
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
	while (Serial.available() == 0) {

		//get the state, filter it, record it   
		DAQ.getRawState(&rawState);                                     //Retrieves raw state from sensors and velocity equation.
		kalman(0, rawState, &filteredState);                   //feeds raw state into kalman filter and retrieves new filtered state.
		DAQ.getAdditionalData(rawState, filteredState);
		DataLog.logData();

#if DEBUG_FLIGHTMODE
		Serial.println("");
		Serial.println("FLIGHTMODE-------------------");
		GUI.printState(rawState, "raw state");                          //If in DEBUG_FLIGHTMODE mode, prints raw state data for evaluation.
		Serial.println("");
		GUI.printState(filteredState, "filtered state");                //If in DEBUG_FLIGHTMODE mode, prints filtered state data for evaluation.
#endif
	}
	//if some serial input ~= to the standdown code or 1 second passes, call flightmode again...  need to discuss
} // END flightMode()



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
  u_k = -9.81 - c_d * area * q / POST_BURN_MASS;

  // if acceleration > 10m/s^2 the motor is probably burning and we should add that in to u_k
  if (z_k[2] > 10) {
    //Serial.println("Burn Phase!"); //errorlog
    u_k += AVG_MOTOR_THRUST / (POST_BURN_MASS + PROP_MASS/2);
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


