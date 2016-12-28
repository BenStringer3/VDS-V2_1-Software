#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <math.h>

#include "hashTagDefines.h"                                      //All the VDS settings and constants are here
#include "RCR_Bmp180.h"                                         //Our own version of the pressure sensor library
#include "MatrixMath.h"
#include <SdFat.h>
#include <SPI.h>

struct stateStruct {
  float alt;                                                    //The most recent altitude reading from Adafruit BMP180 sensor           (m)
  float vel;                                                    //The most recent velocity derived from calculateVelocity() function     (m/s)
  float accel;                                                  //The most recent acceleration reading from Adafruit BNO055 sensor       (m/s^2)
  unsigned long time;                                           //Time since the program began                                           (us)
  float buff_t;                                                 //The time relative to the present moment. (used in calculateVelocity()) (s)
};

/********************BEGIN GLOBAL VARIABLES********************/
/*General Variables*/
struct stateStruct pastRawStates[BUFF_N];                       //Stores past BUFF_N state structures
unsigned long timer = 0;                  
unsigned int stopWatch = 0;
bool timeOverflow = false;

/*BMP180 Variables*/
long padAlt;                                                    //The sea level (SL) altitude of the launchpad. (mm)
bool bmp180_init = false;                                       //used to inform user that the bmp180 was not initialized succesfully

/*BNO055 Variables*/
bool bno055_init = false;                                       //used to inform user that the bno055 was not initialized succesfully

/*GUI Variables*/
char response;                                                  //Holds the most recent char response from Serial

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

//tests for the kalman filter...
struct stateStruct filteredState_test;                          //Structures used in kalman filter tests
struct stateStruct z_k_1;
struct stateStruct z_k_2;
struct stateStruct z_k_3;

/********************CREATE BMP180 OBJECTS********************/
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);   //stores BMP180 object
/*********************END BMP180 OBJECTS*********************/

/********************CREATE BMP180 OBJECTS********************/
Adafruit_BNO055 bno = Adafruit_BNO055();                        //stores BNO055 object
/*********************END BMP180 OBJECTS*********************/

/********************CREATE FILE IO OBJECTS********************/
File data;                                                      //Stores file object
SdFatSdio sd;                                                   //Micro SD card object
/**********************END FILE IO OBJECTS*********************/

/********************BEGIN FUNCTION PROTOTYPES********************/
/*General Functions*/
void systemCheck(void);                                         //Menu Function.  Checks the state of all objects including: BNO055, BMP180, and microSD card.
void newFlight(void);                                           //Initiates files and variables for a new flight.
void initializePastStates(void);                                //Initializes the pastRawStates array to states with 0 values.
void flightMode(void);                                          //Begins flightMode sequence.  Dependent on TESTMODE.
void getRawState(struct stateStruct* rawState);                 //Retrieves data from sensors.
float calulateVelocity(struct stateStruct);                     //Calculates velocity using alt from bmp180 and accel from BNO055.

/*GUI Functions*/
void printMenu(void);                                           //*HIDDEN* Menu Function.  Prints menu options.
void handShake(void);                                           //Initiates pairing with Java program.
void returnResponse(char);                                      //Returns received response from Java program with message stating what was received.
void copyState(struct stateStruct* original, struct stateStruct* destination);  //Deep copies one state to another.
void printPastStates(struct stateStruct*);                      //Prints all pastRawState values.
void printState(struct stateStruct, int);                       //Prints one state and it's location in the pastRawStates array.
void printTitle(void);                                          //Prints out the title sequence.
void eatYourBreakfast(void);                                    //Clears the serial buffer.. This is helpful for carriage returns and things of that sort that
                                                                //hang around after you got what you wanted.

/*BMP180 Functions*/
void testBMP(void);                                             //Menu Function.  Displays altitude values from BMP180.
float altitude_plz(void);                                       //Checks if Bmp180 has a reading ready, retrieves reading and requests a new readings
                                                                //if yes, returns false if not ready yet.

/*BNO055 Functions*/
void calibrateBNO(void);                                        //Menu Function.  Enters program into a calibration mode, requiring the BNO's acceleration calibration
                                                                //value to reach 3 before exiting.
void testAccelerometer(void);                                   //Menu Function.  Displays different sensor values from the BNO055 as well as the calculated vertical acceleration.
float getAcceleration(void);                                    //Returns the vertical acceleration as a floating point value.
float getAcceleration(imu::Vector<3> gravity, imu::Vector<3> linear);//Returns the vertical acceleration as a floating point value. (test)
void testCalibration(void);                                     //Checks if accelerometer is calibrated, logs error if not.

/*Kalman Functions*/
void kalman(int16_t, struct stateStruct, struct stateStruct*);  //Filters the state of the vehicle.

/*File IO Functions*/
void storeInfo(float);                                          //Stores one data point, followed by a comma, to VDSv2FlightData.dat.
void storeStructs(struct stateStruct, struct stateStruct);      //Stores all information from both structs to VDSv2FlightData.dat and ends the line.
void readFromFile(struct stateStruct* destination);             //Retrieves past flight data for tests.  Replaces sensor functions.
void resetNumber(char*);                                        //Resets (char)number array to NULL values.
float charToFloat(char);                                        //Converts a char number to a floating point value.
float numToFloat(char*);                                        //Converts a char array representing a number into a floating point value.
                                                                //Handles certain forms of scientific notation.
void logError(String);                                          //Stores error to VDSv2Errors.dat.
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
  printTitle();
  
  //Confirm connection with Java program
  //handShake();                                                // send a byte to establish contact until receiver responds

  //Initialize BNO055, BMP180, and microSD card
  systemCheck();
      
#if TEST_MODE
  Serial.println("TEST_MODE!;");
#endif

  printMenu();
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
      systemCheck();
      break;
    case 'C':
      Serial.println("\n\n----- Calibrate BNO055 -----;");
      eatYourBreakfast();                                       //Flushes serial port
      calibrateBNO();
      break;
    case 'A':
      Serial.println("\n\n----- Testing Accelerometer -----;");
      eatYourBreakfast();                                       //Flushes serial port
      testAccelerometer();
      break;
    case 'M':                                                   //Case for printing option menu
      eatYourBreakfast();                                       //Flushes serial port
      break;
    case 'B':
      Serial.println("\n\n----- Testing Barometric Pressure Sensor -----;");
      eatYourBreakfast();                                       //Flushes serial port
      testBMP();
      break;
    case 'K':
      Serial.println("\n\n----- Testing Kalman Filter -----;");
      eatYourBreakfast();                                       //Flushes serial port
      quick_kalman_test();
      break;
    case 'F':
      Serial.println("\n\n----- Entering flight mode -----;");
      eatYourBreakfast();                                       //Flushes serial port

      newFlight();
      
      if ((!bmp180_init || !bno055_init) && !TEST_MODE) {       //If sensors are not initialized, send error, do nothing
        Serial.println("Cannot enter flight mode. A sensor is not initialized.;");
        logError(SENSOR_UNIT);
      } else {
        Serial.println("Entering Flight Mode;");                //If sensors are initialized, begin flight mode
        
        #if !TEST_MODE                                          //If not in test mode, zero the pad altitude
          padAlt = altitude_plz();
          delay(30);
          padAlt = altitude_plz();
          Serial.print("Launch pad altitude = ");
          Serial.println(padAlt);
        #endif
        
        delay(2000);                                            //pause for dramatic effect....
        flightMode();                                           //Initiate Flight Mode
      }
      break;
    default:
      Serial.println("Unkown code received;");
      Serial.println(response);
      logError(INVALID_MENU);
      break;
    }
    printMenu();
  }
} // END loop()
/*********************END LOOP FUNCTION*********************/


/********************BEGIN FUNCTION DEFINITIONS********************/
/*______ _ _       _     _     __  __           _        ______                _   _                 
 |  ____| (_)     | |   | |   |  \/  |         | |      |  ____|              | | (_)                
 | |__  | |_  __ _| |__ | |_  | \  / | ___   __| | ___  | |__ _   _ _ __   ___| |_ _  ___  _ __  ___ 
 |  __| | | |/ _` | '_ \| __| | |\/| |/ _ \ / _` |/ _ \ |  __| | | | '_ \ / __| __| |/ _ \| '_ \/ __|
 | |    | | | (_| | | | | |_  | |  | | (_) | (_| |  __/ | |  | |_| | | | | (__| |_| | (_) | | | \__ \
 |_|    |_|_|\__, |_| |_|\__| |_|  |_|\___/ \__,_|\___| |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/
              __/ |                                                                                  
             |___/ */
/**************************************************************************/
/*!
@brief  Menu Function.  Checks the state of all objects including: BNO055, BMP180, and microSD card.
Author: Jacob
*/
  /**************************************************************************/
void systemCheck(void){
  uint8_t system, gyro, accel, mag = 0;

  /********************INITIALIZE OR TEST BMP180********************/
  if (!bmp.begin()) {                                           //Determine if BMP180 is initialized and ready to be used
    Serial.println("NO Bmp180 DETECTED!");
  } else {
    bmp180_init = true;
    Serial.println("Bmp180 Initialized");
  }
  /********************END TESTING OF BMP180********************/

  /********************INITIALIZE OR TEST BNO055********************/
  if (!bno.begin()) {                                           //Determine if BNO055 is initialized and ready to be used
    Serial.println("NO Bno055 DETECTED!");
  } else {
    bno055_init = true;
    bno.setExtCrystalUse(true);
    Serial.println("Bno055 Initialized");

    bno.getCalibration(&system, &gyro, &accel, &mag);                             //Retrieves calibration values from sensor.
    Serial.print("CALIBRATION: Sys=");                                            //Prints calibration values to serial
    Serial.print(system, DEC);
    Serial.print(" Gyro=");
    Serial.print(gyro, DEC);
    Serial.print(" Accel=");
    Serial.print(accel, DEC);
    Serial.print(" Mag=");
    Serial.print(mag, DEC);
    Serial.println(";");

  }
  /********************END TESTING OF BNO055********************/

  /********************INITIALIZE OR TEST SD CARD********************/
  if(!sd.begin()){                                            //Determine if microSD card is initialized and ready to be used.
    Serial.println("No SD card DETECTED!");
  } else {
    Serial.println("SD card Initialized");                    //If microSD card id ready, begin initialization of flight.  Includes creation of dataFile and it's heading
  }
  /********************END TESTING OF SD CARD********************/
} // END systemCheck()



/**************************************************************************/
/*!
@brief  Prepares varaibles for new launch
Author: Jacob
*/
  /**************************************************************************/
void newFlight(void) {
  sd.remove("VDSv2FlightData.dat");                             //Removes prior flight data file
  sd.remove("VDSv2Errors.dat");                                 //Removes prior error file

  File data = sd.open("VDSv2FlightData.dat", FILE_WRITE);       //Creates new data file
  if(!data){                                                    //If unable to be initiated, throw error statement.  Do nothing
    Serial.println("Data file unable to initiated.;"); 
  } else {                                                      
    #if TEST_MODE                                               //Adds unique header depending on if VDS is in test or flight mode
    data.println("leftVel, rightVel, t(s), alt(m), vel(m/s), accel(m/s^2), kalman altitude(m), kalman velocity(m/s), kalman acceleration(m/s^2)");
    #else
    data.println("xG(m/s^2), yG(m/s^2), zG(m/s^2), xL(m/s^2), yL(m/s^2), zL(m/s^2), leftVel, rightVel, heading, roll, pitch, t(us), alts, vels, accels, alts_k, vels_k, accels_k");
    #endif
    data.close();                                               //Closes data file after use.
  }

  data = sd.open("VDSv2Errors.dat", FILE_WRITE);                //Creates new error file
  if(!data){                                                    //If unable to be initiated, throw error statement.  Do nothing
    Serial.println("Data file unable to initiated.;"); 
  } else {                                                      
    data.println("time(us),error");
    data.close();                                               //Closes data file after use.
  }

  initializePastStates();
} // END newFlight()


/**************************************************************************/
/*!
@brief  Initializes the pastRawStates array to states with 0 values.
Author: Jacob
*/
/**************************************************************************/
void initializePastStates(void){
  for(unsigned int i = 0; i<BUFF_N; i++){
    pastRawStates[i].alt = (float)(0);
    pastRawStates[i].vel = (float)(0);
    pastRawStates[i].accel = (float)(0);
    pastRawStates[i].time = (unsigned long)(0);
  }
} // END initializePastStates()


/**************************************************************************/
/*!
@brief  Launch and test sequence.
Author: Jacob & Ben
*/
/**************************************************************************/
void flightMode(void) {
  struct stateStruct rawState, filteredState;

  while (Serial.available() == 0){

    //get the state, filter it, record it   
    getRawState(&rawState);                                     //Retrieves raw state from sensors and velocity equation.
    kalman(encPos, rawState, &filteredState);                   //feeds raw state into kalman filter and retrieves new filtered state.
    storeStructs(rawState, filteredState);                      //Stores both filtered and raw states into data file.  (Also stores orientation information)

    #if DEBUG_FLIGHTMODE
    printState(rawState, "raw state");                          //If in DEBUG_FLIGHTMODE mode, prints raw state data for evaluation.
    printState(filteredState, "filtered state");                //If in DEBUG_FLIGHTMODE mode, prints filtered state data for evaluation.
    #endif
  }
  //if some serial input ~= to the standdown code or 1 second passes, call flightmode again...  need to discuss
} // END flightMode()


  /**************************************************************************/
  /*!
  @brief  Gathers data from the desired source (Sensors or file).  Dependent on TEST_MODE
  Author: Jacob & Ben
  */
  /**************************************************************************/
void getRawState(struct stateStruct* rawState) {
#if TEST_MODE                                                   //If file is in test mode, retrieve sensor data from data file with past flight data
readFromFile(rawState);                                         //Stores past flight information from data file into rawState struct.
rawState->accel = -1 * (rawState->accel);                       //flip around acceleration, as prior flight data considers upwards acceleration to be negative
#else
  //get raw altitude
  rawState->alt = altitude_plz() - padAlt;                      //Retrieves altitude from bmp180 sensor, stores within rawState

  //get time
  if(timeOverflow){
    rawState->time = millis() * 1000;             //Retrieves time from millis() function, stores within rawState
  } else {
    rawState->time = micros();
  }
  
  if (rawState->time > 4200000000) {
    timeOverflow = true;
  }
  
  //get raw acceleration  
  rawState->accel = getAcceleration();                          //Retrieves acceleration from bno055 sensor, stores within rawState

#endif

  //calculate velocity
  rawState->vel = calculateVelocity(*rawState);                 //Calculates velocity using algorithm.  Takes prior acceleration and velocity values from pastRawStates

#if DEBUG_RAWSTATE
  Serial.println();
  Serial.println("RAW STATE--------------------");
  printState(rawState, "raw state");                            //If in DEBUG_RAWSTATE mode, prints raw state data for evaluation.
#endif
} // END getRawState()


/**************************************************************************/
/*!
@brief  Calculates a velocity value using altitude data from BMP180 and acceleration data fromm BNO055.
Author: Jacob & Ben
- Algorithm developed by Ben Stringer, function written by Jacob Cassady
*/
/**************************************************************************/
float calculateVelocity(struct stateStruct rawState)  { //VARIABLES NEEDED FOR CALULATION
  float sumTimes = 0, sumTimes2 = 0, sumAlt = 0, sumAltTimes = 0, leftSide = 0;
  float rightSide = 0, numer = 0, denom = 0, velocity = 0, pastTime = 0, newTime = 0;

  //shift new readings into arrays   
  for (uint8_t i = BUFF_N; i > 0; i--) {
    copyState(&pastRawStates[i],&pastRawStates[i-1]);           //copyState(1,2) deep copies information from struct 1 into struct 2.
  }
  rawState.buff_t = 0;                              
  copyState(&pastRawStates[0], &rawState);                      //Moves newest state into the 0 position of pastRawStates array.

  //time relative to the current moment
  for (uint8_t i = BUFF_N; i > 0; i--) {
    pastTime = (float)pastRawStates[i-1].time;
    newTime = (float)rawState.time;
    pastRawStates[i - 1].buff_t = (pastTime-newTime)/(float)1000000;   //Calculates buff_t values for pastRawStates array
//    Serial.print("pastRawStates time: ");
//    Serial.print(pastRawStates[i-1].time);
//    Serial.println("");
//    Serial.print("rawState time: ");
//    Serial.print(rawState.time);
//    Serial.println("");
//    delay(500);
  }

  #if DEBUG_VELOCITY && DEBUG_EMERGENCY
    Serial.println("");
    Serial.println("Past states post-shift");
    printPastStates(pastRawStates);                             //If in DEBUG_VELOCITY and DEBUG_EMERGENCY, print all pastRawStates for verification of function output
  #endif

  //FIND SUMS FOR BMP
  for (unsigned int i = 0; i < BUFF_N; i++) {                   //Calculates sums for left side of velocity equation.
    sumTimes += (float)(pastRawStates[i].buff_t) ;
    sumTimes2 += (float)((pastRawStates[i].buff_t) * (pastRawStates[i].buff_t));
    sumAlt += pastRawStates[i].alt;
    sumAltTimes += ((float)pastRawStates[i].buff_t * pastRawStates[i].alt);
  }

  //CALCULATE LEFT SIDE OF EQUATION
  numer = ((sumTimes * sumAlt) - (BUFF_N * sumAltTimes));
  denom = ((sumTimes*sumTimes) - (BUFF_N * sumTimes2));
  leftSide = numer / denom;

  storeInfo(leftSide);                                          //Stores leftSide values for further post-flight analysis.

  #if DEBUG_VELOCITY && DEBUG_EMERGENCY                         //Prints header for future rightSide values if in DEBUG_VELOCITY && DEBUG_EMERGENCY modes.
  Serial.println(" ----- rightSide values ----- ");
  #endif

  //CALCULATE RIGHT SIDE OF EQUATION
  for (unsigned int i = 0; i <= (BUFF_N / 2 ); i++) {
    rightSide += 0.5 * (pastRawStates[i].accel + pastRawStates[i + 1].accel) * (pastRawStates[i].buff_t - pastRawStates[i + 1].buff_t);
    #if DEBUG_VELOCITY //&& DEBUG_EMERGENCY                       //Reports rightSide values if in DEBUG_VELOCITY && DEBUG_EMERGENCY modes, final value is used for final velocity calculation.
    Serial.print(i);
    Serial.print(") rightSide = ");
    Serial.println(rightSide,6);
    #endif
  }

  storeInfo(rightSide);                                         //Stores rightSide values for further post-flight analysis.

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
  Serial.print(rightSide+leftSide, 3);
  Serial.println(";");
#endif

  velocity = (leftSide + rightSide);                            //Calculates final velocity value by summing the left and right sides of the equation
  if isnan(velocity) {                                          //logs error if velocity value is given as nan
    #if DEBUG_VELOCITY
    Serial.println("vel is nan!");
    #endif
    logError(NAN_VEL);
    velocity = 0;                                               //Sets returned velocity to zero to minimize damage from egregious reading.
  }
  if ((velocity > MAX_EXP_VEL) || (velocity < -10)) {           //logs error if velocity value is egregiously too high or low.
    #if DEBUG_VELOCITY
    Serial.print("Velocity non-nominal! = ");
    Serial.println(velocity);
    #endif
    logError(NONNOM_VEL);
    velocity = 0;                                               //Sets returned velocity to zero to minimize damage from egregious reading.
  }
  return velocity;
}// END calculateVelocity()


/**************************************************************************/
/*!
@brief  Deep copies one state to another
Author: Jacob
*/
/**************************************************************************/
void copyState(struct stateStruct* destination, struct stateStruct* original){
  destination->alt = original->alt;
  destination->vel = original->vel;
  destination->accel = original->accel;
  destination->time = original->time;
  destination->buff_t = original->buff_t;
} // END copyState()



 /*____                 __  ___   ___    ______                _   _
 |  _ \               /_ |/ _ \ / _ \  |  ____|              | | (_)
 | |_) |_ __ ___  _ __ | | (_) | | | | | |__ _   _ _ __   ___| |_ _  ___  _ __  ___
 |  _ <| '_ ` _ \| '_ \| |> _ <| | | | |  __| | | | '_ \ / __| __| |/ _ \| '_ \/ __|
 | |_) | | | | | | |_) | | (_) | |_| | | |  | |_| | | | | (__| |_| | (_) | | | \__ \
 |____/|_| |_| |_| .__/|_|\___/ \___/  |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/
                 | |
                 |_| */
/**************************************************************************/
/*!
@brief  Menu Function.  Displays altitude values from BMP180.
Author: Jacob
*/
/**************************************************************************/
void testBMP(void){
  while(Serial.available() <= 0){
    Serial.print("Current altitude: ");
    Serial.printf("%0.3f",altitude_plz());
    Serial.println(";");
  }
}

 /**************************************************************************/
 /*!
 @brief  Checks if Bmp180 has a reading ready, retrieves reading and requests a new readings if yes, returns false if not ready yet
 Pronounced "altitude please".
 Author: Ben
 */
 /**************************************************************************/
float altitude_plz(void) {
  static float returnVal;
  float pressure_kPa;
  float pressure_; //units are Pa*10?

  if (bmp.RCR_readyYet()) {
    bmp.RCR_getPressure(&pressure_kPa);                         //picks up the pressure reading from the Bmp180, then puts in a request for a new one
#if DEBUG_ALPHA
    Serial.print("RCR_getPressure returned: ");
    Serial.print(pressure_kPa);
    Serial.print("  kPa at t = ");
    Serial.println(millis());
#endif
    pressure_ = pressure_kPa / 100.0F;

#if DEBUG_ALPHA
    Serial.print("RCR_getPressure returned: ");
    Serial.print(pressure_kPa);
    Serial.print("  kPa at t = ");
    Serial.println(millis());
    Serial.print("  altitude = ");
    Serial.println(rawState->alt);
#endif
    returnVal = bmp.pressureToAltitude(SENSORS_PRESSURE_SEALEVELHPA, pressure_);
  }
  return returnVal;
} // END altitude_plz()


/*____               ___  _____ _____   ______                _   _
|  _ \             / _ \| ____| ____| |  ____|              | | (_)
| |_) |_ __   ___ | | | | |__ | |__   | |__ _   _ _ __   ___| |_ _  ___  _ __  ___
|  _ <| '_ \ / _ \| | | |___ \|___ \  |  __| | | | '_ \ / __| __| |/ _ \| '_ \/ __|
| |_) | | | | (_) | |_| |___) |___) | | |  | |_| | | | | (__| |_| | (_) | | | \__ \
|____/|_| |_|\___/ \___/|____/|____/  |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/*/
/**************************************************************************/
/*!
@brief  Menu Function.  Enters program into a calibration mode, requiring the BNO's acceleration calibration
        value to reach 3 before exiting.
Author: Jacob
*/
/**************************************************************************/
void calibrateBNO(void) {
  uint8_t system, gyro, accel, mag = 0;
  int calibrationCount = 0;
  
  Serial.println("Calibrating BNO055...;");

  while(calibrationCount < 5){                                                    //Waits until it recieves 5 confirmations that sensor's accelerometer is calibrated.
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

    if(accel > 2){                                                                //If the calibration value for accel is 3 or greater than 2, count as confirmation sensor is calibrated.
      calibrationCount += 1;
    } else {                                                                      //If calibration value is too low, disregard all prior confirmations and restart count.
      calibrationCount = 0;
    }
    
    delay(300);
  }
} // END calibrateBNO()



/**************************************************************************/
/*!
@brief  Menu Function.  Displays different sensor values from the BNO055 as well as the calculated vertical acceleration.
Author: Jacob
*/
/**************************************************************************/
void testAccelerometer(void){
  while(Serial.available() <= 0){
    // Possible vector values can be:
    // - VECTOR_ACCELEROMETER - m/s^2
    // - VECTOR_MAGNETOMETER  - uT
    // - VECTOR_GYROSCOPE     - rad/s
    // - VECTOR_EULER         - degrees
    // - VECTOR_LINEARACCEL   - m/s^2
    // - VECTOR_GRAVITY       - m/s^2
    //imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);        //Creates a vector which stores orientation values.
    imu::Vector<3> linear = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL); //Creates a vector which stores linear acceleration values.
    imu::Vector<3> gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);      //Creates a vector which stores orientation values.

    /* Display the current acceleration from gravity*/
    Serial.print("Initial Gravity <x,y,z>: ");
    Serial.print("<");
    Serial.print(gravity.x());                                //
    Serial.print(",");
    Serial.print(gravity.y());                                //
    Serial.print(",");
    Serial.print(gravity.z());                                //
    Serial.println(">;");


    /* Displays the current linear acceleration values */
    Serial.print("LinearAccel <x,y,z>: ");
    Serial.print("<");
    Serial.print(linear.x());                                //
    Serial.print(",");
    Serial.print(linear.y());                                //
    Serial.print(",");
    Serial.print(linear.z());                                //
    Serial.println(">;");

    getAcceleration(gravity,linear);
    
//        /* Display the orientation data */
//        /*Serial.println("----- Orientation (degrees) -----;");
//        Serial.print("X (Heading): ");
//        Serial.print(euler.x());                                //Heading
//        Serial.print(" Y (Roll): ");
//        Serial.print(euler.y());                                //Roll
//        Serial.print(" Z (Pitch): ");
//        Serial.print(euler.z());                                //Pitch
//        Serial.println(";");*/
//        delay();
  }
}


/**************************************************************************/
/*!
@brief  Returns the vertical acceleration as a floating point value
Author: Jacob
*/
/**************************************************************************/
float getAcceleration(void) {
  imu::Vector<3> gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);        //Creates vector to store acceleration from gravity components
  imu::Vector<3> linear = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);     //Creates vector to store linear acceleration components
  float linearDotGravity = 0, theta = 0, defOfProduct = 0, verticalAcceleration = 0, magL = 0, magG = 0;
  float xG=0, yG=0, zG=0, xL=0, yL=0, zL=0;

  xG = (float)gravity.x();                                                        //Stores most recent x-component of acceleration by gravity
  yG = (float)gravity.y();                                                        //Stores most recent y-component of acceleration by gravity
  zG = (float)gravity.z();                                                        //Stores most recent z-component of acceleration by gravity

  xL = (float)linear.x();                                                         //Stores most recent x-component of linear acceleration
  yL = (float)linear.y();                                                         //Stores most recent y-component of linear acceleration
  zL = (float)linear.z();                                                         //Stores most recent z-component of linear acceleration
  
  linearDotGravity = (xG*xL)+(yG*yL)+(zG*zL);                                     //Calculates dot product of linear acceleration and acceleration from gravity vectors

  magG = pow(((xG*xG)+(yG*yG)+(zG*zG)),0.5);                                       //Calculates magnitude of acceleration from gravity vector.

  verticalAcceleration = linearDotGravity / 9.81;                                 //Finds the acceleration in the direction of gravity.

  storeInfo(xG);                                                                  //logs most recent x-component of acceleration by gravity to dataFile.
  storeInfo(yG);                                                                  //logs most recent y-component of acceleration by gravity to dataFile.
  storeInfo(zG);                                                                  //logs most recent z-component of acceleration by gravity to dataFile.

  storeInfo(xL);                                                                  //logs most recent x-component of linear acceleration to dataFile.
  storeInfo(yL);                                                                  //logs most recent y-component of linear acceleration to dataFile.
  storeInfo(zL);                                                                  //logs most recent z-component of linear acceleration to dataFile.

  testCalibration();

  return verticalAcceleration;                                                    //Returns calculated vertical acceleration.
} // END getAcceleration();


/*OVERLOADED VERSION.  TAKES IN THE TWO VECTORS AND RETURNS THE VERTICAL ACCELERATION :: USED FOR TESTING PURPOSES*/
float getAcceleration(imu::Vector<3> gravity, imu::Vector<3> linear) {
  float linearDotGravity = 0, theta = 0, defOfProduct = 0, magOfVerticalAcceleration = 0, verticalAcceleration = 0, magL = 0, magG = 0;
  float xG=0, yG=0, zG=0, xL=0, yL=0, zL=0;

  xG = (float)gravity.x();                                                        //Stores most recent x-component of acceleration by gravity
  yG = (float)gravity.y();                                                        //Stores most recent y-component of acceleration by gravity
  zG = (float)gravity.z();                                                        //Stores most recent z-component of acceleration by gravity

  xL = (float)linear.x();                                                         //Stores most recent x-component of linear acceleration
  yL = (float)linear.y();                                                         //Stores most recent y-component of linear acceleration
  zL = (float)linear.z();                                                         //Stores most recent z-component of linear acceleration
  
  linearDotGravity = (xG*xL)+(yG*yL)+(zG*zL);                                     //Calculates dot product of linear acceleration and acceleration from gravity vectors

  magL = pow(((xL*xL)+(yL*yL)+(zL*zL)),0.5);                                      //Calculates magnitude of linear acceleration vector.
  magG = pow(((xG*xG)+(yG*yG)+(zG*zG)),0.5);                                      //Calculates magnitude of acceleration from gravity vector.

  defOfProduct = linearDotGravity / (magL*magG);                                  //Calculates the cosine value using the definition of a dot product.

  theta = acos(defOfProduct);                                                     //Calculates theta using the arc cosine of the previously calculated vosine value.
  theta = (theta*180)/PI;                                                         //Converts theta from radians to degress.

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
 @brief  Returns the vertical acceleration as a floating point value
 Author: Jacob with edits by Ben
 */
 /**************************************************************************/
float getAcceleration_2(void) {
  imu::Vector<3> gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  imu::Vector<3> linear = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  float linearDotGravity = 0, theta = 0, defOfProduct = 0, magOfVerticalAcceleration = 0, verticalAcceleration = 0, magL = 0, magG = 0;
  float xG = 0, yG = 0, zG = 0, xL = 0, yL = 0, zL = 0;

  xG = (float)gravity.x();                                                        //Stores most recent x-component of acceleration by gravity
  yG = (float)gravity.y();                                                        //Stores most recent y-component of acceleration by gravity
  zG = (float)gravity.z();                                                        //Stores most recent z-component of acceleration by gravity

  xL = (float)linear.x();                                                         //Stores most recent x-component of linear acceleration
  yL = (float)linear.y();                                                         //Stores most recent y-component of linear acceleration
  zL = (float)linear.z();                                                         //Stores most recent z-component of linear acceleration

#if DEBUG_GETACCELERATION                                                         //if in DEBUG_GETACCELERATION mode, print out all component values
  Serial.println("");
  Serial.println("GETACCELERATION---------------------");
  Serial.print("xG = ");
  Serial.print(xG);
  Serial.print("\t yG = ");
  Serial.print(yG);
  Serial.print("\t zG = ");
  Serial.println(zG);
  Serial.print("xL = ");
  Serial.print(xL);
  Serial.print("\t yL = ");
  Serial.print(yL);
  Serial.print("\t zL = ");
  Serial.println(zL);
#endif

  storeInfo(xG);                                                                  //logs most recent x-component of acceleration by gravity to dataFile.
  storeInfo(yG);                                                                  //logs most recent y-component of acceleration by gravity to dataFile.
  storeInfo(zG);                                                                  //logs most recent z-component of acceleration by gravity to dataFile.

  storeInfo(xL);                                                                  //logs most recent x-component of linear acceleration to dataFile.
  storeInfo(yL);                                                                  //logs most recent y-component of linear acceleration to dataFile.
  storeInfo(zL);                                                                  //logs most recent z-component of linear acceleration to dataFile.

  linearDotGravity = (xG*xL)+(yG*yL)+(zG*zL);                                     //Calculates dot product of linear acceleration and acceleration from gravity vectors

  return (linearDotGravity / 9.81 - 9.81);
} // END get_Acceleration_2()



/**************************************************************************/
/*!
@brief  Checks if accelerometer is calibrated, logs error if not.
Author: Jacob
*/
/**************************************************************************/
void testCalibration(void){
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);                               //Retrieves calibration values from sensor.

  if(accel < 3){                                                                  //If accelerometer is not calibrated, log in errorLog the occurance.
    logError(UNCALIBRATED_BNO);
  }
} // END testCalibration


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
  static float lastTime;
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

  delta_t = rawState.time - lastTime;
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
  c_d = CD_R*(1 - encPos / ENC_RANGE) + CD_B / ENC_RANGE;
  area = A_R*(1 - encPos / ENC_RANGE) + A_B / ENC_RANGE;
  q = RHO * rawState.vel * rawState.vel / 2;
  u_k = -9.81 - c_d * area * q / POST_BURN_MASS;

  // if acceleration > 10m/s^2 the motor is probably burning and we should add that in to u_k
  if (z_k[2] > 10) {
    Serial.println("Burn Phase!"); //errorlog
    u_k += AVG_MOTOR_THRUST / POST_BURN_MASS;
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
    logError(NAN_UK);
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
@brief  Initializes some dummy state variables used to test the Kalman filter
Author: Ben
*/
/**************************************************************************/
void quick_kalman_test(void) {
  z_k_1.accel = 102.8645;
  z_k_1.vel = 6.7814;
  z_k_1.alt = 0.3974;
  z_k_1.time = (unsigned long)(128 * 1000);
  z_k_2.accel = 110.7171;
  z_k_2.vel = 31.5675;
  z_k_2.alt = 4.7064;
  z_k_2.time = (unsigned long)((128 + 226) * 1000);
  z_k_3.accel = 138.1404;
  z_k_3.vel = 61.2562;
  z_k_3.alt = 15.9803;
  z_k_3.time = (unsigned long)((128 + 226 + 245) * 1000);

  kalman(0, z_k_1, &filteredState_test);
  printState(filteredState_test, "test 1");
  kalman(0, z_k_2, &filteredState_test);
  printState(filteredState_test, "test 2");
  kalman(0, z_k_3, &filteredState_test);
  printState(filteredState_test, "test 3");
} // END quick_kalman_test()

/*           ,,    ,,                                                                                     ,,                             
`7MM"""YMM db  `7MM              `7MMF' .g8""8q.       `7MM"""YMM                                mm     db                             
  MM    `7       MM                MM .dP'    `YM.       MM    `7                                MM                                    
  MM   d `7MM    MM  .gP"Ya        MM dM'      `MM       MM   d `7MM  `7MM  `7MMpMMMb.  ,p6"bo mmMMmm `7MM  ,pW"Wq.`7MMpMMMb.  ,pP"Ybd 
  MM""MM   MM    MM ,M'   Yb       MM MM        MM       MM""MM   MM    MM    MM    MM 6M'  OO   MM     MM 6W'   `Wb MM    MM  8I   `" 
  MM   Y   MM    MM 8M""""""       MM MM.      ,MP       MM   Y   MM    MM    MM    MM 8M        MM     MM 8M     M8 MM    MM  `YMMMa. 
  MM       MM    MM YM.    ,       MM `Mb.    ,dP'       MM       MM    MM    MM    MM YM.    ,  MM     MM YA.   ,A9 MM    MM  L.   I8 
.JMML.   .JMML..JMML.`Mbmmd'     .JMML. `"bmmd"'       .JMML.     `Mbod"YML..JMML  JMML.YMbmd'   `Mbmo.JMML.`Ybmd9'.JMML  JMML.M9mmmP' */

/**************************************************************************/
/*!
@brief  Stores one data point to VDSv2FlightData.dat
Author: Jacob
*/
/**************************************************************************/
void storeInfo(float dataPoint){
  File myFile = sd.open("VDSv2FlightData.dat", FILE_WRITE);

  if(myFile) {
    myFile.printf("%0.3f",dataPoint);
    myFile.print(",");
  } else {
    Serial.print("Unable to open VDSv2FlightData.dat;");
    logError(E_FILE_FLIGHT);
  }
  
  myFile.close();
} // END storeInfo()


/**************************************************************************/
/*!
@brief  Stores all information from both structs to VDSv2FlightData.dat and ends the line.
Author: Jacob
*/
/**************************************************************************/
void storeStructs(struct stateStruct sensorData, struct stateStruct kalmanData){
  File myFile = sd.open("VDSv2FlightData.dat", FILE_WRITE);
  #if !TEST_MODE                                                              //If we are using sensors for data acquisition, retrieve orientation values from bno055
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);        //Creates a vector which stores orientation values.
  #endif
  
  if(myFile) {
    #if !TEST_MODE                                                            //If not in TEST_MODE log orientation values into dataFile
    myFile.printf("%0.2f",euler.x());
    myFile.print(",");
    myFile.printf("%0.2f",euler.y());
    myFile.print(",");
    myFile.printf("%0.2f",euler.z());
    myFile.print(",");
    #endif
    myFile.print(sensorData.time);                                   //Regardless of TEST_MODE, stores "raw" data and kalman data into dataFile
    myFile.print(",");
    myFile.printf("%0.3f",sensorData.alt);
    myFile.print(",");
    myFile.printf("%0.4f",sensorData.vel);
    myFile.print(",");
    myFile.printf("%0.3f",sensorData.accel);
    myFile.print(",");
    myFile.printf("%0.3f",kalmanData.alt);
    myFile.print(",");
    myFile.printf("%0.4f",kalmanData.vel);
    myFile.print(",");
    myFile.printf("%0.3f",kalmanData.accel);
    myFile.println("");
  } else {                                                                    //If unable to open dataFile, log occurance within errorFile
    Serial.print("Unable to open VDSv2FlightData.dat;");
    logError(E_FILE_FLIGHT);
  }
    myFile.close();
} // END storeStructs


/**************************************************************************/
/*!
@brief  Retrieves past flight data for tests.  Replaces sensor functions
Author: Jacob
*/
/**************************************************************************/
void readFromFile(struct stateStruct* destination){
  File myFile = sd.open(TEST_FILENAME, FILE_READ);
  char place = '\n';
  char number[20] = {'\0'};
  short numPlace = 0, numCount = 0;
  float value = 0;
  int lineCount = 0;
  static int linePlaceHolder = 0;

  if(myFile){
    while(myFile.available()) {
      place = myFile.read();

      if(isdigit(place)){
        number[numPlace] = place;
      } else if (place == '.') {
        number[numPlace] = place;
      } else if (place == '-') {
        number[numPlace] = place;
      } else if (place == 'e' || place == '+') {
        number[numPlace] = place;
      } else if (place == ','){
        value = numToFloat(number);
        numCount++;
        numPlace = -1;
        switch(numCount-(lineCount*3)){
        case 1:
          destination->time = (value);
          break;
          
        case 2:
          destination->alt = value;
          break;
        }
        resetNumber(number);
      } else {
        value = numToFloat(number);
        numCount++;
        numPlace = -1;
        resetNumber(number);
        lineCount++;
        destination->accel = (value*-1);
      }

      if(numCount == 0) {
        
      } else {
        if((numCount % ((linePlaceHolder+1)*3)) == 0){
#if DEBUG_READFROMFILE
      Serial.println("");
      Serial.print("Time: ");
      Serial.print(destination->time);
        Serial.print(";");
      Serial.print("Altitude: ");
      Serial.print(destination->alt);
      Serial.print(";");
      Serial.print("Acceleration: ");
      Serial.print(destination->accel);
      Serial.print(";");
#endif
          linePlaceHolder++;
          break;
        }
      }
      numPlace++;
      
    }

    myFile.close();
  } else {
    Serial.print("error opening the text file within readFromFile()!;");
    logError(E_FILE_TEST);
  }
} //END readFromFile();


/**************************************************************************/
/*!
@brief  Resets (char)number array to NULL values.
Author: Jacob
*/
/**************************************************************************/
void resetNumber(char* number){
  for(short i = 0; i<20; i++){
    number[i] = '\0';
  }
} //END resetNumber();


/**************************************************************************/
/*!
@brief  Converts a char number to a floating point value
Author: Jacob
*/
/**************************************************************************/
float charToFloat(char input){
  int temp = input - '\0';
  temp -= 48;
  return float(temp);
} //END charToFloat();

/**************************************************************************/
/*!
@brief  Converts a char array representing a number into a floating point value.
        Handles certain forms of scientific notation.
Author: Jacob
*/
/**************************************************************************/
float numToFloat(char* number){
  short index = 0, decimalIndex = 0;
  boolean decimal = false, e = false, negative = false, one = false;
  float result = 0, temp = 0;

  while(number[index] != '\0'){
    if(number[index] == 'e'){
      e = true;
    } else if (number[index] == '.'){
      decimal = true;
    } else if (number[index] == '-'){
      negative = true;
    } else if (number[index] == '+'){
      ;
    } else {
      temp = charToFloat(number[index]);
      if(e){
        result *= (float)pow(10,temp);
      } else if (decimal) {
        temp *= pow(10,(-1*(decimalIndex+1)));
        result += temp;
        decimalIndex++;
      } else {
        if(one){
          result *= 10;
        } else {
           one = true;
        }
        result += temp;
      }
    }
    index++;
  }
  if(negative){
      result *= -1;
  }
  return result;
} //END numToFloat();


/**************************************************************************/
/*!
@brief  Stores error to VDSv2Errors.dat.
Author: Jacob
*/
/**************************************************************************/
void logError(String error){
  File myFile = sd.open("VDSv2Errors.dat", FILE_WRITE);
  float time = (float)millis() / (float)1000;
  
  if(myFile) {
    myFile.printf("%0.6f",time);
    myFile.print(",");
    myFile.print(error);
    myFile.println("");
  } else {
    Serial.print("Unable to open VDSv2Errors.dat;");
  }
    myFile.close();
} // END logError()


/*/$$$$$$  /$$   /$$ /$$$$$$       /$$$$$$$$                              /$$     /$$
/ $$__  $$| $$  | $$|_  $$_/      | $$_____/                             | $$    |__/
| $$  \__/| $$  | $$  | $$        | $$    /$$   /$$ /$$$$$$$   /$$$$$$$ /$$$$$$   /$$  /$$$$$$  /$$$$$$$   /$$$$$$$
| $$ /$$$$| $$  | $$  | $$        | $$$$$| $$  | $$| $$__  $$ /$$_____/|_  $$_/  | $$ /$$__  $$| $$__  $$ /$$_____/
| $$|_  $$| $$  | $$  | $$        | $$__/| $$  | $$| $$  \ $$| $$        | $$    | $$| $$  \ $$| $$  \ $$|  $$$$$$
| $$  \ $$| $$  | $$  | $$        | $$   | $$  | $$| $$  | $$| $$        | $$ /$$| $$| $$  | $$| $$  | $$ \____  $$
|  $$$$$$/|  $$$$$$/ /$$$$$$      | $$   |  $$$$$$/| $$  | $$|  $$$$$$$  |  $$$$/| $$|  $$$$$$/| $$  | $$ /$$$$$$$/
\______/  \______/ |______/      |__/    \______/ |__/  |__/ \_______/   \___/  |__/ \______/ |__/  |__/|_______/ */
/**************************************************************************/
/*!
@brief  *HIDDEN* Menu Function.  Prints menu options.
Author: Jacob
*/
/**************************************************************************/
void printMenu(void){
  Serial.println("\n\n--------- Menu -----------;");
  Serial.println("'S' - System Check;");
  Serial.println("'C' - Calibrate BNO055;");
  Serial.println("'A' - Accelerometer Test;");
  Serial.println("'B' - Barometric Pressure Sensor Test;");
  Serial.println("'K' - Kalman Filter Test;");
  Serial.println("'F' - Flight Mode;");
} // END printMenu()


/**************************************************************************/
/*!
@brief  Initializes and confirms connection with Java program.
Author: Jacob
*/
/**************************************************************************/
void handShake() {
  while (Serial.available() <= 0) {
    Serial.write('~');   // send a ~ until a response is received.
    delay(300);
  }
} //END handShake()


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
@brief  Returns a received response from the Java program to ensure successful delivery
Author: Jacob
*/
/**************************************************************************/
void returnResponse(char response) {
  if (response == '~') {
    ;
  }
  else {
    Serial.print(response);
    Serial.print(" RECEIVED;");
    Serial.flush();
  }
} //END returnResponse()


/**************************************************************************/
/*!
@brief  Prints one state and it's location in the pastRawStates array
Author: Jacob
*/
/**************************************************************************/
void printState(struct stateStruct state, int label) {
  Serial.print(label);
  Serial.print(") alt = ");
  Serial.print(state.alt, 4);
  Serial.print(", vel = ");
  Serial.print(state.vel, 4);
  Serial.print(", accel = ");
  Serial.print(state.accel, 4);
  Serial.print(", time = ");
  Serial.print(state.time);
  Serial.print(", buff_t = ");
  Serial.print(state.buff_t, 4);
  Serial.println(");");
} //End printState()


  /**************************************************************************/
  /*!
  @brief  prints the state struct
  Author: Ben
  */
  /**************************************************************************/
void printState(struct stateStruct state, String label) {
  Serial.println();
  Serial.println(label);
  Serial.print("alt =   ");
  Serial.println(state.alt, 3);
  Serial.print("vel =   ");
  Serial.println(state.vel, 4);
  Serial.print("accel = ");
  Serial.println(state.accel, 3);
  Serial.print("t =     ");
  Serial.println(state.time, 6);
} // END printState()


/**************************************************************************/
/*!
@brief  Prints all pastRawState values.
Author: Jacob
*/
/**************************************************************************/
void printPastStates(struct stateStruct* pastStates) {
  Serial.println("");
  for (int i = 0; i < BUFF_N; i++) {
    printState(pastStates[i], i);
  }
} // END printPastStates()


/**************************************************************************/
/*!
@brief  Prints out the title sequence
Author: Ben
*/
/**************************************************************************/
void printTitle(void) {
  //remember backslahses have to be double backslashed to print correctly
  Serial.println("             __      _______   _____  __      _____ ");
  Serial.println("             \\ \\    / /  __ \\ / ____| \\ \\    / /__ \\ ");
  Serial.println("              \\ \\  / /| |  | | (___    \\ \\  / /   ) | ");
  Serial.println("               \\ \\/ / | |  | |\\___ \\    \\ \\/ /   / / ");
  Serial.println("                \\  /  | |__| |____) |    \\  /   / /_ ");
  Serial.println("                 \\/   |_____/|_____/      \\/   |____| ");
  Serial.println("");
  Serial.println("             River City Rocketry's Variable Drag System");
  Serial.println(" \t\t December 2016 Sensor/Filter Tests");
  Serial.println("");
  Serial.println("Software written by Jacob Cassady, Ben Stringer, Lydia Sharp, and Denny Joy.");
  Serial.println("With help from libraries written by Adafruit Industries.");
  Serial.println("Mechanical hardware developed by Justin Johnson.");
  Serial.println("Electrical hardware developed by Kenny Dang and Alora Mazarakis.");
  Serial.println("");
} // END printTitle()


/**************************************************************************/
/*!
@brief  Just a place for quick tests
Author: Ben
*/
/**************************************************************************/
void testNAN(void){
  float a, b, c;
  a = 0;
  b = 0;
  c = a * b;
  Serial.println("testNAN");
  Serial.println(c);
    if (isnan(c)) {
      Serial.println("c is nan");
    }
} // END testNAN(void)

void test(void) {
  float a = 0.3456;
  int b;
  b = 1000 * a;
  Serial.println(a,5);
  while (Serial.available() == 0) {
    a = (float)micros() / 1000000;
    Serial.println(a, 6);
    delay(5);
  }
} // END test(void)
/*********************END FUNCTION DEFINITIONS*********************/
