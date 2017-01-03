// DAQ.h
#include "hashTagDefines.h" 
#include "RCR_Bmp180.h"                                         //Our own version of the pressure sensor library
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Wire.h>
#include "SdFat.h"
#include "SPI.h"

#ifndef _DAQ_h
#define _DAQ_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

struct stateStruct {
	float alt;                                                    //The most recent altitude reading from Adafruit BMP180 sensor           (m)
	float vel;                                                    //The most recent velocity derived from calculateVelocity() function     (m/s)
	float accel;                                                  //The most recent acceleration reading from Adafruit BNO055 sensor       (m/s^2)
	unsigned long time;                                           //Time since the program began                                           (us)
	float buff_t;                                                 //The time relative to the present moment. (used in calculateVelocity()) (s)
};

class DAQClass
{
 protected:
	 float padAlt;
	 Adafruit_BNO055 bno;// = Adafruit_BNO055();                        //stores BNO055 object
	 Adafruit_BMP085_Unified bmp;// = Adafruit_BMP085_Unified(10085);   //stores BMP180 object
	 float altitude_plz(void);
	 float calculateVelocity(struct stateStruct rawState);           //Calculates velocity using alt from bmp180 and accel from BNO055.
	 bool timeOverflow = false;
	 float getAcceleration(void);										//Returns the vertical acceleration as a floating point value.
	 float getAcceleration(imu::Vector<3> gravity, imu::Vector<3> linear);//Returns the vertical acceleration as a floating point value. (test)
	 void testCalibration(void);									//Checks if accelerometer is calibrated, logs error if not.
	 void copyState(struct stateStruct* destination, struct stateStruct* original);//Deep copies one state to another.
	 float lastAlt; 
	 stateStruct pastRawStates[BUFF_N];                       //Stores past BUFF_N state structures

 public:
	void init();
	void setPadAlt(void);
	void getRawState(struct stateStruct* rawState);                 //Retrieves data from sensors.
	bool bmp180_init = false;                                       //used to inform user that the bmp180 was not initialized succesfully
	bool bno055_init = false;                                       //used to inform user that the bno055 was not initialized succesfully
	void getAdditionalData(stateStruct rawState, stateStruct filteredState);
	void testBMP(void);											//Menu Function.  Displays altitude values from BMP180.
	void testAccelerometer(void);									 //Menu Function.  Displays different sensor values from the BNO055 as well as the calculated vertical acceleration.
	void calibrateBNO(void);											//Menu Function.  Enters program into a calibration mode, requiring the BNO's acceleration calibration
																		//value to reach 3 before exiting.
	//friend void derp(void);

	//friend class DataLogClass;
};

extern DAQClass DAQ;

#endif



#ifndef _DATALOG_h
#define _DATALOG_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

struct stateToLogStruct {
	unsigned long time;
	float alt;
	float vel;
	float leftVel;
	float rightVel;
	float accel;
	float rollAxisGrav;
	float yawAxisGrav;
	float pitchAxisGrav;
	float rollAxisLin;
	float yawAxisLin;
	float pitchAxisLin;
	float rollAxisGyro;
	float yawAxisGyro;
	float pitchAxisGyro;
	float roll;
	float yaw;
	float pitch;
	float alt_k;
	float vel_k;
	float accel_k;
};

class DataLogClass
{
protected:
	File data;                                                      //Stores file object
	SdFatSdio sd;                                                   //Micro SD card object

public:
	void init();
	void logData(void);
	stateToLogStruct supStat;
	void logError(String error);				 //Stores error to VDSv2Errors.dat.
	void newFlight(void);						//Initiates files and variables for a new flight.
};

extern DataLogClass DataLog;

#endif


#ifndef _GUI_h
#define _GUI_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

class GUIClass
{


public:
	void init();
	void printPastStates(struct stateStruct*);                      //Prints all pastRawState values.
	void printState(struct stateStruct, int);                       //Prints one state and it's location in the pastRawStates array.
	void printState(struct stateStruct, String);                       //Prints one state and it's location in the pastRawStates array.
	void printTitle(void);                                          //Prints out the title sequence.
};

extern GUIClass GUI;

#endif