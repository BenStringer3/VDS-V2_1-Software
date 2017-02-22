// DAQ.h
#include "Adafruit_BMP280.h"
#include "GlobVars.h" 
#include "RCR_Bmp180.h"                                         //Our own version of the pressure sensor library
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Wire.h>
#include "SdFat.h"
#include "SPI.h"
#include "RCRPID.h"
#include <eeprom.h>


/*
 _____          ____     _____ _
|  __ \   /\   / __ \   / ____| |
| |  | | /  \ | |  | | | |    | | __ _ ___ ___
| |  | |/ /\ \| |  | | | |    | |/ _` / __/ __|
| |__| / ____ \ |__| | | |____| | (_| \__ \__ \
|_____/_/    \_\___\_\  \_____|_|\__,_|___/___/
*/

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
	unsigned long time;                                           //Time since the program began                                           (sec*10^-5)
	float buff_t;                                                 //The time relative to the present moment. (used in calculateVelocity()) (s)
};

class DAQClass
{
protected:
	Adafruit_BNO055 bno = Adafruit_BNO055();                        //stores BNO055 object
#if !BMP280
	Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);   //stores BMP180 object
#else
	Adafruit_BMP280 bme;// = Adafruit_BMP280();
#endif
	float padAlt;
	bool timeOverflow = false;
	float lastAlt;
	float altitude_plz(void);
	float calculateVelocity(struct stateStruct rawState);           //Calculates velocity using alt from bmp180 and accel from BNO055.
	float getAcceleration(void);										//Returns the vertical acceleration as a floating point value.
	float getAcceleration(imu::Vector<3> gravity, imu::Vector<3> linear);//Returns the vertical acceleration as a floating point value. (test)
	void testCalibration(void);									//Checks if accelerometer is calibrated, logs error if not.
	void copyState(struct stateStruct* destination, struct stateStruct* original);//Deep copies one state to another.


	stateStruct pastRawStates[BUFF_N];                       //Stores past BUFF_N state structures

public:
	void init(bool bnoToo);
	void setPadAlt(void);
	bool getRawState(struct stateStruct* rawState, bool testMode);                 //Retrieves data from sensors.
	void getAdditionalData(stateStruct rawState, stateStruct filteredState, bool testMode);
	void testBMP(void);											//Menu Function.  Displays altitude values from BMP180.
	void testAccelerometer(void);									 //Menu Function.  Displays different sensor values from the BNO055 as well as the calculated vertical acceleration.
	void calibrateBNO(void);											//Menu Function.  Enters program into a calibration mode, requiring the BNO's acceleration calibration
																		//value to reach 3 before exiting.
};

extern DAQClass DAQ;

#endif

/*
 _____        _        _                    _____ _
|  __ \      | |      | |                  / ____| |
| |  | | __ _| |_ __ _| |     ___   __ _  | |    | | __ _ ___ ___
| |  | |/ _` | __/ _` | |    / _ \ / _` | | |    | |/ _` / __/ __|
| |__| | (_| | || (_| | |___| (_) | (_| | | |____| | (_| \__ \__ \
|_____/ \__,_|\__\__,_|______\___/ \__, |  \_____|_|\__,_|___/___/
									__/ |
									|___/
*/

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
	float vSPP;
	int16_t encPos;
	int16_t encPosCmd;
	bool limit_out;
	bool limit_in;
	int16_t encMax;
	int16_t encMin;
	int mtrSpdCmd;
};

class DataLogClass
{
protected:
	File data;                                                      //Stores file object
	int pos = 0;
	uint32_t testFileSize;
public:
	void printTestFileNames();
	void init();
	SdFatSdio sd;                                                   //Micro SD card object
	void logData(bool testMode);
	stateToLogStruct supStat;
	void logError(String error);				 //Stores error to VDSv2Errors.dat.
	void newFlight(bool flightMode);						//Initiates files and variables for a new flight.
	bool readCSV(struct stateStruct* destination);
};

extern DataLogClass DataLog;

#endif

/*
  _____ _    _ _____    _____ _
 / ____| |  | |_   _|  / ____| |
| |  __| |  | | | |   | |    | | __ _ ___ ___
| | |_ | |  | | | |   | |    | |/ _` / __/ __|
| |__| | |__| |_| |_  | |____| | (_| \__ \__ \
 \_____|\____/|_____|  \_____|_|\__,_|___/___/
*/

#ifndef _GUI_h
#define _GUI_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif



class GUIClass
{
protected:
	void editRocket();
	float readFloat(int address);
	void writeFloat(float value, int address);
	String readString(int address);
	uint8_t readUint8_t(int address);
	void writeUint8_t(uint8_t value, int address);
	void writeString(String value, int address);
	bool loadRocket(uint8_t whichOne);
	void saveRocket(uint8_t whichOne);
	void printRocket();
	void printRocketMenu();
	uint8_t currentRocket = 1;

public:
	void init();
	void rocketMenu();
	//struct rocketStruct rocket;
	void printPastStates(struct stateStruct*);                      //Prints all pastRawState values.
	void printState(struct stateStruct, int);                       //Prints one state and it's location in the pastRawStates array.
	void printState(struct stateStruct, String);                       //Prints one state and it's location in the pastRawStates array.
	void printTitle(void);                                          //Prints out the title sequence.
	void printMenu(void);
	void eatYourBreakfast();
};

extern GUIClass GUI;

#endif


/*
 _____                  ____  _           _              _____ _
|  __ \                |  _ \| |         | |            / ____| |
| |  | |_ __ __ _  __ _| |_) | | __ _  __| | ___  ___  | |    | | __ _ ___ ___
| |  | | '__/ _` |/ _` |  _ <| |/ _` |/ _` |/ _ \/ __| | |    | |/ _` / __/ __|
| |__| | | | (_| | (_| | |_) | | (_| | (_| |  __/\__ \ | |____| | (_| \__ \__ \
|_____/|_|  \__,_|\__, |____/|_|\__,_|\__,_|\___||___/  \_____|_|\__,_|___/___/
				   __/ |
				   |___/
*/


#ifndef _DRAGBLADES_h
#define _DRAGBLADES_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

class DragBladesClass
{
	RCRPID motorPID;
protected:
	int mtrSpdCmd = 0;											//motor speed command	
	int myAbs(int x);
	void motorGoToPersistent(uint16_t goTo);
public:
	int encMin = 0;
	int encMax = ENC_RANGE;
	void dragBladesCheck();
	void powerTest();
	DragBladesClass() : motorPID(&encPos, &mtrSpdCmd, &encPosCmd, KP, KI, KD, KN, -255, 255) {}
	volatile int encPos;
	int airBrakesGoToEncPos(float vehVel, float sppVel);
	int encPosCmd = 0;											//encoder position command
	void init(void);
	void motorDo(bool direction, uint8_t speed);
	void motorDont();
	bool motorGoTo(int16_t encCmd);
	void motorTest();
	void motorExercise();

};


extern DragBladesClass DragBlades;


#endif

