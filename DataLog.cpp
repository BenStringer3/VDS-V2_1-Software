// 
// 
// 

//#include "DataLog.h"
#include "RCRClasses.h"

void DataLogClass::init()
{
	/********************INITIALIZE OR TEST SD CARD********************/
	if (!sd.begin()) {                                            //Determine if microSD card is initialized and ready to be used.
		Serial.println("No SD card DETECTED!");
	}
	else {
		Serial.println("SD card Initialized");                    //If microSD card id ready, begin initialization of flight.  Includes creation of dataFile and it's heading
	}
	/********************END TESTING OF SD CARD********************/

}


/**************************************************************************/
/*!
@brief  Stores all data to the SD card
Author: Ben
*/
/**************************************************************************/
void DataLogClass::logData(void) {
	File myFile = sd.open(LOG_FILENAME, FILE_WRITE);
	if (myFile) {
		myFile.printf("%lu,%.3f,%.3f,%.3f,%.6f,", supStat.time, supStat.alt, supStat.vel, supStat.leftVel, supStat.rightVel);
		myFile.printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,", supStat.accel, supStat.rollAxisGrav, supStat.yawAxisGrav, supStat.pitchAxisGrav, supStat.rollAxisLin, supStat.yawAxisLin, supStat.pitchAxisLin);
		myFile.printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,", supStat.rollAxisGyro, supStat.yawAxisGyro, supStat.pitchAxisGyro, supStat.roll, supStat.yaw, supStat.pitch);
		myFile.printf("%.3f,%.3f,%.3f", supStat.alt_k, supStat.vel_k, supStat.accel_k);
		myFile.println("");
		myFile.close();
	}
}


/**************************************************************************/
/*!
@brief  Retrieves past flight data for tests.  Replaces sensor functions
Author: Jacob
*/
/**************************************************************************/
//void DataLogClass::readFromFile(struct stateStruct* destination) {
//	File myFile = sd.open(TEST_FILENAME, FILE_READ);
//	char place = '\n';
//	char number[20] = { '\0' };
//	short numPlace = 0, numCount = 0;
//	float value = 0;
//	int lineCount = 0;
//	static int linePlaceHolder = 0;
//
//	if (myFile) {
//		while (myFile.available()) {
//			place = myFile.read();
//
//			if (isdigit(place)) {
//				number[numPlace] = place;
//			}
//			else if (place == '.') {
//				number[numPlace] = place;
//			}
//			else if (place == '-') {
//				number[numPlace] = place;
//			}
//			else if (place == 'e' || place == '+') {
//				number[numPlace] = place;
//			}
//			else if (place == ',') {
//				value = numToFloat(number);
//				numCount++;
//				numPlace = -1;
//				switch (numCount - (lineCount * 3)) {
//				case 1:
//					destination->time = (value);
//					break;
//
//				case 2:
//					destination->alt = value;
//					break;
//				}
//				resetNumber(number);
//			}
//			else {
//				value = numToFloat(number);
//				numCount++;
//				numPlace = -1;
//				resetNumber(number);
//				lineCount++;
//				destination->accel = (value*-1);
//			}
//
//			if (numCount == 0) {
//
//			}
//			else {
//				if ((numCount % ((linePlaceHolder + 1) * 3)) == 0) {
//#if DEBUG_READFROMFILE
//					Serial.println("");
//					Serial.print("Time: ");
//					Serial.print(destination->time);
//					Serial.print(";");
//					Serial.print("Altitude: ");
//					Serial.print(destination->alt);
//					Serial.print(";");
//					Serial.print("Acceleration: ");
//					Serial.print(destination->accel);
//					Serial.print(";");
//#endif
//					linePlaceHolder++;
//					break;
//				}
//			}
//			numPlace++;
//
//		}
//
//		myFile.close();
//	}
//	else {
//		Serial.print("error opening the text file within readFromFile()!;");
//		logError(E_FILE_TEST);
//	}
//} //END readFromFile();


/**************************************************************************/
/*!
@brief  Stores error to VDSv2Errors.dat.
Author: Jacob
*/
/**************************************************************************/


void DataLogClass::logError(String error) {
	File myFile = sd.open(ERROR_FILENAME, FILE_WRITE);
	float time = (float)millis() / (float)1000;

	if (myFile) {
		myFile.printf("%0.6f", time);
		myFile.print(",");
		myFile.print(error);
		myFile.println("");
	}
	else {
		Serial.print("Unable to open error file");
	}
	myFile.close();
} // END logError()


  /*!
  @brief  Prepares varaibles for new launch
  Author: Jacob
  */
  /**************************************************************************/
void DataLogClass::newFlight(void) {
	sd.remove(LOG_FILENAME);                             //Removes prior flight data file
	sd.remove(ERROR_FILENAME);                                 //Removes prior error file

	File data = sd.open(LOG_FILENAME, FILE_WRITE);       //Creates new data file
	if (!data) {                                                    //If unable to be initiated, throw error statement.  Do nothing
		Serial.println("Data file unable to initiated.;");
	}
	else {
#if TEST_MODE                                               //Adds unique header depending on if VDS is in test or flight mode
		data.println("leftVel, rightVel, t(s), alt(m), vel(m/s), accel(m/s^2), kalman altitude(m), kalman velocity(m/s), kalman acceleration(m/s^2)");
#else
		data.println("times, alts, vels, leftVel, rightVel, accels, rollAxisGrav, yawAxisGrav, pitchAxisGrav, rollAxisLin, yawAxisLin, pitchAxisLin, rollAxisGyro, yawAxisGyro, pitchAxisGyro, roll, yaw, pitch, alts_k, vels_k, accels_k");
#endif
		data.close();                                               //Closes data file after use.
	}

	data = sd.open(ERROR_FILENAME, FILE_WRITE);                //Creates new error file
	if (!data) {                                                    //If unable to be initiated, throw error statement.  Do nothing
		Serial.println("Data file unable to initiated.;");
	}
	else {
		data.println("time(us),error");
		data.close();                                               //Closes data file after use.
	}

	//initializePastStates();
} // END newFlight()

DataLogClass DataLog;

