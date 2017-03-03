// 
// 
// 

//#include "DataLog.h"
#include "RCRClasses.h"

void DataLogClass::init()
{
	Serial.println("\r\n-------DatLog.init-------");
	Serial.println("Initializing SD card");
	if (!sd.begin()) {                                            //Determine if microSD card is initialized and ready to be used.
		SD_GO = false;
		Serial.println("No SD card DETECTED!");
		return;
	}
	else {
		SD_GO = true;
		Serial.println("SD card Initialized");                    //If microSD card id ready, begin initialization of flight.  Includes creation of dataFile and it's heading
	}
	File myFile = sd.open(TEST_FILENAME, FILE_READ);
	if (myFile && myFile.available()) {
		testFileSize = myFile.size();
		Serial.printf("Test file size: %d", testFileSize);
		myFile.close();
	}
	else {
		Serial.print("Couldn't open test file, ");
		Serial.print(TEST_FILENAME);
		Serial.println(" to determine its size.");
	}
}

void DataLogClass::printTestFileNames() {
	FatFile dir;
	char name[32];
	//dir.open(sd.vwd(), "/tests/", O_READ);
	sd.ls("/tests/");

	dir.openNext(sd.vwd());
	dir.getName(name,32);
	Serial.println(name);
}

/**************************************************************************/
/*!
@brief  Stores all data to the SD card
Author: Ben
*/
/**************************************************************************/
void DataLogClass::logData(bool testMode) {
	File myFile = sd.open(LOG_FILENAME, FILE_WRITE);
	if (myFile) {
		myFile.printf("%lu,%.3f,%.3f,%.3f,%.3f,%.6f,", supStat.time, supStat.alt, supStat.vel, supStat.accel, supStat.leftVel, supStat.rightVel);
		if (!testMode) {
			myFile.printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,", supStat.rollAxisGrav, supStat.yawAxisGrav, supStat.pitchAxisGrav, supStat.rollAxisLin, supStat.yawAxisLin, supStat.pitchAxisLin);
			myFile.printf("%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,", supStat.rollAxisGyro, supStat.yawAxisGyro, supStat.pitchAxisGyro, supStat.roll, supStat.yaw, supStat.pitch);
		}
		//myFile.printf("%.3f,%.3f,%.3f,", supStat.alt_k, supStat.vel_k, supStat.accel_k); //log kalman filter
		myFile.printf("%.3f,%d,%d,%d,%d,%d,%d,%d", supStat.vSPP, supStat.encPos, supStat.encPosCmd, supStat.limit_out, supStat.limit_in, supStat.encMax, supStat.encMin, supStat.mtrSpdCmd);
		myFile.println("");
		myFile.close();
	}
}

/**************************************************************************/
/*!
@brief  Reads "sensor data" from a test file
Author: Ben
*/
/**************************************************************************/
bool DataLogClass::readCSV(struct stateStruct* destination) {
	File myFile = sd.open(TEST_FILENAME, FILE_READ);
	float time, alt, accel;
	bool returnVal;
	Serial.println("Asdf");
	if (myFile && myFile.available()) {
		myFile.seek(pos);
		time = myFile.parseFloat();
		alt = myFile.parseFloat();
		accel = myFile.parseFloat();
		//myFile.readStringUntil('\n');
		pos = myFile.position();	
		myFile.close();
		destination->time = time*TEST_TIME_DIVISOR/10;
		destination->alt = alt;
		destination->accel = accel;
		if (pos + 1 >= testFileSize) {
			Serial.printf("pos: %d\r\n", pos);
			pos = 0;
			return false;
		}
		else {
			returnVal = true;
		}
			Serial.println("");
			Serial.println("READCSV---------------------");
			Serial.print("position = ");
			Serial.println(pos);
			Serial.print("time = ");
			Serial.print(time, 4);
			Serial.println(" [sec]");
			Serial.print("altitude = ");
			Serial.println(alt, 4);
			Serial.print("acceleration = ");
			Serial.println(accel, 4);
	}
	else {
		Serial.print("error opening the text file within readCSV()!");
		logError(E_FILE_TEST);
		return false;
	}
	return returnVal;
}

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

  /**************************************************************************/
  /*!
  @brief  Prepares varaibles for new launch
  Author: Jacob
  */
  /**************************************************************************/
void DataLogClass::newFlight(bool testMode) {
	sd.remove(LOG_FILENAME);                             //Removes prior flight data file
	sd.remove(ERROR_FILENAME);                                 //Removes prior error file

	File data = sd.open(LOG_FILENAME, FILE_WRITE);       //Creates new data file
	if (!data) {                                                    //If unable to be initiated, throw error statement.  Do nothing
		Serial.println("Data file unable to initiated.;");
		SD_GO = false;
	}
	else {
		if (testMode) {                                               //Adds unique header depending on if VDS is in test or flight mode
			data.println(LOG_HEADER_STRING_TEST);
		}
		else {
			data.println(LOG_HEADER_STRING_FLIGHT);
		}
		data.close();                                               //Closes data file after use.
	}

	data = sd.open(ERROR_FILENAME, FILE_WRITE);                //Creates new error file
	if (!data) {                                                    //If unable to be initiated, throw error statement.  Do nothing
		Serial.println("Data file unable to initiated.;");
		SD_GO = false;
	}
	else {
		data.println("time(us),error");
		data.close();                                               //Closes data file after use.
	}

	//initializePastStates();
} // END newFlight()

DataLogClass DataLog;

