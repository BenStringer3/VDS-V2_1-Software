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
		myFile.printf("%lu,%.3f,%.3f,%.3f,%.3f,%.6f,", supStat.time, supStat.alt, supStat.vel, supStat.accel, supStat.leftVel, supStat.rightVel);
#if !TEST_MODE
		myFile.printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,",  supStat.rollAxisGrav, supStat.yawAxisGrav, supStat.pitchAxisGrav, supStat.rollAxisLin, supStat.yawAxisLin, supStat.pitchAxisLin);
		myFile.printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,", supStat.rollAxisGyro, supStat.yawAxisGyro, supStat.pitchAxisGyro, supStat.roll, supStat.yaw, supStat.pitch);
#endif
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
void DataLogClass::readFromFile(struct stateStruct* destination) {
	File myFile = sd.open(TEST_FILENAME, FILE_READ);
	char place = '\n';
	char number[20] = { '\0' };
	short numPlace = 0, numCount = 0;
	float value = 0;
	int lineCount = 0;
	static int linePlaceHolder = 0;

	if (myFile) {
		while (myFile.available()) {
			place = myFile.read();

			if (isdigit(place)) {
				number[numPlace] = place;
			}
			else if (place == '.') {
				number[numPlace] = place;
			}
			else if (place == '-') {
				number[numPlace] = place;
			}
			else if (place == 'e' || place == '+') {
				number[numPlace] = place;
			}
			else if (place == ',') {
				value = numToFloat(number);
				numCount++;
				numPlace = -1;
				switch (numCount - (lineCount * 3)) {
				case 1:
					destination->time = (value);
					break;

				case 2:
					destination->alt = value;
					break;
				}
				resetNumber(number);
			}
			else {
				value = numToFloat(number);
				numCount++;
				numPlace = -1;
				resetNumber(number);
				lineCount++;
				destination->accel = (value*-1);
			}

			if (numCount == 0) {

			}
			else {
				if ((numCount % ((linePlaceHolder + 1) * 3)) == 0) {
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
	}
	else {
		Serial.print("error opening the text file within readFromFile()!;");
		logError(E_FILE_TEST);
	}
} //END readFromFile();

bool DataLogClass::readCSV(struct stateStruct* destination) {
	File myFile = sd.open(TEST_FILENAME, FILE_READ);
	float time, alt, accel;
	bool returnVal;
	if (myFile && myFile.available()) {
		if (pos + 1 >= 35155) {
			return false;
		}
		else {
			returnVal = true;
		}
		myFile.seek(pos);
		time = myFile.parseFloat();
		alt = myFile.parseFloat();
		accel = myFile.parseFloat();
		//myFile.readStringUntil('\n');
		pos = myFile.position();	
		myFile.close();
		destination->time = time*1000000;
		destination->alt = alt;
		destination->accel = accel;
#if	DEBUG_READCSV
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
#endif
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
		data.println("times, alts, vels, leftVel, rightVel, accels, alts_k, vels_k, accels_k");
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


  /**************************************************************************/
  /*!
  @brief  Resets (char)number array to NULL values.
  Author: Jacob
  */
  /**************************************************************************/
void DataLogClass::resetNumber(char* number) {
	for (short i = 0; i<20; i++) {
		number[i] = '\0';
	}
} //END resetNumber();


  /**************************************************************************/
  /*!
  @brief  Converts a char number to a floating point value
  Author: Jacob
  */
  /**************************************************************************/
float DataLogClass::charToFloat(char input) {
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
float DataLogClass::numToFloat(char* number) {
	short index = 0, decimalIndex = 0;
	boolean decimal = false, e = false, negative = false, one = false;
	float result = 0, temp = 0;

	while (number[index] != '\0') {
		if (number[index] == 'e') {
			e = true;
		}
		else if (number[index] == '.') {
			decimal = true;
		}
		else if (number[index] == '-') {
			negative = true;
		}
		else if (number[index] == '+') {
			;
		}
		else {
			temp = charToFloat(number[index]);
			if (e) {
				result *= (float)pow(10, temp);
			}
			else if (decimal) {
				temp *= pow(10, (-1 * (decimalIndex + 1)));
				result += temp;
				decimalIndex++;
			}
			else {
				if (one) {
					result *= 10;
				}
				else {
					one = true;
				}
				result += temp;
			}
		}
		index++;
	}
	if (negative) {
		result *= -1;
	}
	return result;
} //END numToFloat();

DataLogClass DataLog;

