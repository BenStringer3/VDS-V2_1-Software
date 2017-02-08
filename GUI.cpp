// 
// 
// 

//#include "GUI.h"
#include "RCRClasses.h"

/**************************************************************************/
/*!
@brief  Gets the current rocket and loads it
Author: Ben
*/
/**************************************************************************/
void GUIClass::init() {
	Serial.println("\r\n---Initializing rocket settings---");
	currentRocket = readUint8_t(3 * ROCKETSTRUCT_STORSIZE);
	if ((currentRocket > 3) || (currentRocket < 1) || isnan(currentRocket)) {
		Serial.printf("An incompatible rocket ID # was found on the EEPROM\r\nIs it possible that this is a new Teensy?\r\nSetting rocket ID # to 1\r\n");
		currentRocket = 1;
		writeUint8_t(currentRocket, 3 * ROCKETSTRUCT_STORSIZE);
	}
	loadRocket(currentRocket);
}

/**************************************************************************/
/*!
@brief  Prints one state and it's location in the pastRawStates array
Author: Jacob
*/
/**************************************************************************/
void GUIClass::printState(struct stateStruct state, int label) {
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
void GUIClass::printState(struct stateStruct state, String label) {
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
void GUIClass::printPastStates(struct stateStruct* pastStates) {
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
void GUIClass::printTitle(void) {
	//remember backslahses have to be double backslashed to print correctly
	Serial.println(F("             __      _______   _____  __      _____    __ "));
	delay(100);
	Serial.println(F("             \\ \\    / /  __ \\ / ____| \\ \\    / /__ \\   /_ |"));
	delay(100);
	Serial.println(F("              \\ \\  / /| |  | | (___    \\ \\  / /   ) |   | |"));
	delay(100);
	Serial.println(F("               \\ \\/ / | |  | |\\___ \\    \\ \\/ /   / /    | |"));
	delay(100);
	Serial.println(F("                \\  /  | |__| |____) |    \\  /   / /_   _| |"));
	delay(100);
	Serial.println(F("                 \\/   |_____/|_____/      \\/   |____| (_)_|"));
	delay(100);
	Serial.println("");
	Serial.println("             River City Rocketry's Variable Drag System");
	delay(100);
	Serial.println(" \t\t\t Full Scale Test Flights");
	delay(200);
	Serial.print(F("Software written by Jacob Cassady, "));
	delay(100);
	Serial.println(F("Ben Stringer, Lydia Sharp, and Denny Joy."));
	delay(100);
	Serial.println(F("With help from libraries written by Adafruit Industries."));
	delay(100);
	Serial.println(F("Mechanical hardware developed by Justin Johnson."));
	delay(100);
	Serial.println(F("Electrical hardware developed by Kenny Dang, Kristian Meyer, and Alora Mazarakis."));
	Serial.println("");
} // END printTitle()


  /**************************************************************************/
  /*!
  @brief  *HIDDEN* Menu Function.  Prints menu options.
  Author: Jacob
  */
  /**************************************************************************/
void GUIClass::printMenu(void) {
	if (TEST_MODE) {
		Serial.println("WARNING! TEST_MODE!");
	}
	if (!ERROR_LOGGING) {
		Serial.println("WARNING! ERROR LOGGING IS OFF!");
	}
#if LIMITSWITCHES_DETATCHED
	Serial.println("WARNING! LIMITSWITCHES_DETATCHED MODE IS ON!");
#endif
	delay(50);
	Serial.println("\n--------- Menu -----------;");
	delay(50);
	Serial.println("'S' - (S)ystem Check");
	delay(50);
	Serial.println("'D' - (D)rag Blades Check");
	delay(50);
	Serial.println("'T' - Toggle (T)est Mode");
	delay(50);
	Serial.println("'E' - Toggle (E)rror Logging");
	delay(50);
	Serial.println("'C' - (C)alibrate BNO055");
	delay(50);
	Serial.println("'R' - Edit (R)ockets");
	delay(50);
	Serial.println("'I' - Inch (I)nward");
	delay(50);
	Serial.println("'O' - Inch (O)utward");
	delay(50);
	Serial.println("'A' - (A)ccelerometer Test");
	delay(50);
	Serial.println("'B' - (B)arometric Pressure Sensor Test");
	delay(50);
	Serial.println("'M' - (M)otor Calibration & Test");
	delay(50);
	Serial.println("'F' - (F)light Mode");
} // END printMenu()

  /**************************************************************************/
  /*!
  @brief  Clears the serial buffer.. This
  is helpful for carriage returns and things of that sort that
  hang around after you got what you wanted.
  Author: Ben
  */
  /**************************************************************************/
void GUIClass::eatYourBreakfast() {
	delay(10);
	while (Serial.available() > 0) {
		delay(2);
		Serial.read();
	}
} // END eatYourBreakfast()




/**************************************************************************/
/*!
@brief  Loads the rocket indicated by whichOne from its designated spot in EEPROM
Author: Ben
*/
/**************************************************************************/
bool GUIClass::loadRocket(uint8_t whichOne) {
	rocket.dryMass = readFloat(0 + (whichOne - 1)*ROCKETSTRUCT_STORSIZE);
	rocket.propMass = readFloat(4 + (whichOne - 1)*ROCKETSTRUCT_STORSIZE);
	rocket.Cd_r = readFloat(8 + (whichOne - 1)*ROCKETSTRUCT_STORSIZE);
	rocket.Cd_b = readFloat(12 + (whichOne - 1)*ROCKETSTRUCT_STORSIZE);
	rocket.Ar = readFloat(16 + (whichOne - 1)* ROCKETSTRUCT_STORSIZE);
	rocket.Ab = readFloat(20 + (whichOne - 1)*ROCKETSTRUCT_STORSIZE);
	rocket.avgMotorThrust = readFloat(24 + (whichOne - 1)*ROCKETSTRUCT_STORSIZE);
	rocket.targetAlt = readFloat(28 + (whichOne - 1)*ROCKETSTRUCT_STORSIZE);
	rocket.interVel = readFloat(32 + (whichOne - 1)*ROCKETSTRUCT_STORSIZE);
	rocket.name = readString(36 + (whichOne - 1)*ROCKETSTRUCT_STORSIZE);
	rocket.Cmin = (rocket.Cd_r*rocket.Ar*RHO / 2 / rocket.dryMass);
	rocket.Cmax = (rocket.Cd_b*rocket.Ab*RHO / 2 / rocket.dryMass);
	rocket.Cspp = (rocket.Cmax + rocket.Cmin) / 2;
	rocket.interAlt = (rocket.targetAlt - log(sqrt((400 * rocket.Cmin*(rocket.interVel*rocket.interVel)) / 981 + 4) / 2) / rocket.Cmin);

	printRocket();

	//add if here to check rocket values against max nominal values
	if ((rocket.dryMass < MAX_EXP_DRYMASS) && (rocket.dryMass > 0) && 
		(rocket.propMass < MAX_EXP_PROPMASS) && (rocket.propMass > 0) && 
		(rocket.Cd_r < MAX_EXP_CD_R) && (rocket.Cd_r > 0) && 
		(rocket.Cd_b < MAX_EXP_CD_B) && (rocket.Cd_b > 0) && 
		(rocket.Ar < MAX_EXP_AR) && (rocket.Ar > 0) && 
		(rocket.Ab < MAX_EXP_AB) && (rocket.Ab > 0) &&
		(rocket.avgMotorThrust < MAX_EXP_AVGMOTORTHRUST) && (rocket.avgMotorThrust > 0) &&
		(rocket.targetAlt < MAX_EXP_TARGETALT) && (rocket.targetAlt > 0) &&
		(rocket.interVel < MAX_EXP_INTERVEL) && (rocket.interVel > 0) &&
		(rocket.interAlt < MAX_EXP_INTERALT) && (rocket.interVel > 0)) {
		return true;
	}
	else {
		Serial.println("WARNING: NON-NOMINAL VALUES DETECTED IN ROCKET SETTINGS");
		return false;
	}
}

/**************************************************************************/
/*!
@brief  Saves the rocket indicated by whichOne to the EEPROM in its designated spot
Author: Ben
*/
/**************************************************************************/
void GUIClass::saveRocket(uint8_t whichOne) {
	writeFloat(rocket.dryMass, 0 + (whichOne - 1)*ROCKETSTRUCT_STORSIZE);
	writeFloat(rocket.propMass, 4 + (whichOne - 1)*ROCKETSTRUCT_STORSIZE);
	writeFloat(rocket.Cd_r, 8 + (whichOne - 1)*ROCKETSTRUCT_STORSIZE);
	writeFloat(rocket.Cd_b, 12 + (whichOne - 1)*ROCKETSTRUCT_STORSIZE);
	writeFloat(rocket.Ar, 16 + (whichOne - 1)* ROCKETSTRUCT_STORSIZE);
	writeFloat(rocket.Ab, 20 + (whichOne - 1)*ROCKETSTRUCT_STORSIZE);
	writeFloat(rocket.avgMotorThrust, 24 + (whichOne - 1)*ROCKETSTRUCT_STORSIZE);
	writeFloat(rocket.targetAlt, 28 + (whichOne - 1)*ROCKETSTRUCT_STORSIZE);
	writeFloat(rocket.interVel, 32 + (whichOne - 1)*ROCKETSTRUCT_STORSIZE);
	writeString(rocket.name, 36 + (whichOne - 1)*ROCKETSTRUCT_STORSIZE);
	eeprom_write_block((void *)&whichOne, (unsigned char *)(3 * ROCKETSTRUCT_STORSIZE), 1);
}

/**************************************************************************/
/*!
@brief  Prints out the settings of the currently selected rocket
Author: Ben
*/
/**************************************************************************/
void GUIClass::printRocket() {
	Serial.print("Selected Rocket = ");
	Serial.println(rocket.name);
	delay(50);
	Serial.printf("Selected Rocket # = %d\r\n", currentRocket);
	delay(50);
	Serial.printf("dryMass = %f\r\n", rocket.dryMass);
	delay(50);
	Serial.printf("propMass = %f\r\n", rocket.propMass);
	delay(50);
	Serial.printf("Cd_r = %f\r\n", rocket.Cd_r);
	delay(50);
	Serial.printf("Cd_b = %f\r\n", rocket.Cd_b);
	delay(50);
	Serial.printf("Ar = %f\r\n", rocket.Ar);
	delay(50);
	Serial.printf("Ab = %f\r\n", rocket.Ab);
	delay(50);
	Serial.printf("avgMotorThrust = %d\r\n", rocket.avgMotorThrust);
	delay(50);
	Serial.printf("targetAlt = %d\r\n", rocket.targetAlt);
	delay(50);
	Serial.printf("interVel = %d\r\n", rocket.interVel);
	delay(50);
	Serial.printf("interAlt = %d\r\n", rocket.interAlt);
}

/**************************************************************************/
/*!
@brief  Code for navigating the rocket submenu
Author: Ben
*/
/**************************************************************************/
void GUIClass::rocketMenu() {
	bool exit = false;
	int tempVar;
	printRocketMenu();
	while (!exit) {
		if (Serial.available() > 0) {
			switch (Serial.read()) {
			case 'e'://edit rocket
				editRocket();
				saveRocket(currentRocket);
				loadRocket(currentRocket);
				break;
			case 's'://switch rockets
				Serial.println("------Switch Rockets-----");
				Serial.println("Type a number 1-3");
				GUI.eatYourBreakfast();
				while (!(Serial.available() > 0)) {
					//wait
				}
				tempVar = Serial.parseInt();
				if ((tempVar > 3) || (tempVar < 1)) {
					Serial.println(" Type a number between 1 & 3");
				}
				else {
					currentRocket = tempVar;
				}
				eeprom_write_block((void *)&currentRocket, (unsigned char *)(3 * ROCKETSTRUCT_STORSIZE), 1);
				loadRocket(currentRocket);
				break;
			case 'x':
				exit = true;
				return;
				break;
			default:
				Serial.println("unknown code received - rocket submenu");
				break;
			}
			GUI.eatYourBreakfast();
			printRocketMenu();
		}
	}
}

/**************************************************************************/
/*!
@brief  Asks for user input to change the stored rocket settings
Author: Ben
*/
/**************************************************************************/
void GUIClass::editRocket() {
	int eqlIndex;
	String myString;
	String myVariable;
	Serial.println("------Editing Rockets------");
	printRocket();
	Serial.printf("Enter the variable you want to change in the following format:\r\nvariableName=value;\r\n");
	GUI.eatYourBreakfast();
	while (!(Serial.available() > 0)) {
		//wait
	}
	delay(10);
	myString = Serial.readStringUntil(';');
	Serial.print("String received: ");
	Serial.println(myString);
	eqlIndex = myString.indexOf('=');
	myVariable = myString.substring(0, eqlIndex);
	Serial.print("variable parsed: ");
	Serial.println(myVariable);
	if (myVariable.equals("dryMass")) {
		rocket.dryMass = myString.substring(eqlIndex + 1).toFloat();
	}
	else if (myVariable.equals("propMass")) {
		rocket.propMass = myString.substring(eqlIndex + 1).toFloat();
	}
	else if (myVariable.equals("Cd_r")) {
		rocket.Cd_r = myString.substring(eqlIndex + 1).toFloat();
	}
	else if (myVariable.equals("Cd_b")) {
		rocket.Cd_b = myString.substring(eqlIndex + 1).toFloat();
	}
	else if (myVariable.equals("Ar")) {
		rocket.Ar = myString.substring(eqlIndex + 1).toFloat();
	}
	else if (myVariable.equals("Ab")) {
		rocket.Ab = myString.substring(eqlIndex + 1).toFloat();
	}
	else if (myVariable.equals("avgMotorThrust")) {
		rocket.avgMotorThrust = myString.substring(eqlIndex + 1).toInt();
	}
	else if (myVariable.equals("targetAlt")) {
		rocket.targetAlt = myString.substring(eqlIndex + 1).toInt();
	}
	else if (myVariable.equals("interVel")) {
		rocket.interVel = myString.substring(eqlIndex + 1).toInt();
	}
	else if (myVariable.equals("name")) {
		rocket.name = myString.substring(eqlIndex + 1);
		Serial.println(rocket.name);
	}
	else {
		Serial.println("Bad string received");
	}
}

/**************************************************************************/
/*!
@brief  Displays rocket submenu
Author: Ben
*/
/**************************************************************************/
void GUIClass::printRocketMenu() {
	Serial.printf("\r\n------Rocket SubMenu-------\r\n");
	delay(100);
	Serial.println("'e' - (e)dit rockets");
	delay(100);
	Serial.println("'s' - (s)witch rockets");
	delay(100);
	Serial.println("'x' - e(x)it rocket submenu");
}

/**************************************************************************/
/*!
@brief  All read/write EEPROM functions below. Read/writes different variable
types to the EEPROM
Author: Ben
*/
/**************************************************************************/
float GUIClass::readFloat(int address) {
	float out;
	eeprom_read_block((void *)&out, (unsigned char *)address, 4);
	return out;
}

uint8_t GUIClass::readUint8_t(int address) {
	uint8_t out;
	eeprom_read_block((void *)&out, (unsigned char *)address, 1);
	return out;
}

void GUIClass::writeUint8_t(uint8_t value, int address) {
	eeprom_write_block((void *)&value, (unsigned char *)address, 1);
}

void GUIClass::writeFloat(float value, int address) {
	eeprom_write_block((void *)&value, (unsigned char *)address, 4);
}

void GUIClass::writeString(String value, int address) {
	//eeprom_write_block((void *)&value, (unsigned char *)address, 4);
	byte buf[4];
	value.getBytes(buf, 4);
	eeprom_write_block((void *)&buf, (unsigned char *)address, 4);

	/*const byte* p = (const byte*)(const void*)&value;
	unsigned int i;
	for (i = 0; i < 4; i++) {
	EEPROM.write(address++, *p++);
	}*/
}

String GUIClass::readString(int address) {
	char out[32];
	eeprom_read_block((void *)&out, (unsigned char *)address, 4);
	return out;
}

GUIClass GUI;

