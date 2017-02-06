// 
// 
// 

#include "RCRClasses.h"

void RocketsClass::init() {
	Serial.println("Initializing rocket settings");
	currentRocket = readUint8_t(3 * ROCKETSTRUCT_STORSIZE);
	if ((currentRocket > 3) || (currentRocket < 1) || isnan(currentRocket)) {
		Serial.printf("An incompatible rocket ID # was found on the EEPROM\r\nIs it possible that this is a new Teensy?\r\nSetting rocket ID # to 1\r\n");
		currentRocket = 1;
		writeUint8_t(currentRocket, 3 * ROCKETSTRUCT_STORSIZE);
	}
	loadRocket(currentRocket);
}

void RocketsClass::loadRocket(uint8_t whichOne) {
	rocket.dryMass = readFloat(0 + (whichOne-1)*ROCKETSTRUCT_STORSIZE);
	rocket.propMass = readFloat(4 + (whichOne-1)*ROCKETSTRUCT_STORSIZE);
	rocket.Cd_r = readFloat(8 + (whichOne-1)*ROCKETSTRUCT_STORSIZE);
	rocket.Cd_b = readFloat(12 + (whichOne-1)*ROCKETSTRUCT_STORSIZE);
	rocket.Ar = readFloat(16 + (whichOne-1)* ROCKETSTRUCT_STORSIZE);
	rocket.Ab = readFloat(20 + (whichOne-1)*ROCKETSTRUCT_STORSIZE);
	rocket.avgMotorThrust = readFloat(24 + (whichOne-1)*ROCKETSTRUCT_STORSIZE);
	rocket.targetAlt = readFloat(28 + (whichOne-1)*ROCKETSTRUCT_STORSIZE);
	rocket.interVel = readFloat(32 + (whichOne-1)*ROCKETSTRUCT_STORSIZE);
	Serial.println(rocket.propMass);
	rocket.name = readString(36 + (whichOne-1)*ROCKETSTRUCT_STORSIZE);
	Serial.println(rocket.name);
	rocket.Cmin = (rocket.Cd_r*rocket.Ar*RHO / 2 / rocket.dryMass);
	rocket.Cmax = (rocket.Cd_b*rocket.Ab*RHO / 2 / rocket.dryMass);
	rocket.Cspp = (rocket.Cmax + rocket.Cmin) / 2;
	rocket.interAlt = (rocket.targetAlt - log(sqrt((400 * rocket.Cmin*(rocket.interVel*rocket.interVel)) / 981 + 4) / 2) / rocket.Cmin);

	printRocket();

}

void RocketsClass::saveRocket(uint8_t whichOne) {
	writeFloat(rocket.dryMass, 0 + (whichOne-1)*ROCKETSTRUCT_STORSIZE);
	writeFloat(rocket.propMass, 4 + (whichOne-1)*ROCKETSTRUCT_STORSIZE);
	writeFloat(rocket.Cd_r  , 8 + (whichOne-1)*ROCKETSTRUCT_STORSIZE);
	writeFloat(rocket.Cd_b  , 12 + (whichOne-1)*ROCKETSTRUCT_STORSIZE);
	writeFloat(rocket.Ar     , 16 + (whichOne-1)* ROCKETSTRUCT_STORSIZE);
	writeFloat(rocket.Ab     , 20 + (whichOne-1)*ROCKETSTRUCT_STORSIZE);
	writeFloat(rocket.avgMotorThrust, 24 + (whichOne-1)*ROCKETSTRUCT_STORSIZE);
	writeFloat(rocket.targetAlt, 28 + (whichOne-1)*ROCKETSTRUCT_STORSIZE);
	writeFloat(rocket.interVel , 32 + (whichOne-1)*ROCKETSTRUCT_STORSIZE);
	writeString(rocket.name    , 36 + (whichOne-1)*ROCKETSTRUCT_STORSIZE);
	eeprom_write_block((void *)&whichOne, (unsigned char *)(3 * ROCKETSTRUCT_STORSIZE), 1);
}

void RocketsClass::printRocket() {
	Serial.print("\r\nSelected Rocket = ");
	Serial.println(rocket.name);
	Serial.printf("Selected Rocket # = %d\r\n", currentRocket);
	Serial.printf("dryMass = %f\r\n", rocket.dryMass);
	Serial.printf("propMass = %f\r\n", rocket.propMass);
	Serial.printf("Cd_r = %f\r\n", rocket.Cd_r);
	Serial.printf("Cd_b = %f\r\n", rocket.Cd_b);
	Serial.printf("Ar = %f\r\n", rocket.Ar);
	Serial.printf("Ab = %f\r\n", rocket.Ab);
	Serial.printf("avgMotorThrust = %d\r\n", rocket.avgMotorThrust);
	Serial.printf("targetAlt = %d\r\n", rocket.targetAlt);
	Serial.printf("interVel = %d\r\n", rocket.interVel);
	Serial.printf("interAlt = %d\r\n", rocket.interAlt);
}

void RocketsClass::rocketMenu() {
	bool exit = false;
	char response;
	int tempVar;
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

void RocketsClass::editRocket() {
	int eqlIndex;
	String myString;
	String myVariable;
	Serial.println("------Editing Rockets------");
	printRocket();
	Serial.printf("\r\n\nEnter the variable you want to change in the following format:\r\n\nvariableName=value;\r\n\n");
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

void RocketsClass::printRocketMenu() {
	Serial.printf("\r\n------Rocket SubMenu-------\r\n");
	delay(100);
	Serial.println("'e' - (e)dit rockets");
	delay(100);
	Serial.println("'s' - (s)witch rockets");
	delay(100);
	Serial.println("'x' - e(x)it rocket submenu");
}


float RocketsClass::readFloat(int address) {
	float out;
	eeprom_read_block((void *)&out, (unsigned char *)address, 4);
	return out;
}

uint8_t RocketsClass::readUint8_t(int address) {
	uint8_t out;
	eeprom_read_block((void *)&out, (unsigned char *)address, 1);
	return out;
}

void RocketsClass::writeUint8_t(uint8_t value, int address) {
	eeprom_write_block((void *)&value, (unsigned char *)address, 1);
}

void RocketsClass::writeFloat(float value, int address) {
	eeprom_write_block((void *)&value, (unsigned char *)address, 4);
}

void RocketsClass::writeString(String value, int address) {
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

String RocketsClass::readString(int address) {
	char out[32];
	eeprom_read_block((void *)&out, (unsigned char *)address, 4);
	return out;
}



RocketsClass Rockets;