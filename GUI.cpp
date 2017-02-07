// 
// 
// 

//#include "GUI.h"
#include "RCRClasses.h"

void GUIClass::init()
{


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

GUIClass GUI;

