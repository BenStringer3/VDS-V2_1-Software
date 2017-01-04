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
  @brief  *HIDDEN* Menu Function.  Prints menu options.
  Author: Jacob
  */
  /**************************************************************************/
void GUIClass::printMenu(void) {
	Serial.println("\n\n--------- Menu -----------;");
	Serial.println("'S' - System Check;");
	Serial.println("'C' - Calibrate BNO055;");
	Serial.println("'A' - Accelerometer Test;");
	Serial.println("'B' - Barometric Pressure Sensor Test;");
	Serial.println("'K' - Kalman Filter Test;");
	Serial.println("'F' - Flight Mode;");
} // END printMenu()

GUIClass GUI;

