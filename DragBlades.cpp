#include "RCRClasses.h"


void DragBladesClass::init() {
	//setup motor pins
	pinMode(MOTOR_A, OUTPUT);
	pinMode(MOTOR_B, OUTPUT);
	pinMode(MOTOR_PWM, OUTPUT);
	//setup encoder pins
	pinMode(ENC_A, INPUT);
	pinMode(ENC_B, INPUT);
	//setup limit switch pins
	pinMode(LIM_OUT, INPUT);
	pinMode(LIM_IN, INPUT);
}

void DragBladesClass::motorDo(bool direction, uint8_t speed) {
	if (direction) {
		digitalWrite(MOTOR_A, HIGH);
		digitalWrite(MOTOR_B, LOW);
	}
	else {
		digitalWrite(MOTOR_A, LOW);
		digitalWrite(MOTOR_B, HIGH);
	}
	if (!digitalRead(LIM_IN) || !digitalRead(LIM_OUT)) {
		analogWrite(MOTOR_PWM, 0);
	}
	else {
		analogWrite(MOTOR_PWM, speed);
	}
}

void DragBladesClass::motorGoTo(int16_t encCmd)
{

}



DragBladesClass DragBlades;