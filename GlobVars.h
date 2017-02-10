//constants.h
#include <WString.h>

//debug settings.
//Since print statements make things run slow and take up memory to store. choosing what to compile
//can be a good way to make code more efficient

#define DEBUG_EMERGENCY			false				//toggles print statements that are super detailed
#define DEBUG_FLIGHTMODE		false
#define DEBUG_KALMAN			false
#define DEBUG_VELOCITY			false
#define	DEBUG_RAWSTATE			false
#define DEBUG_READFROMFILE		false
#define DEBUG_READCSV			false
#define DEBUG_GETACCELERATION	false
#define DEBUG_ALTITUDEPLZ		false
#define DEBUG_V_SPP				false
#define DEBUG_MOTORGOTO			false
#define DEBUG_PIDCOMPUTE		false

#define BMP280					true
//#define TEST_FILENAME			"12_18_16_test.dat"   //"8_6_16_test.dat"
#define LOG_FILENAME			"VDSv2_1Data.dat"
#define ERROR_FILENAME			"VDSv2_1Errors.dat"

//miscallaneous constants
#define TIME_DIVISOR		100000			//time divisor to convert ms to sec
#define TEST_TIME_DIVISOR	1000000			//time divisor to convert uS to sec
#define SEALVL_PRESS		1013.25
#define ENC_RANGE			240
#define AIRBRAKES_GAIN		50
#define BUFF_N				31					//Number of Data Points per accel and alt array. 
#define MAX_EXP_VEL			300
#define MIN_EXP_VEL			-50

//max nominal rocket values
#define MAX_EXP_DRYMASS			30
#define MAX_EXP_PROPMASS		10
#define MAX_EXP_CD_R			1
#define MAX_EXP_CD_B			2
#define MAX_EXP_AR				0.5
#define MAX_EXP_AB				0.5
#define MAX_EXP_AVGMOTORTHRUST	3000
#define MAX_EXP_TARGETALT		1700
#define MAX_EXP_INTERVEL		250
#define MAX_EXP_INTERALT		1700


//physical constants. Used for Kalman filter and SPP
#define RHO					1.18
#define G					9.81

//pins
#define LED			13
#define MOTOR_A		2//4
#define MOTOR_B		3//5
#define MOTOR_PWM	6//8
#define ENC_A		4//6
#define ENC_B		5//7
#define LIM_IN		23
#define LIM_OUT		22
#define LIM_3v3		21

//motor stuff
#define OUTWARD				true
#define INWARD				false
#define SETPOINT_TOLERANCE	3
#define SETPOINT_INAROW		8
#define MOTORTEST_DELAY_MS  15
#define DEADZONE_MAX		60
#define DEADZONE_MIN		10
#define DEADZONE_BOOST		0

//PID stuff
#define KP	3.149							//PID's P coefficient
#define KI	0.5								//PID's I coefficient
#define KD  0.0816							//PID's D coefficient
#define KN	488								//PID's N coefficient (unused currently?)
#define PID_RESET_TIME_S	0.021			//The amount of time (sec) over which that the PID's integrated error will reset to 0

//Erorr Logging
#define SENSOR_UNIT        ('0')             //Notes a sensor was not initialized
#define UNCALIBRATED_BNO   ('1')             //Notes the BNO055's Accelerometer is not calibrated to a value of 3
#define NAN_VEL            ('2')             //Notes the velocity value retrieved from the algorithm is nan
#define NONNOM_VEL         ('3')             //Notes the velocity value retrieved from the algorithm is non-nominal
#define E_FILE_FLIGHT      ('4')             //Notes the program was unable to open the flight data
#define E_FILE_TEST        ('5')             //Notes the program was unable to open the test data
#define INVALID_MENU       ('6')             //Notes an invalid response was sent to the menu.
#define NAN_UK             ('7')             //Notes a u_k value of nan was found in kalman filter
#define UNCALIBRATED_ACCEL ('8')
#define UNCALIBRATED_GYRO  ('9')
#define UNCALIBRATED_MAGN  ("10")
#define ENC_RANGE_ERROR		("11")

#ifndef _GLOBVARS_h
#define _GLOBVARS_h

#define ROCKETSTRUCT_STORSIZE (10*4) //bytes
struct rocketStruct {
	String name;
	float dryMass;
	float propMass;
	float Cd_r;
	float Cd_b;
	float Ar;
	float Ab;
	int avgMotorThrust;
	int targetAlt;
	int interVel;
	int interAlt;
	float Cmin;
	float Cmax;
	float Cspp;
} ;
extern rocketStruct rocket;
extern bool TEST_MODE; 
extern bool BMP_GO;
extern bool BNO_GO;
extern bool SD_GO;
extern bool DragBlades_GO;
extern String TEST_FILENAME;
#endif
