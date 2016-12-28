// hashTagDefines.h

//debug settings.
//Since print statements make things run slow and take up memory to store. choosing what to compile
//can be a good way to make code more efficient

#define DEBUG_EMERGENCY			false				//toggles print statements that are super detailed
#define DEBUG_FLIGHTMODE		false
#define DEBUG_KALMAN			false
#define DEBUG_VELOCITY			false
#define	DEBUG_RAWSTATE			false
#define DEBUG_READFROMFILE		false
#define DEBUG_GETACCELERATION	false
#define DEBUG_ALTITUDEPLZ		false

#define TEST_MODE				false				//print statement indicating test mode. Set to TRUE for ground testing. SET TO FALSE FOR FLIGHT!
#define TEST_FILENAME			"8_6_16_test.dat"

//miscallaneous constants
#define ENC_RANGE			400
#define BUFF_N				15					//Number of Data Points per accel and alt array. MUST BE EVEN
#define MAX_EXP_VEL			300

//physical constants. Used for Kalman filter and SPP
#define POST_BURN_MASS		3.111644
#define RHO					1.225
#define CD_R				0.4
#define CD_B				0.54
#define A_R					0.00591134302
#define A_B					A_R + 0.005938865542
#define AVG_MOTOR_THRUST	150

//SPP constants
#define TARGET_ALTITUDE		1609
#define ON_TRACK_VELOCITY	125

//pins
#define LED 13

//Erorr Logging
#define SENSOR_UNIT        ('0')             //Notes a sensor was not initialized
#define UNCALIBRATED_BNO   ('1')             //Notes the BNO055's Accelerometer is not calibrated to a value of 3
#define NAN_VEL            ('2')             //Notes the velocity value retrieved from the algorithm is nan
#define NONNOM_VEL         ('3')             //Notes the velocity value retrieved from the algorithm is non-nominal
#define E_FILE_FLIGHT      ('4')             //Notes the program was unable to open the flight data
#define E_FILE_TEST        ('5')             //Notes the program was unable to open the test data
#define INVALID_MENU       ('6')             //Notes an invalid response was sent to the menu.
#define NAN_UK             ('7')             //Notes a u_k value of nan was found in kalman filter

