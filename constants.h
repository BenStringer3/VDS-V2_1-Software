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
#define DEBUG_READCSV			true
#define DEBUG_GETACCELERATION	false
#define DEBUG_ALTITUDEPLZ		false
#define DEBUG_V_SPP				false
#define DEBUG_MOTORGOTO			true
#define DEBUG_PIDCOMPUTE		false

#define TEST_MODE				true				//print statement indicating test mode. Set to TRUE for ground testing. SET TO FALSE FOR FLIGHT!
#define DATA_LOGGING			false
#define TEST_FILENAME			"12_18_16_test.dat"   //"8_6_16_test.dat"
#define LOG_FILENAME			"VDSv2_1Data.dat"
#define ERROR_FILENAME			"VDSv2_1Errors.dat"

//miscallaneous constants
#define ENC_RANGE			280
#define AIRBRAKES_GAIN		50
#define BUFF_N				15					//Number of Data Points per accel and alt array. MUST BE EVEN
#define MAX_EXP_VEL			300

//physical constants. Used for Kalman filter and SPP
#define DRY_MASS			3.3396
#define PROP_MASS			0.226
#define RHO					1.18
#define G					9.81
#define CD_R				0.47
#define CD_B				0.54
#define A_R					0.00591134302
#define A_B					(A_R + 0.005938865542)
#define AVG_MOTOR_THRUST	120

//SPP constants
#define TARGET_ALTITUDE		454
#define C_MIN				(CD_R*A_R*RHO/2/DRY_MASS)
#define C_MAX				(CD_B*A_B*RHO/2/DRY_MASS)
#define C_SPP				((C_MIN+C_MAX)/2)
#define INTER_VEL			40				//The velocity at which the piecwise SPP is split in two
#define INTER_ALT			(TARGET_ALTITUDE  - log(sqrt((400*C_MIN*(INTER_VEL*INTER_VEL))/981 + 4)/2)/C_MIN)

//pins
#define LED			13
#define MOTOR_A		4
#define MOTOR_B		5
#define MOTOR_PWM	8
#define ENC_A		6
#define ENC_B		7
#define LIM_IN		8
#define LIM_OUT		9

//motor stuff
#define CLOCKWISE			true
#define COUNTERCLOCKWISE	false
#define SETPOINT_TOLERANCE	3
#define SETPOINT_INAROW		8
#define MOTORTEST_DELAY_MS  15
#define DEADZONE_MAX		60
#define DEADZONE_MIN		10

//PID stuff
#define KP	3.149
#define KI	0.5
#define KD  0.0816
#define KN	488
#define PID_RESET_TIME_S	0.021

//Erorr Logging
#define SENSOR_UNIT        ('0')             //Notes a sensor was not initialized
#define UNCALIBRATED_BNO   ('1')             //Notes the BNO055's Accelerometer is not calibrated to a value of 3
#define NAN_VEL            ('2')             //Notes the velocity value retrieved from the algorithm is nan
#define NONNOM_VEL         ('3')             //Notes the velocity value retrieved from the algorithm is non-nominal
#define E_FILE_FLIGHT      ('4')             //Notes the program was unable to open the flight data
#define E_FILE_TEST        ('5')             //Notes the program was unable to open the test data
#define INVALID_MENU       ('6')             //Notes an invalid response was sent to the menu.
#define NAN_UK             ('7')             //Notes a u_k value of nan was found in kalman filter
