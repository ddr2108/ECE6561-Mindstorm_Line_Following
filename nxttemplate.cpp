///////////////////////////////////////////////////////////////////////
//
// Line following Mindstorm robot
//
///////////////////////////////////////////////////////////////////////

// ECRobot++ API
#include "LightSensor.h"
#include "SonarSensor.h"
#include "SoundSensor.h"
#include "TouchSensor.h"
#include "Clock.h"
#include "Lcd.h"
#include "Motor.h"
#include "Nxt.h"
using namespace ecrobot;

#define CENTER 60
#define FULLSPEED -100

// Follow State Machine
#define LEFT_FOLLOW  0
#define RIGHT_FOLLOW 1
#define STRAIGHT_FOLLOW 2
#define STOP_FOLLOW  3
//Find Line State Machine
#define STRAIGHT   0 
#define LEFT_TURN  1
#define TURNED	   2
// High Level State Machine
#define START  0
#define FIND_LINE 1
#define FOLLOW 2
#define STOP  3
#define FOLLOW_AFTER_STOP 4

#define MAIN_WAIT 200

#define WHITE 0
#define BLACK 1
#define GRAY  2
//Flags for when multiple sensors
#define LEFT  0
#define RIGHT 1

extern "C" 
{
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"

//Follow state machine
void dispatcherFollow();
int straightFollow();
int leftFollow();
int rightFollow();
int stopFollow();
//Find line state machine
void dispatcherFindLine();
int straightFindLine();
int leftTurnFindLine();
//General state machine
void dispatcherMain();
int begin();
int findLine();
int follow();
int stop();

//IO functions
int getLight(int light);
int getSonar();
int getTouch();
void setMotor(int motor, int PWM);

// Initialize Sensors
LightSensor  leftLight(PORT_1, true); // init light sensor ON
LightSensor  rightLight(PORT_4, true);
SonarSensor  sonar(PORT_2);
TouchSensor  touch(PORT_3);

// Initialize Actuators
Motor steering(PORT_A);
Motor rightMotor(PORT_B);
Motor leftMotor(PORT_C);

// Initialize Clock
Clock clock;

// Initialize LCD
Lcd lcd;		

int maxCount = 100;

///////////////////////////////////////////////////////////////////////
////////////////Interacting with IO////////////////////////////////////
///////////////////////////////////////////////////////////////////////

/*Function: getLight()
* Get light from sensor
*
* Parameters:
*	int light - LEFT or RIGHT light sensor
*
* Returns:
*	int - value returned by light sensor
*/
int getLight(int light){
	int brightness = 0;

	//Get data from light sesnor
	if (light==LEFT){
		brightness = leftLight.getBrightness();
	}else if (light==RIGHT){
		brightness = rightLight.getBrightness();
	}

	//Based on brightness, return color
	if (brightness>520){
		return WHITE;
	}else if (brightness<450){
		return BLACK;
	}else{
		return GRAY;
	}
}

/*Function: getSonar()
* Gets distance according to sonar 
*
* Returns: 
*	int - distance returned by sonar
*/
int getSonar(){
	return sonar.getDistance();	
}

/*Function: getTouch()
* Get value of touch sensor
*
* Returns:
*	int - value of touch sensor
*/
int getTouch(){
	return touch.isPressed();
}

/*Function: setMotor()
* Sets PWM on motor
*
* Parameters:
*	int motor - LEFT or RIGHT motor
*	int PWN	  - PWM to be sent to motor
*/
void setMotor(int motor, int PWM){
	if (motor==LEFT){
		leftMotor.setPWM(PWM);
	}else if (motor==RIGHT){
		rightMotor.setPWM(PWM);
	}
}

///////////////////////////////////////////////////////////////////////	

///////////////////////////////////////////////////////////////////////
////////////////////STATE MACHINES//////////////////////////////////////
///////////////////////////////////////////////////////////////////////

////////////////////FOLLOW STATE MACHINE////////////////////////////

/*Function: straightFollow()
* Go straight for following
*
* Returns:
*	int - next state
*/
int straightFollow(){
	int leftLightIn, rightLightIn;
	
	do{
		//Follow
		setMotor(LEFT, 40);
		setMotor(RIGHT, 40);

		//Get light sensor values
		leftLightIn = getLight(LEFT);
		rightLightIn = getLight(RIGHT);
	}while (rightLightIn!=WHITE && leftLightIn!=WHITE);

	//Change direction based on the lights
	if (leftLightIn==GRAY && rightLightIn==GRAY){
		return STOP_FOLLOW;
	}else if (leftLightIn==WHITE){
		return RIGHT_FOLLOW;
	}else if (rightLightIn==WHITE){
		return LEFT_FOLLOW;
	}

	return STOP_FOLLOW;
}

/*Function: leftFollow()
* Go left for following
*
* Returns:
*	int - next state
*/
int leftFollow(){
	int leftLightIn, rightLightIn;
	
	do{
		//Follow
		setMotor(LEFT, 0);
		setMotor(RIGHT, 30);

		//Get light sensor values
		leftLightIn = getLight(LEFT);
		rightLightIn = getLight(RIGHT);
	}while (rightLightIn==WHITE && leftLightIn!=WHITE);

	//Change direction based on the lights
	if (leftLightIn==GRAY && rightLightIn==GRAY){
		return STOP_FOLLOW;
	}else if (leftLightIn!=WHITE && rightLightIn!=WHITE){
		return STRAIGHT_FOLLOW;
	}else if (leftLightIn==WHITE){
		return RIGHT_FOLLOW;
	}

	return STOP_FOLLOW;
}

/*Function: rightFollow()
* Go right for following
*
* Returns:
*	int - next state
*/
int rightFollow(){
	int leftLightIn, rightLightIn;
	
	do{
		//Follow
		setMotor(LEFT, 30);
		setMotor(RIGHT, 0);

		//Get light sensor values
		leftLightIn = getLight(LEFT);
		rightLightIn = getLight(RIGHT);
	}while (rightLightIn!=WHITE && leftLightIn==WHITE);

	//Change direction based on the lights
	if (leftLightIn==GRAY && rightLightIn==GRAY){
		return STOP_FOLLOW;
	}else if (leftLightIn!=WHITE && rightLightIn!=WHITE){
		return STRAIGHT_FOLLOW;
	}else if (rightLightIn==WHITE){
		return LEFT_FOLLOW;
	}

	return STOP_FOLLOW;
}

/*Function: dispatcherFollow()
* In control of follow state machine - Dispatches calls 
* to other functions based on current state
*/
void dispatcherFollow(){
	int state = STRAIGHT_FOLLOW;
	
	while(1){	
		//Switch between states
		switch(state){
			//Initial state
			case STRAIGHT_FOLLOW:
				state = straightFollow();
				break;
			//Locate the line
			case LEFT_FOLLOW:
				state = leftFollow();
				break;
				//Locate the line
			case RIGHT_FOLLOW:
				state = rightFollow();
				break;
			case STOP_FOLLOW:
				return;
				break;

		}
	}
}

///////////////////////////////////////////////////////////////////////

////////////////////FIND LINE STATE MACHINE////////////////////////////

/*Function: straightFindLine()
* Go straight till find line
*
* Returns:
*	int - next state
*/
int straightFindLine(){
	int leftLightIn, rightLightIn;
	
	do{
		//Go forward till find the line
		setMotor(LEFT, 40);
		setMotor(RIGHT, 40);

		//Get light sensor values
		leftLightIn = getLight(LEFT);
		rightLightIn = getLight(RIGHT);
	}while (rightLightIn==WHITE || leftLightIn==WHITE);


	//Return and go to next state
	return LEFT_TURN;
}

/*Function: leftTurnFindLine()
* Turn left onto line
*
* Returns:
*	int - next state
*/
int leftTurnFindLine(){
	int leftLightIn, rightLightIn;
	int i = 0;
	do{
		//Go forward till find the line
		setMotor(LEFT, 0);
		setMotor(RIGHT, 30);

		//Get light sensor values
		leftLightIn = getLight(LEFT);
		rightLightIn = getLight(RIGHT);

		// Wait some time between iterations
		clock.wait(MAIN_WAIT);

	}while (i++<10);

	//Return and go to next state
	return TURNED;
}

/*Function: dispatcherFindLine()
* In control of find line state machine - Dispatches calls 
* to other functions based on current state
*/
void dispatcherFindLine(){
	int state = STRAIGHT;
	while(1){	
		//Switch between states
		switch(state){
			//Initial state
			case STRAIGHT:
				state = straightFindLine();
				break;
			//Locate the line
			case LEFT_TURN:
				state = leftTurnFindLine();
				break;
			case TURNED:
				return;
				break;
		}
	}
}

/////////////////////////////HIGH LEVEL STATE MACHINE///////////////////////////////

/*Function: begin()
* Intial state of state machine
*
* Returns:
*	int - next state
*/
int begin(){
	int leftLightIn, rightLightIn;

	//Get light sensor values
	leftLightIn = getLight(LEFT);
	rightLightIn = getLight(RIGHT);

	//If in white space, try to find line
	if (leftLightIn==WHITE && rightLightIn==WHITE){
		return FIND_LINE;
	}

	//If already on line, follow
	return FOLLOW;
}

/*Function: findLine()
* Function that finds the line initially
*
* Returns:
*	int - next state
*/
int findLine(){
	//Dispatcher
	dispatcherFindLine();

	//Return and go to next state
	return FOLLOW;
}

/*Function: follow()
* Function for following the line
*
* Parameters:
*	
*/
int follow(){
	//Dispatcher
	dispatcherFollow();

	return STOP;
}

/*Function: stop()
*Function that stops and does action at points
*
* Returns:
*	int - next state
*/
int stop(){
	//Infinite loop while touch not pressed
	while(getTouch()==0){
		//Spin around in circles
		setMotor(LEFT, 0);
		setMotor(RIGHT, 50);
	}

	//When pressed, return to following line
	return FOLLOW;
}

/*Function: dispatcher()
* In control of state machine - Dispatches calls 
* to other functions based on current state
*/
void dispatcherMain(){
	//Holds current state
	int state = START;

	while(1){	
		lcd.clear();

		//Switch between states
		switch(state){
			//Initial state
			case START:
				lcd.putf("sn",   "START");
				state = begin();
				break;
			//Locate the line
			case FIND_LINE:
				lcd.putf("sn",   "FIND");
				state = findLine();
				break;
			//Follow the line
			case FOLLOW:
				lcd.putf("sn",   "FOLLOW");
				state = follow();
				break;
			//Stop when come to points
			case STOP:
				lcd.putf("sn",   "STOP");
				state = stop();
				break;
		}
		int dist = sonar.getDistance();
		int bump = touch.isPressed();

		// Read the sonar and bump sensors every iteration
		lcd.putf("sn",   "NXT Sensors");
		lcd.putf("sddn", "D1/2: ", leftLight.getBrightness(),0, dist,5);
		lcd.putf("sddn",  "3/4: ", rightLight.getBrightness(),0, bump,5);

		lcd.putf("sdn", "Steering: ", steering.getCount(),0);
		
		// Update LCD display
		lcd.disp();  
		
		// Wait some time between iterations
		clock.wait(MAIN_WAIT);
	}
}

///////////////////////////////////////////////////////////////////////

/*Function: user_1ms_isr_type2()
* nxtOSEK hook to be invoked from an ISR in 
* category 2
*/
void user_1ms_isr_type2(){
	SleeperMonitor(); // needed for I2C device and Clock classes
}

/*Function: main()
* main loop
*/
TASK(TaskMain){
	// Endless loop
	while(1){
		//Call dispatcher
		dispatcherMain();
	}
}

}