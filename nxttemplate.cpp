/* nxtdemo.cpp */ 
// modified from nxtgt.c by Paul Robinette probinette3@gatech.edu

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

// Various operational states
#define START  0
#define FIND_LINE 1
#define FOLLOW 2
#define STOP  3

#define MAIN_WAIT 200

//Flags for when multiple sensors
#define LEFT  0
#define RIGHT 1

extern "C" 
{
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"


void dispatcher();
int begin();
int findLine();
int follow();
int stop();
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


////////////////Interacting with IO//////////////////////////////////
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
	if (light==LEFT){
		return leftLight.getBrightness();
	}else if (light==RIGHT){
		return rightLight.getBrightness();
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

////////////////////States/////////////////////////////////////////////

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

	if (leftLightIn > 500 && rightLightIn > 500){
		return FIND_LINE;
	}

	//Return and go to next state
	return STOP;
}

/*Function: findLine()
* Function that finds the line initially
*
* Returns:
*	int - next state
*/
int findLine(){
	int leftLightIn, rightLightIn;
	do{
		//Go forward till find the line
		if (1){
			setMotor(LEFT, 20);
			setMotor(RIGHT, 20);
		}

		//Get light sensor values
		leftLightIn = getLight(LEFT);
		rightLightIn = getLight(RIGHT);
	}while (rightLightIn>500 || leftLightIn > 500);

	//Return and go to next state
	return STOP;
}

/*Function: follow()
* Function for following the line
*
* Parameters:
*	
*/
int follow(){
	return FOLLOW;
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
///////////////////////////////////////////////////////////////////////
/*Function: user_1ms_isr_type2()
* nxtOSEK hook to be invoked from an ISR in 
* category 2
*/
void user_1ms_isr_type2()
{
	SleeperMonitor(); // needed for I2C device and Clock classes
}

/*Function: dispatcher()
* In control of state machine - Dispatches calls 
* to other functions based on current state
*/
void dispatcher(){
	//Holds current state
	int state = START;

	while(1){	
		lcd.clear();

		//Switch between states
		switch(state){
			//Initial state
			case START:
				state = begin();
				lcd.putf("sn",   "START");
				break;
			//Locate the line
			case FIND_LINE:
				state = findLine();
				lcd.putf("sn",   "FIND");
				break;
			//Follow the line
			case FOLLOW:
				state = follow();
				lcd.putf("sn",   "FOLLOW");
				break;
			//Stop when come to points
			case STOP:
				state = stop();
				lcd.putf("sn",   "STOP");
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

		// Wait some time between iterations
		clock.wait(MAIN_WAIT);
	}
}

/*Function: main()
* main loop
*/
TASK(TaskMain)
{
	// Endless loop
	while(1){
		//Call dispatcher
		dispatcher();
	}
} // End Task

} // End Extern C