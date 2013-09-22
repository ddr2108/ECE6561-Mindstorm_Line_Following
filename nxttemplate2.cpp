/* nxttemplate.cpp */ 
// This is a template file to be used with the Lego NXT Robot tutorials.  
// Before usage, be sure to rename "nxttemplate.cpp", "nxttemplate.oil",
// as well as all instances of "nxttemplate" within the "Makefile" file 
// with the name of the new project.

// ECRobot++ API (http://lejos-osek.sourceforge.net/html/index.html)
#include "LightSensor.h"
#include "SonarSensor.h"
#include "SoundSensor.h"
#include "TouchSensor.h"
#include "Clock.h"
#include "Lcd.h"
#include "Motor.h"
#include "Nxt.h"

using namespace ecrobot;


//Flags for when multiple sensors
#define LEFT  0
#define RIGHT 1

// Encoder value for the robot to be facing straight forward
#define CENTER 60

// Value for each motor to move full speed ahead
#define FULLSPEED -100

// Various operational states
#define START  0
#define FIND_LINE 1
#define FOLLOW 2
#define STOP  3

// Time to wait (in milliseconds) between control updates
#define MAIN_WAIT 1000

extern "C" 
{
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"

// Initialize Sensors
LightSensor  leftLight(PORT_1, true); // init light sensor ON
LightSensor  rightLight(PORT_3, true);
SonarSensor  sonar(PORT_2);
TouchSensor  touch(PORT_4);

// Initialize Actuators
Motor steering(PORT_A);
Motor rightMotor(PORT_B);
Motor leftMotor(PORT_C);

// Initialize Clock
Clock clock;

// Initialize LCD
Lcd lcd;		

void dispatcher();
int start();
int findLine();
int follow();
int stop();
int getLight(int light);
int getSonar();
int getTouch();
void setMotor(int motor, int PWM);

/* nxtOSEK hook to be invoked from an ISR in category 2 */
void user_1ms_isr_type2(void)
{
	SleeperMonitor(); // needed for I2C device and Clock classes
}

// LOCATION A: INITIALIZATION CODE

/**
   Main Loop
**/
/*TASK(TaskMain)
{
	// LOCATION B: LOOP INITIALIZATION CODE
	
	// Endless loop
	while(1){	
		// Call Dispatcher
		lcd.clear();
		lcd.putf("sn",   "NXT Sensors");
		lcd.putf("sn", "SCAN");
		lcd.disp();
		//dispatcher();
		// Wait some time between iterations
		clock.wait(MAIN_WAIT);

	}
}*/
	TASK(TaskMain)
{
	// Start in Scan Mode
	/*int mode = SCAN_MODE;

	// Steering count initialization
	// First, turn all the way to the left
	steering.setPWM(-50);
	clock.wait(1000);
	
	// Set encoder count to 0 at far left
	steering.setPWM(0);
	steering.setCount(0);
	
	// now turn all the way to the right
	steering.setPWM(50);
	clock.wait(1000);
	
	// Get the encoder count and save it as maxCount
	maxCount = steering.getCount();
	
	// Turn to face the center
	turnTo(CENTER);
	int turnCount = 0;
	int revCount = 0;*/

	// Endless loop
	while(1)
	{
		// Read the sonar and bump sensors every iteration
		int dist = sonar.getDistance();
		int bump = touch.isPressed();
		lcd.clear();
		lcd.putf("sn",   "NXT Sensors");
		lcd.putf("sddn", "1/2: ", leftLight.getBrightness(),0, dist,5);
		lcd.putf("sddn",  "3/4: ", rightLight.getBrightness(),0, bump,5);

		lcd.putf("sdn", "Steering: ", steering.getCount(),0);

		/*if(mode == SCAN_MODE) // SCAN MODE
		{
			// Dont move and find the best angle
			leftMotor.setPWM(0);
			rightMotor.setPWM(0);
			dist = findBestAngle();
			
			// If there is more than 30 free space in best angle, switch to TURN mode
			if(dist > 30)
			{
				mode = TURN_MODE;
				lcd.putf("sn","SCAN: TURN");
			}
			else // Otherwise, switch to REV mode
			{
				mode = REV_MODE;
				lcd.putf("sn","SCAN: REV");
			}
		}
		else if(mode == TURN_MODE) // TURN MODE
		{
			if(dist <= 30)
			{
				// If sensed distance too close, stop turning and switch to REV mode
				turnCount = 0;
				mode = REV_MODE;
				lcd.putf("sn","TURN: REV");
			}
			else if(turnCount > 1000/MAIN_WAIT)
			{
				// If turned for too long (1 second), stop turning and switch to FWD mode
				turnCount = 0;
				mode = FWD_MODE;
				lcd.putf("sn","TURN: FWD");
			}
			else
			{
				// Use Proportional controller to steer left and right motors differently until
				// steering lines up in center direction.
				turnCount++;
				int diff = (steering.getCount() - CENTER)/4;
				leftMotor.setPWM(FULLSPEED-diff);
				rightMotor.setPWM(FULLSPEED+diff);
				lcd.putf("sn","TURN: TURN");
			}
		}
		else if(mode == REV_MODE) // REV MODE
		{
			if(touch.isPressed() == 1)
			{
				// If bumper is pressed, switch to SCAN mode
				mode = SCAN_MODE;
				lcd.putf("sn","REV: BUMP");
			}
			else if(revCount > 1000/MAIN_WAIT)
			{
				// If reversed for long enough, stop and switch to SCAN mode
				revCount = 0;
				leftMotor.setPWM(0);
				rightMotor.setPWM(0);
				mode = SCAN_MODE;
				lcd.putf("sn","REV: SCAN");
			}
			else
			{
				// Go straight back at full speed
				turnTo(CENTER);
				leftMotor.setPWM(-FULLSPEED);
				rightMotor.setPWM(-FULLSPEED);
				revCount++;
				lcd.putf("sn","REV: REV");
			}
		}
		else if(mode == FWD_MODE) // FWD MODE
		{
			if(dist < 30)
			{
				// Stop and scan if less than 30 in front of you
				leftMotor.setPWM(0);
				rightMotor.setPWM(0);
				mode = SCAN_MODE;
				lcd.putf("sn","FWD: SCAN");
			}
			else if(dist < 50)
			{
				// Slow down if less than 50 in front of you
				turnTo(CENTER);
				int speed = FULLSPEED*dist/50;
				leftMotor.setPWM(speed);
				rightMotor.setPWM(speed);
				lcd.putf("sn", "FWD: SLOW");
			}
			else
			{
				// Go forward at full speed
				turnTo(CENTER);
				leftMotor.setPWM(FULLSPEED);
				rightMotor.setPWM(FULLSPEED);
				lcd.putf("sn","FWD: FULL");
			}

		}*/
		
		// Update LCD display
		lcd.disp();  
		
		// Wait some time between iterations
		clock.wait(MAIN_WAIT);
	}
}

/*Function: dispatcher()
* In control of state machine - Dispatches calls 
* to other functions based on current state
*/
void dispatcher(){
	//Holds current state
	int state = START;

	while(1){	
		//Switch between states
		switch(state){
			//Initial state
			case START:
				state = start();
				break;
			//Locate the line
			case FIND_LINE:
				state = findLine();
				break;
			//Follow the line
			case FOLLOW:
				state = follow();
				break;
			//Stop when come to points
			case STOP:
				state = stop();
				break;
		}

		// Wait some time between iterations
		clock.wait(MAIN_WAIT);
	}
}

////////////////////States/////////////////////////////////////

/*Function: start()
* Intial state of state machine
*
* Returns:
*	int - next state
*/
int start(){
	int leftLightIn, rightLightIn;

	//Get light sensor values
	leftLightIn = getLight(LEFT);
	rightLightIn = getLight(RIGHT);

	if (1){
		return FIND_LINE;
	}

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
			setMotor(LEFT, 50);
			setMotor(RIGHT, 50);
		}

		//Get light sensor values
		leftLightIn = getLight(LEFT);
		rightLightIn = getLight(RIGHT);
	}while (rightLightIn>300);

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

////////////////Interacting with IO///////////////////////////////
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

} // End Extern C
