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
#define SPEED1 40
#define SPEED2 40
// Follow State Machines
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
#define FOLLOW_BEFORE_STOP1 3
#define FOLLOW_BEFORE_STOP2 4
#define STOP  5
#define FOLLOW_AFTER_STOP 6

//Flags for Colors
#define WHITE 0
#define BLACK 1
#define GRAY  2

//Flags for when multiple sensors
#define LEFT  0
#define RIGHT 1

//Timing parameters
#define MAIN_WAIT 200
#define FAS_TIMEOUT 1000

//Transfer parameters
#define NXT_UDL_PKTHDRLEN (4)
#define NXT_UDL_GETRECORD (0xff)
#define NXT_UDL_GETHEADER (0xf0)
#define NXT_UDL_ERROR (0xaa)
#define NXT_UDL_CLOSECONN (0x00)

//Datalogging
#define RECORD_SIZE (sizeof(short)*3)
#define SAMPLE_COUNT 1000
#define RECORD_COUNT SAMPLE_COUNT

extern "C" 
{
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"
#include <string.h>

// OSEK declarations
DeclareTask(Task_ts1);
DeclareTask(Task_sampler);
DeclareResource(USB_Rx);
DeclareCounter(SysTimerCnt);

//Datalogging
void udl_loop(void *recordsp, int record_size, int record_count);
void sampler_func();
//Follow after stop state machine
int dispatcherFollowBeforeStop1();
int dispatcherFollowAfterStop();
int straightFollowWaypoint();
int leftFollowWaypoint();
int rightFollowWaypoint();
int stopFollowWaypoint();
//Follow state machine
int dispatcherFollow();
int dispatcherBeforeStop2();
int straightFollowMain();
int leftFollowMain();
int rightFollowMain();
int stopFollowMain();
//Find line state machine
int dispatcherFindLine();
int straightFindLine();
int leftTurnFindLine();
//General state machine
int dispatcherMain();
int begin();
int findLine();
int follow();
int followBeforeStop1();
int followBeforeStop2();
int stop();
int followAfterStop();

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

//Records
short records[SAMPLE_COUNT*3];
int sample = 0;


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

	//Average values from sensor
	for (int i = 0; i<5; i++){
		//Get data from light sesnor
		if (light==LEFT){
			brightness += leftLight.getBrightness();
		}else if (light==RIGHT){
			brightness += rightLight.getBrightness();
		}
	}
	brightness/=5;

	//Based on brightness, return color
	if (brightness>530){
		return WHITE;
	}else if (brightness<=400){
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
////////////////////STATE MACHINES/////////////////////////////////////
///////////////////////////////////////////////////////////////////////

//////FOLLOW BEFORE STOP 1, FOLLOW AFTER STOP STATE MACHINE///////////

/*Function: straightFollowWaypoint()
* Go straight for following
*
* Returns:
*	int - next state
*/
int straightFollowWaypoint(){
	int leftLightIn, rightLightIn;	//Sensor Input

	//Follow
	setMotor(LEFT, SPEED2);
	setMotor(RIGHT, SPEED2);

	//Get light sensor values
	leftLightIn = getLight(LEFT);
	rightLightIn = getLight(RIGHT);

	//Change direction based on the lights
	if (rightLightIn==WHITE && leftLightIn==WHITE){
		return STRAIGHT_FOLLOW;
	}else if (leftLightIn==BLACK && rightLightIn==BLACK){
		return LEFT_FOLLOW;
	}else if (rightLightIn==BLACK){
		return RIGHT_FOLLOW;
	}else if (leftLightIn==BLACK){
		return LEFT_FOLLOW;
	}

	return STRAIGHT_FOLLOW;
}

/*Function: leftFollowWaypoint()
* Go left for following
*
* Returns:
*	int - next state
*/
int leftFollowWaypoint(){
	int leftLightIn, rightLightIn;	//Sensor Input

	//Follow
	setMotor(LEFT, 0);
	setMotor(RIGHT, SPEED1);

	//Get light sensor values
	leftLightIn = getLight(LEFT);
	rightLightIn = getLight(RIGHT);

	//Change direction based on the lights
	if (rightLightIn==WHITE && leftLightIn==BLACK){
		return LEFT_FOLLOW;
	}else if (leftLightIn==BLACK && rightLightIn==BLACK){
		return LEFT_FOLLOW;
	}else if (leftLightIn==WHITE && rightLightIn==WHITE){
		return STRAIGHT_FOLLOW;
	}else if (rightLightIn==BLACK){
		return RIGHT_FOLLOW;
	}

	return LEFT_FOLLOW;
}

/*Function: rightFollowWaypoint()
* Go right for following
*
* Returns:
*	int - next state
*/
int rightFollowWaypoint(){
	int leftLightIn, rightLightIn;	//Sensor Input
	
	//Follow
	setMotor(LEFT, SPEED1);
	setMotor(RIGHT, 0);

	//Get light sensor values
	leftLightIn = getLight(LEFT);
	rightLightIn = getLight(RIGHT);
	
	if (rightLightIn==BLACK && leftLightIn==WHITE){
		return RIGHT_FOLLOW;
	}else if (leftLightIn==BLACK && rightLightIn==BLACK){
		return LEFT_FOLLOW;
	}else if (leftLightIn==WHITE && rightLightIn==WHITE){
		return STRAIGHT_FOLLOW;
	}else if (leftLightIn==BLACK){
		return LEFT_FOLLOW;
	}

	return RIGHT_FOLLOW;
}

/*Function: dispatcherFollowAfterStop()
* In control of follow Aafter stop state machine - Dispatches calls 
* to other functions based on current state
*
* Returns:
*	int - last state
*/
int dispatcherFollowBeforeStop1(){
	//States
	int state = STRAIGHT_FOLLOW;
	//Clock
	Clock time;

	time.reset();	//Initialize clock 
	while(time.now() <= FAS_TIMEOUT){	

		//Switch between states
		switch(state){
			//Initial state
			case STRAIGHT_FOLLOW:
				state = straightFollowWaypoint();
				break;
			//Locate the line
			case LEFT_FOLLOW:
				state = leftFollowWaypoint();
				break;
				//Locate the line
			case RIGHT_FOLLOW:
				state = rightFollowWaypoint();
				break;
		}

		// Wait some time between iterations
		clock.wait(MAIN_WAIT);
	}

	return state;
}

/*Function: dispatcherFollowAfterStop()
* In control of follow Aafter stop state machine - Dispatches calls 
* to other functions based on current state
*
* Returns:
*	int - last state
*/
int dispatcherFollowAfterStop(){
	//States
	int state = STRAIGHT_FOLLOW;
	//Clock
	Clock time;

	time.reset();	//Initialize clock 
	while(time.now() <= FAS_TIMEOUT){	

		//Switch between states
		switch(state){
			//Initial state
			case STRAIGHT_FOLLOW:
				state = straightFollowWaypoint();
				break;
			//Locate the line
			case LEFT_FOLLOW:
				state = leftFollowWaypoint();
				break;
				//Locate the line
			case RIGHT_FOLLOW:
				state = rightFollowWaypoint();
				break;
		}

		// Wait some time between iterations
		clock.wait(MAIN_WAIT);
	}

	return state;
}

///////////////////////////////////////////////////////////////////////

/////////////FOLLOW/FOLLOW BEFORE STOP 2 STATE MACHINE/////////////////

/*Function: straightFollowMain()
* Go straight for following
*
* Returns:
*	int - next state
*/
int straightFollowMain(){
	int leftLightIn, rightLightIn;
	
	//Follow
	setMotor(LEFT, SPEED2);
	setMotor(RIGHT, SPEED2);

	//Get light sensor values
	leftLightIn = getLight(LEFT);
	rightLightIn = getLight(RIGHT);

	//Change direction based on the lights
	if (rightLightIn==WHITE && leftLightIn==WHITE){
		return STRAIGHT_FOLLOW;
	}else if (leftLightIn==GRAY && rightLightIn==GRAY){
		return STOP_FOLLOW;
	}else if (leftLightIn!=WHITE && rightLightIn!=WHITE){
		return LEFT_FOLLOW;
	}else if (rightLightIn!=WHITE){
		return RIGHT_FOLLOW;
	}else if (leftLightIn!=WHITE){
		return LEFT_FOLLOW;
	}

	return STRAIGHT_FOLLOW;
}

/*Function: leftFollowMain()
* Go left for following
*
* Returns:
*	int - next state
*/
int leftFollowMain(){
	int leftLightIn, rightLightIn;
	
	//Follow
	setMotor(LEFT, 0);
	setMotor(RIGHT, SPEED1);

	//Get light sensor values
	leftLightIn = getLight(LEFT);
	rightLightIn = getLight(RIGHT);

	//Change direction based on the lights
	if (rightLightIn==WHITE && leftLightIn!=WHITE){
		return LEFT_FOLLOW;
	} else if (leftLightIn==GRAY && rightLightIn==GRAY){
		return STOP_FOLLOW;
	}else if (leftLightIn!=WHITE && rightLightIn!=WHITE){
		return LEFT_FOLLOW;
	}else if (leftLightIn==WHITE && rightLightIn==WHITE){
		return STRAIGHT_FOLLOW;
	}else if (rightLightIn!=WHITE){
		return RIGHT_FOLLOW;
	}

	return LEFT_FOLLOW;
}

/*Function: rightFollowMain()
* Go right for following
*
* Returns:
*	int - next state
*/
int rightFollowMain(){
	int leftLightIn, rightLightIn;
	
	//Follow
	setMotor(LEFT, SPEED1);
	setMotor(RIGHT, 0);

	//Get light sensor values
	leftLightIn = getLight(LEFT);
	rightLightIn = getLight(RIGHT);

	//Change direction based on the lights
	if (rightLightIn!=WHITE && leftLightIn==WHITE){
		return RIGHT_FOLLOW;
	}else if (leftLightIn==GRAY && rightLightIn==GRAY){
		return STOP_FOLLOW;
	}else if (leftLightIn!=WHITE && rightLightIn!=WHITE){
		return LEFT_FOLLOW;
	}else if (leftLightIn==WHITE && rightLightIn==WHITE){
		return STRAIGHT_FOLLOW;
	}else if (leftLightIn!=WHITE){
		return LEFT_FOLLOW;
	}

	return RIGHT_FOLLOW;
}

/*Function: dispatcherFollow()
* In control of follow state machine - Dispatches calls 
* to other functions based on current state
*
* Returns:
*	int - last state
*/
int dispatcherFollow(){
	int state = STRAIGHT_FOLLOW;
	
	while(1){	

		//Switch between states
		switch(state){
			//Initial state
			case STRAIGHT_FOLLOW:
				state = straightFollowMain();
				break;
			//Locate the line
			case LEFT_FOLLOW:
				state = leftFollowMain();
				break;
				//Locate the line
			case RIGHT_FOLLOW:
				state = rightFollowMain();
				break;
			case STOP_FOLLOW:
				return state;;
				break;

		}

		// Wait some time between iterations
		clock.wait(MAIN_WAIT);
	}

	return state;
}

/*Function: dispatcherFollowBeforeStop2()
* In control of follow state machine - Dispatches calls 
* to other functions based on current state
*
* Returns:
*	int - last state
*/
int dispatcherFollowBeforeStop2(){
	int state = STRAIGHT_FOLLOW;
	//Clock
	Clock time;

	time.reset();	//Initialize clock 
	while(time.now() <= FAS_TIMEOUT){	
		//Switch between states
		switch(state){
			//Initial state
			case STRAIGHT_FOLLOW:
				state = straightFollowMain();
				break;
			//Locate the line
			case LEFT_FOLLOW:
				state = leftFollowMain();
				break;
				//Locate the line
			case RIGHT_FOLLOW:
				state = rightFollowMain();
				break;
			case STOP_FOLLOW:
				return state;
				break;

		}

		// Wait some time between iterations
		clock.wait(MAIN_WAIT);
	}

	return state;

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
		setMotor(LEFT, SPEED2);
		setMotor(RIGHT, SPEED2);

		//Get light sensor values
		leftLightIn = getLight(LEFT);
		rightLightIn = getLight(RIGHT);
	}while (rightLightIn!=BLACK|| leftLightIn!=BLACK);


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
	
	//Go forward till find the line
	setMotor(LEFT, 0);
	setMotor(RIGHT, SPEED1);

	//Get light sensor values
	leftLightIn = getLight(LEFT);
	rightLightIn = getLight(RIGHT);

	// Wait some time between iterations
	clock.wait(MAIN_WAIT);

	//Return and go to next state
	return TURNED;
}

/*Function: dispatcherFindLine()
* In control of find line state machine - Dispatches calls 
* to other functions based on current state
*
* Returns:
*	int - last state
*/
int dispatcherFindLine(){
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
				return state;;
				break;
		}

		// Wait some time between iterations
		clock.wait(MAIN_WAIT);
	}

	return state;
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

	// Initial wait
	clock.wait(MAIN_WAIT*3);

	//Get light sensor values
	leftLightIn = getLight(LEFT);
	rightLightIn = getLight(RIGHT);

	//If in white space, try to find line
	if (leftLightIn!=BLACK && rightLightIn!=BLACK){
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

	return FOLLOW_BEFORE_STOP1;
}

/*Function: followAfterStop()
* Function for following the line after a stop
*
* Parameters:
*	
*/
int followBeforeStop1(){
	//Dispatcher
	dispatcherFollowBeforeStop1();

	return FOLLOW_BEFORE_STOP2;
}

/*Function: followAfterStop()
* Function for following the line after a stop
*
* Parameters:
*	
*/
int followBeforeStop2(){
	int state; 

	//Dispatcher
	state = dispatcherFollowBeforeStop2();

	//IF there is a time out, just go back to regular following
	if (state == STOP_FOLLOW){
		return STOP;
	} else{
		return FOLLOW;
	}
}

/*Function: stop()
*Function that stops and does action at points
*
* Returns:
*	int - next state
*/
int stop(){
	udl_loop(records, RECORD_SIZE, sample);
	//Infinite loop while touch not pressed
	while(getTouch()==0){
		//Spin around in circles
		setMotor(LEFT, 0);
		setMotor(RIGHT, SPEED1);
	}

	//When pressed, return to following line
	return FOLLOW_AFTER_STOP;
}

/*Function: followAfterStop()
* Function for following the line after a stop
*
* Parameters:
*	
*/
int followAfterStop(){
	//Dispatcher
	dispatcherFollowAfterStop();

	return FOLLOW;
}

/*Function: dispatcher()
* In control of state machine - Dispatches calls 
* to other functions based on current state
*
* Returns:
*	int - last state
*/
int dispatcherMain(){
	//Holds current state
	int state = START;

	while(1){	
		lcd.clear();

		//Switch between states
		switch(state){
			//Initial state
			case START:
				lcd.putf("sn",   "START");
				lcd.disp();
				state = begin();
				break;
			//Locate the line
			case FIND_LINE:
				lcd.putf("sn",   "FIND LINE");
				lcd.disp(); 
				state = findLine();
				break;
			//Follow the line
			case FOLLOW:
				lcd.putf("sn",   "FOLLOW");
				lcd.disp();
				state = follow();
				break;
			//Follow the line
			case FOLLOW_BEFORE_STOP1:
				lcd.putf("sn",   "FOLLOW BEFORE 1");
				lcd.disp();
				state = followBeforeStop1();
				break;
				//Follow the line
			case FOLLOW_BEFORE_STOP2:
				lcd.putf("sn",   "FOLLOW BEFORE 2");
				lcd.disp();
				state = followBeforeStop2();
				break;
			//Stop when come to points
			case STOP:
				lcd.putf("sn",   "STOP");
				lcd.disp();
				state = stop();
				break;
			//Stop when come to points
			case FOLLOW_AFTER_STOP:
				lcd.putf("sn",   "FOLLOW AFTER STOP");
				lcd.disp();
				state = followAfterStop();
				break;
		}		
		
		// Wait some time between iterations
		clock.wait(MAIN_WAIT);
	}

	return state;
}

///////////////////////////////////////////////////////////////////////

//////////////////////////LOGGING///////////////////////////////////////////
/*Function: user_1ms_isr_type2()
* nxtOSEK hook to be invoked from an ISR in 
* category 2
*/
void user_1ms_isr_type2(){
	// Increment System Timer Count to activate periodical Tasks
	(void)SignalCounter(SysTimerCnt);

	SleeperMonitor(); // needed for I2C device and Clock classes
}

/* ECRobot hooks */
void ecrobot_device_initialize()
{
	ecrobot_init_usb(); 	// init USB
}

void ecrobot_device_terminate()
{
	ecrobot_term_usb(); 	// terminate USB
}

/*Function: Task_ts1()
* USB Process Handler
*/
TASK(Task_ts1){
	
	//USB process handler
	GetResource(USB_Rx);
	ecrobot_process1ms_usb(); 
	ReleaseResource(USB_Rx);

	TerminateTask();		//Terminate task
}


/*Function: Task_sampler()
* Performs sampling in background
*/
TASK(Task_sampler){
	sampler_func();		//Sample
	TerminateTask();	//Terminate task
}

/*Function: sampler_func()
* Actually records data
*/
void sampler_func(){

	//log the sample
	records[sample*3+0] = ecrobot_get_systick_ms();
	records[sample*3+1] = leftMotor.getCount();
	records[sample*3+2] = rightMotor.getCount();

	//Increment number of samples
	sample++;
	sample = sample%1000;
}

/*Function: udl_loop()
* Actually records data
*
* Parameters:
*	void *recordsp   - buffer of data 
*	int record_size  - size of data
*	int record_count - number of data
*/
void udl_loop(void *recordsp, int record_size, int record_count)
{
	unsigned char buffer[MAX_USB_DATA_LEN] __attribute__ ((aligned (sizeof(short)))) ;
	int len;
	
	unsigned char command;
	unsigned short index;
		
	while(sample!=0){		
		
		GetResource(USB_Rx);
		//read USB data
		len = ecrobot_read_usb(buffer, 0, MAX_USB_DATA_LEN);
		ReleaseResource(USB_Rx);
		
		if(len > 3)
		{
			command = buffer[0];
			index = ((unsigned short*)buffer)[1];
						
			switch(command)
			{
				case NXT_UDL_CLOSECONN:
					//nothing needs to be changed
					//just zero out other fields and send it back
					//and close the connection
					buffer[1] = 0;
					((unsigned short*)buffer)[1] = 0;
					ecrobot_send_usb(buffer, 0, NXT_UDL_PKTHDRLEN);
					ecrobot_disconnect_usb();
					
					sample = 0;	//Reset sample counter
					
					break;
					
				case NXT_UDL_GETHEADER:
					//Send header
					buffer[1] = (unsigned char)record_size;
					((unsigned short*)buffer)[1] 
						= (unsigned short)record_count;
					ecrobot_send_usb(buffer, 0, NXT_UDL_PKTHDRLEN);
					
					break;
					
				case NXT_UDL_GETRECORD:
					//Send data
					if(index < record_count)
					{
						//the command and index will be echoed back
						
						buffer[1] = (unsigned char)record_size;
						
						memcpy(	&(buffer[NXT_UDL_PKTHDRLEN]), 
								((short *)recordsp+(index*record_size)),
								record_size);
								
						ecrobot_send_usb(buffer, 0, 
							(NXT_UDL_PKTHDRLEN+record_size));
						

						break;
					}
	
				default:
					buffer[0] = NXT_UDL_ERROR;
					ecrobot_send_usb(buffer, 0, NXT_UDL_PKTHDRLEN);
			}
		}	
	}	
}

///////////////////////////////////////////MAIN//////////////////////////////////////

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
