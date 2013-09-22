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

// Encoder value for the robot to be facing straight forward
#define CENTER 60

// Value for each motor to move full speed ahead
#define FULLSPEED -100

// Various operational states
#define FWD_MODE  0
#define SCAN_MODE 1
#define TURN_MODE 2
#define REV_MODE  3

// Time to wait (in milliseconds) between control updates
#define MAIN_WAIT 200

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

/* nxtOSEK hook to be invoked from an ISR in category 2 */
void user_1ms_isr_type2(void)
{
	SleeperMonitor(); // needed for I2C device and Clock classes
}

// LOCATION A: INITIALIZATION CODE

/**
   Main Loop
**/
TASK(TaskMain)
{
	// LOCATION B: LOOP INITIALIZATION CODE
	
	// Endless loop
	while(1)
	{	
		// LOCATION C: LOOP CODE
		
		// Wait some time between iterations
		clock.wait(MAIN_WAIT);
	}
} // End Task

} // End Extern C