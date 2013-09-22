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

#define FWD_MODE  0
#define SCAN_MODE 1
#define TURN_MODE 2
#define REV_MODE  3

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

int maxCount = 100;

/* nxtOSEK hook to be invoked from an ISR in category 2 */
void user_1ms_isr_type2(void)
{
	SleeperMonitor(); // needed for I2C device and Clock classes
}

/** 
   This function takes in a desired heading (count) and does bang-bang type
   steering control until magnitude of heading error is less than or equal to 1
**/
void turnTo(int count)
{
	// Confine "count" to be in interval [0,maxCount]
	if(count > maxCount)
	{
		count = maxCount;
	}
	if(count < 0)
	{
		count = 0;
	}

	int error = count - steering.getCount();

	// Continue steering until |error| <= 1
	while(error > 1 || error < -1)
	{
		// Steer at PWM "75" with sign same as error
		if(error < 0)
		{
			steering.setPWM(-75);
		}
		else
		{
			steering.setPWM(75);
		}
		
		// Recalculate the steering error
		error = count - steering.getCount();
		
		// Wait some time before re-evaluating
		clock.wait(10);
	}
	
	// Done turning, no more steering
	steering.setPWM(0);
}

/**
   Finds and steers the robot to the angle (count) with the largest sonar reading. Returns the 
   associated sonar reading to the caller.
**/
int findBestAngle()
{
	// First, steer to face angle 0
	turnTo(0);
	int bestDist = sonar.getDistance();
	int bestCount = 0;
	
	// Consider angles (counts) 0,25,50,75,100
	for(int i = 0; i < maxCount; i+=25)
	{
		// Get sonar and touch sensor readings at that angle
		int dist = sonar.getDistance();
		int bump = touch.isPressed();
		
		// Debugging LCD output
		lcd.clear();
		lcd.putf("sn",   "NXT Sensors");
		lcd.putf("sddn", "1/2: ", leftLight.getBrightness(),0, dist,5);
		lcd.putf("sddn",  "3/4: ", rightLight.getBrightness(),0, bump,5);

		lcd.putf("sdn", "Steering: ", steering.getCount(),0);
		lcd.putf("sn", "SCAN");
		lcd.disp();

		// Save the angle (count) with the largest distance from sonar
		turnTo(i);
		if(dist > bestDist)
		{
			bestCount = i;
			bestDist = sonar.getDistance();
		}
		
		// Wait between testing out different angles
		clock.wait(500);
	}
	
	// Turn to the angle with the largest sonar reading and return that distance
	turnTo(bestCount);
	return bestDist;
}

/**
   Backs the robot up for 1000*time time units, stops early if touch sensor pressed
**/
void backup(float time)
{
	// Turn steering to face center (straight forward)
	turnTo(CENTER);
	
	// Set both motors to full speed backwards
	leftMotor.setPWM(-FULLSPEED);
	rightMotor.setPWM(-FULLSPEED);
	
	// Back up for 1000*time time units, stop early if touch sensor pressed
	for(int i = 0; i < time*10; i++)
	{
		if(touch.isPressed() == 1)
		{
			leftMotor.setPWM(0);
			rightMotor.setPWM(0);
			break;
		}
		clock.wait(100);
	}
	
	// Stop both motors when done backing up
	leftMotor.setPWM(0);
	rightMotor.setPWM(0);
}

/**
   Main Loop
**/
TASK(TaskMain)
{
	// Start in Scan Mode
	int mode = SCAN_MODE;

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
	int revCount = 0;

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

		if(mode == SCAN_MODE) // SCAN MODE
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

		}
		
		// Update LCD display
		lcd.disp();  
		
		// Wait some time between iterations
		clock.wait(MAIN_WAIT);
	}
} // End Task

} // End Extern C