#pragma config(Motor,  port2,           ClivesLeftMotor, tmotorServoContinuousRotation, openLoop)
#pragma config(Motor,  port3,           SzalaysArm,    tmotorServoContinuousRotation, openLoop)
#pragma config(Motor,  port5,           RemisGripper,  tmotorServoStandard, openLoop)
#pragma config(Motor,  port6,           HumbsGripper,  tmotorServoStandard, openLoop)
#pragma config(Motor,  port9,           AidanRightMotor, tmotorServoContinuousRotation, openLoop, reversed)
#define Stop 0

task main(){
// Declare variables to track button presses
bool Btn5UWasPressed = false;
bool Btn5DWasPressed = false;


// Declare variable to store servo position
int ServoValueright = 105;

// Declare variables to store the power that the arm raises up and down
int down = -127;
int up = 127;

// Declare variable to store the percentage of speed the wheels move at
float bower = 1.00;

while(1){
	// Button 8D: Slow mode
	if(vexRT[Btn8D]){
		bower = .30; // Reduce motor power
		down = -33; // Adjust arm movement speed
		up = 83; // Adjust arm movement speed
		}

	// Button 8R: Normal mode
	if(vexRT[Btn8R]){
		bower = 1.00; // Restore full motor power
		down = -127; // Restore arm movement speed
		up = 127; // Restore arm movement speed
		}

	// Control drive motors based on joystick input
	motor[ClivesLeftMotor] = vexRT[Ch3] * bower;
	motor[AidanRightMotor] = vexRT[Ch2] * bower;


  // Control arm motor based on button presses
	if(vexRT[Btn5D]){
		motor[SzalaysArm] = down; // Move arm down
	} else if(vexRT[Btn5U]){
		motor[SzalaysArm] = up; // Move arm up
	} else{
		motor[SzalaysArm] = Stop; // stop arm
		}

		// Control right servo based on button presses and debouncing
		if(vexRT[Btn6U] && !Btn5UWasPressed)
		{
			ServoValueright = ServoValueright + 35; // Increase servo position
			Btn5UWasPressed = true; // Prevent multiple triggers
		}

		else if (!vexRT[Btn6U]){
			Btn5UWasPressed = false;
}

		if(vexRT[Btn6D] && !Btn5DWasPressed)
		{

			ServoValueright = ServoValueright - 35; // Decrease servo position
			Btn5DWasPressed = true; // Prevent multiple triggers
		}

		else if(!vexRT[Btn6D]){
			Btn5DWasPressed = false;
}

		// Limit servo position
		if(ServoValueright > 105){
			ServoValueright = 105;
			}
			if(ServoValueright < -140){
			ServoValueright = -140;
			}

		// Set servo positions
		motor[port5] = ServoValueright;
		motor[port6] = ServoValueright;

		// Delay to reduce allow for smooth control
		wait1Msec(10);

}


}
