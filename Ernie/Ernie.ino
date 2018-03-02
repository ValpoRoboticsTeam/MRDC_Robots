/*
  __/\\\\\\\\\\\\\\\______________________________________________________
   _\/\\\///////////_______________________________________________________
   _\/\\\_____________________________________________/\\\_________________
    _\/\\\\\\\\\\\_______/\\/\\\\\\\____/\\/\\\\\\____\///_______/\\\\\\\\__
     _\/\\\///////_______\/\\\/////\\\__\/\\\////\\\____/\\\____/\\\/////\\\_
      _\/\\\______________\/\\\___\///___\/\\\__\//\\\__\/\\\___/\\\\\\\\\\\__
       _\/\\\______________\/\\\__________\/\\\___\/\\\__\/\\\__\//\\///////___
        _\/\\\\\\\\\\\\\\\__\/\\\__________\/\\\___\/\\\__\/\\\___\//\\\\\\\\\\_
         _\///////////////___\///___________\///____\///___\///_____\//////////__

      \WWWWWWW/
    _/`  o_o  `\_
   (_    (_)    _)
     \ '-...-' /
     (`'-----'`)
      `"""""""`

  Ernie Base Drive Code

  /////Revision History/////
  
  ///2.7
   -Removing peripheral code from 2016 season
   -Rough Draft of peripheral code for 2017 season. Control scheme has not been implemented yet.
    This is just assigning motors to buttons for testing.

  ///2.6
   Some changes made at competition. These still require documentation.

  ///2.5
   Added wrist servo in setup loop... needs to be implemented in arm function
   Need to add and implement victors for linear actuator and tiny arm motor in code.
   General cleanup including:
   Added definition of initial handicap, changed Handicap int to be equal to this define.
   Removed code associeated with the non-existant arm position switch.


  ///2.4
   Removed unneeded time delay code in order to make things look nicer.
   Added macros to enable or disable different robot functions.
   Drive imputs moved to their own function.
   Controller LEDs now show what drive mode is currently in use.

  ///2.3
   Holding down L1 reverses the controls for the left joysitck, making the robot easier to drive while it is facing you.
   Added the ability to measure voltage. *Disabled for the time being
   Drive scheme now has its own function.
   When ajusting the motor correction values, the controller will not vibrate when returning to the "center" value.

  ///2.2
   Updated code to support 4 motors.
   Added ability to remove speed governor. Also reduced turning rate when governor is removed.

  ///2.1
   Stop function now uses writeMicroseconds so the stop function is no longer dependent on motor handicap values.
   Added abiltiy to check the status of the controller battery.
  //////////////////////////

  Parts List
  ---------------------------------
  Uses USBHostShield 2.0
  Arduino Mega ADK
  USB Bluetooth Dongle - some models not supported
  PS3 Controller - name brand recommended
  ---------------------------------

*/

#include <stdio.h>
#include <PS3BT.h>
#include <usbhub.h> //these are both libraries used by the PS3 communication

#ifndef DEBUG_USB_HOST //this should enable additional information to be viewed in the serial monitor
#define DEBUG_USB_HOST
#endif

#include <Servo.h>  //used to interface with victors. included here so compiler knows to have it for the drivetrain library
#include <math.h>

//----------------------------------------------------------------------
//////////////////////////////////////////////////////////////
//////////////////Enable Robot Functions//////////////////////
//////////////////////////////////////////////////////////////
//Comment out a line to disable that function.

//#define DEBUG
#define ENABLE_DRIVE
#define PERIPHERALS  //2017 PERIPHERALS
//#define ENABLE_GOVERNOR_CHANGE //comment out this line to lock the governor   <--- investigate what this governor is, rename 


//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Pin numbers for motors and servos set here.
#define LF_MOTOR 10
#define RF_MOTOR 8
#define LR_MOTOR 11
#define RR_MOTOR 9

#ifdef PERIPHERALS
#define LIFTER_MOTOR     2 //we need a better name
#define BALL_ROLLER      3
#define BALL_ACTUATOR    4
#define KEY_ROLLER       5
#define GUTTER_MOTOR     6
#endif
//----------------------------------------------------------------------

//If one of the motors is spinning when stopped, adjust it's offset here
#define LEFT_ADJUST 0
#define RIGHT_ADJUST 0

//Ignore this part, it's just PS3 magic we copied and pasted...
//------------------------------------------------------------
int newconnect = 0;
USB Usb;
USBHub Hub1(&Usb);
BTD Btd(&Usb);
PS3BT PS3(&Btd);
//--------------------------------------------------------------

//----------------------------------------------------------------------
//motor objects
Servo lfmotor;
Servo rfmotor;
Servo lrmotor;
Servo rrmotor;

#ifdef PERIPHERALS
//servo objects
Servo lifterMotor;
Servo ballRoller;
Servo ballActuator;
Servo keyRoller;
Servo gutterMotor;

#define LIFTER_UP_VALUE  180
#define LIFTER_DOWN_VALUE 0

#define BALL_ROLLER_IN_VALUE 180
#define BALL_ROLLER_OUT_VALUE 0

#define BALL_ACTUATOR_UP_VALUE 180
#define BALL_ACTUATOR_DOWN_VALUE 0

#define KEY_ROLLER_IN_VALUE 105
#define KEY_ROLLER_OUT_VALUE 75

#define GUTTER_UP_VALUE 110
#define GUTTER_DOWN_VALUE 80
#define GUTTER_STOP_VALUE 100
#define GUTTER_HANDICAP_VALUE 4

int triggerInput = 0;
int triggerValue = 0;

#define PRIMARY   0 
#define SECONDARY 1

int peripheralState = PRIMARY;

#endif
//----------------------------------------------------------------------

//----------------------------------------------------------------------
#define INITIAL_HANDICAP 4

#define TANK 0
#define ARCADE 1

#define FORWARD   1
#define BACKWARD -1

int driveDirection = FORWARD;
int driveMode = TANK; // 0 for yx control, 1 for yy control
int arcadeDrive = 0; //Initial speed before turning calculations
int arcadeTurn = 0; //Turn is adjustment to drive for each motor separately to create turns
int leftYinput = 0; //Tank Drive input variables
int rightYinput = 0;
int rightXinput = 0;
int leftTankDrive  = 0;
int rightTankDrive = 0;
int handicap = INITIAL_HANDICAP; //speed limited mode
int motorCorrect = 0;
int turnhandicap = 1; //This value gets changed by the governor function to reduce trun speed when speed governor is removed

//----------------------------------------------------------------------


void setup() {
  // put your setup code here, to run once:
  lfmotor.attach(LF_MOTOR);
  rfmotor.attach(RF_MOTOR);
  lrmotor.attach(LR_MOTOR);
  rrmotor.attach(RR_MOTOR);

#ifdef PERIPHERALS
  lifterMotor.attach(LIFTER_MOTOR);
  lifterMotor.writeMicroseconds(1500);
  ballRoller.attach(BALL_ROLLER);
  ballRoller.writeMicroseconds(1500);
  ballActuator.attach(BALL_ACTUATOR);
  ballActuator.writeMicroseconds(1500);
  keyRoller.attach(KEY_ROLLER);
  keyRoller.writeMicroseconds(1500);
  gutterMotor.attach(GUTTER_MOTOR);
  gutterMotor.writeMicroseconds(1500);
#endif
  //Initialize the USB port, with an error catch. New Programmers can ignore this section
  //-------------------------------------------------------------------------------------
  Serial.begin(115200);
  if (Usb.Init() == -1)
  { // this is for an error message with USB connections
    Serial.print(F("\r\nOSC did not start"));
    while (1);
  }
  Serial.print(F("\r\nPS3 Bluetooth Library Started"));
  //-------------------------------------------------------------------------------------
}

void loop() {
  // put your main code here, to run repeatedly:
  Usb.Task(); //We think this updates the USB buffers     "USB main task. Performs enumeration/cleanup" ~matt

  if (PS3.PS3Connected)
  {
    if (newconnect == 0)
    {
      newconnect++;
      //Serial.println("\nRumble is on!");
      //PS3.moveSetRumble(64); **********************************NOT necessary unless using PS3 move controller.
      PS3.setRumbleOn(100, 255, 100, 255); //VIBRATE!!!
      PS3.printStatusString();
    }

    //if (PS3.getButtonClick(SELECT)) battStat(); //Get controller battery status.
    if (PS3.getButtonClick(PS)) { //E-STOP: Terminate connection to controller.
      newconnect = 0;
      PS3.disconnect();
    }

#ifdef ENABLE_DRIVE
    driveInputs();
    drive();
#endif

#ifdef PERIPHERALS
    peripherals();
#endif
  }

  else stop();//your daily dose of terminator prevention
}




//////////////////////////////////////////////////////////////
////////////////////////Functions/////////////////////////////
//////////////////////////////////////////////////////////////

void driveInputs() {
  /*
    PS3.getAnalogHat() gets the input from the Left or Right joystick in the X or Y direction in the form RightHatX, LeftHatY, etc.
    map() takes a value, the range of that value, and then the desired range of the value, and maths it to convert it to the range you want
    So we map the controller input from it's original range of 0-255 to a range of -90-90.
    Victors use a range of 0-180. Reflecting this range over 0 makes some of the math to come simpler.
  */
  leftYinput  = map(PS3.getAnalogHat(LeftHatY),  0, 255, -84, 84); //left joystick y-axis
  rightYinput = map(PS3.getAnalogHat(RightHatY), 0, 255, -84, 84); //right joystick y-axis
  rightXinput = map(PS3.getAnalogHat(RightHatX), 0, 255, -90, 90); //x-axis

  /*
     Joysticks have a "sticky" area around the middle of the joystick - this means they never go back
     to true zero. So we have to check and see if the joystick is "close enough", in which case we say
     it is zero.
  */
  if (abs(leftYinput)  < 10) leftYinput = 0;
  if (abs(rightYinput) < 10) rightYinput = 0;
  if (abs(rightXinput) < 10) rightXinput = 0;
}

void drive()
{
  //Instead of following some sort of equation to slow down acceleration
  //We just increment the speed by one towards the desired speed.
  //The acceleration is then slowed because of the loop cycle time
  if(PS3.getButtonPress(RIGHT))
  {
    if(PS3.getButtonClick(R3))
    {
      if(TANK == driveMode)
      {
        driveMode = ARCADE;
        PS3.setRumbleOn(10,255,10,255);
        arcadeDrive = arcadeTurn = leftTankDrive = rightTankDrive = 0;
      }
      else
      {
        driveMode = TANK;
        PS3.setRumbleOn(10,255,10,255);
        arcadeDrive = arcadeTurn = leftTankDrive = rightTankDrive = 0;
      }
    }
  }

  if(PS3.getButtonClick(SELECT))
  {
    if(FORWARD == driveDirection)
    {
      driveDirection = BACKWARD;
      PS3.setRumbleOn(10,255,0,255);
    }
    else
    {
      driveDirection = FORWARD;
      PS3.setRumbleOn(0,255,10,255);
    }
  }

  if(ARCADE == driveMode)
  {
    if (arcadeDrive < leftYinput)arcadeDrive++; //Accelerates
    else if (arcadeDrive > leftYinput) arcadeDrive--; //Decelerates

    if (arcadeTurn < rightXinput) arcadeTurn++;
    else if (arcadeTurn > rightXinput) arcadeTurn--;

    int ThrottleL = ((arcadeDrive - (arcadeTurn / turnhandicap)) / handicap) + LEFT_ADJUST; //This is the final variable that decides motor speed.
    int ThrottleR = ((arcadeDrive + (arcadeTurn / turnhandicap)) / handicap) + RIGHT_ADJUST;
    if (BACKWARD == driveDirection) 
    { // This will flip the direction of the left stick to allow easier driving in reverse.
      ThrottleL = ((arcadeDrive + (arcadeTurn / turnhandicap)) / handicap) + LEFT_ADJUST;
      ThrottleR = ((arcadeDrive - (arcadeTurn / turnhandicap)) / handicap) + RIGHT_ADJUST;
      ThrottleL = -ThrottleL;
      ThrottleR = -ThrottleR;
    }
    if (ThrottleL > 90) ThrottleL = 90;
    if (ThrottleR > 90) ThrottleR = 90;
  
    int left = (-ThrottleL + 90 + motorCorrect);
    int right = (ThrottleR + 90 + motorCorrect);
    lfmotor.write(left); //Sending values to the speed controllers
    rfmotor.write(right);
    lrmotor.write(left); //Send values to the rear speed controllers.
    rrmotor.write(right);
  }
  else // TANK DRIVE
  {
    if(leftTankDrive < leftYinput)      leftTankDrive++;
    else if(leftTankDrive > leftYinput) leftTankDrive--;
    

    if(rightTankDrive < rightYinput)      rightTankDrive++;
    else if(rightTankDrive > rightYinput) rightTankDrive--;
/*
    if(FORWARD == driveDirection)
    {
    */
      lfmotor.write(-leftTankDrive +90); //Sending values to the speed controllers
      rfmotor.write(rightTankDrive+90);
      lrmotor.write(-leftTankDrive +90); //Send values to the rear speed controllers.
      rrmotor.write(rightTankDrive+90);
      /*
    }
    else
    {
      rfmotor.write(-leftTankDrive +90); //Sending values to the speed controllers
      lfmotor.write(rightTankDrive+90);
      rrmotor.write(-leftTankDrive +90); //Send values to the rear speed controllers.
      lrmotor.write(rightTankDrive+90);
    }

    */
  }  
}

#ifdef PERIPHERALS
void peripherals()
{
  // for right now, this is just a place to dump control of all the peripherals. When we develop the actual control scheme
  // we will actually split this into and inputs function and a peripherals function. for right now we just want to 
  // prove that all the parts move

  if(PS3.getButtonClick(START)) {
    if (PRIMARY == peripheralState) 
    {
      peripheralState = SECONDARY;          //primary/secondary mode control
      PS3.setLedRaw(9);
    }
    else // state is SECONDARY 
    {
      peripheralState = PRIMARY; 
      PS3.setLedRaw(1);
    }
  }

  // lifterMotor controls     ***draw bridge***
  if(PS3.getButtonPress(UP)) lifterMotor.write(LIFTER_UP_VALUE);
  else if(PS3.getButtonPress(DOWN)) lifterMotor.write(LIFTER_DOWN_VALUE);
  else lifterMotor.writeMicroseconds(1500);

  // primary/secondary roller controls
  if (PRIMARY == peripheralState)
  {
    if(PS3.getButtonPress(R1)) keyRoller.write(KEY_ROLLER_IN_VALUE);
    else if(PS3.getButtonPress(L1)) keyRoller.write(KEY_ROLLER_OUT_VALUE);
    else keyRoller.writeMicroseconds(1500);

    if(PS3.getButtonPress(SQUARE)) ballRoller.write(BALL_ROLLER_IN_VALUE);
    else if(PS3.getButtonPress(CIRCLE)) ballRoller.write(BALL_ROLLER_OUT_VALUE);
    else ballRoller.writeMicroseconds(1500);
  }
  else
  {
    if(PS3.getButtonPress(SQUARE)) keyRoller.write(KEY_ROLLER_IN_VALUE);
    else if(PS3.getButtonPress(CIRCLE)) keyRoller.write(KEY_ROLLER_OUT_VALUE);
    else keyRoller.writeMicroseconds(1500);

    if(PS3.getButtonPress(R1)) ballRoller.write(BALL_ROLLER_IN_VALUE);
    else if(PS3.getButtonPress(L1)) ballRoller.write(BALL_ROLLER_OUT_VALUE);
    else ballRoller.writeMicroseconds(1500);
  }

  int inputL2 = map(PS3.getAnalogButton(R2), 0, 255, 0, -90);
  int inputR2 = map(PS3.getAnalogButton(L2), 0, 255, 0,  90);

  if(abs(inputL2) > abs(inputR2)) triggerInput = inputL2;
  else triggerInput = inputR2;
  if(abs(triggerInput) < 10) triggerInput = 0;

  if(triggerInput > triggerValue) triggerValue++;
  else if(triggerInput < triggerValue) triggerValue--;
  
  #ifdef DEBUG
    Serial.println(triggerValue);
  #endif
  if(PRIMARY == peripheralState)
  {
    if(!triggerValue) gutterMotor.write(GUTTER_STOP_VALUE);
    else gutterMotor.write((triggerValue/GUTTER_HANDICAP_VALUE)+90);

    if(PS3.getButtonPress(TRIANGLE)) ballActuator.write(BALL_ACTUATOR_UP_VALUE);
    else if(PS3.getButtonPress(CROSS)) ballActuator.write(BALL_ACTUATOR_DOWN_VALUE);
    else ballActuator.writeMicroseconds(1500);    
  }
  else
  {
    if(!triggerValue) ballActuator.writeMicroseconds(1500);
    else ballActuator.write(triggerValue+90);
    
    if(PS3.getButtonPress(TRIANGLE)) gutterMotor.write(GUTTER_UP_VALUE);
    else if(PS3.getButtonPress(CROSS)) gutterMotor.write(GUTTER_DOWN_VALUE);
    else gutterMotor.writeMicroseconds(1500); 
  }
}
#endif

/*
  Completely stops the motors
  IMPORTANT NOTE: This is the equivalent of putting your car in NEUTRAL, if on a hill, it will roll! If it's already moving, it will coast!
*/
void stop()
{
  lfmotor.writeMicroseconds(1500); //stop value
  rfmotor.writeMicroseconds(1500);
  lrmotor.writeMicroseconds(1500);
  rrmotor.writeMicroseconds(1500);

  
#ifdef PERIPHERALS
  lifterMotor.writeMicroseconds(1500);
  ballRoller.writeMicroseconds(1500);
  ballActuator.writeMicroseconds(1500);
  keyRoller.writeMicroseconds(1500);
  gutterMotor.writeMicroseconds(1500);
#endif
}


// Gets status of the PS3 controllers battery and turns on LEDs to represent state of charge.

void battStat()
{
  if (PS3.getStatus(Full)) PS3.setLedRaw(15); //Turns on LEDs: 1,2,3,4
  else if (PS3.getStatus(High)) PS3.setLedRaw(7); //Turns on LEDs: 1,2,3
  else if (PS3.getStatus(Low)) PS3.setLedRaw(3); //Turns on LEDs: 1,2
  else if (PS3.getStatus(Dying)) PS3.setLedRaw(1); //Turns on LEDs: 1
  else if (PS3.getStatus(Shutdown)) PS3.setLedRaw(9); //Turns on LEDs: 1,4   Not entirely sure if this is the lowest reportable battery level, or if the controller is actually turned off.
  else PS3.setLedRaw(9);
}
/*
void governor()
{
  if (PS3.getButtonPress(CROSS)) {
    if (handicap == initialHandicap)  {
      handicap = 1;
      turnhandicap = 3; // Reduces the turn speed when the speed governor is removed.
      PS3.setLedOn(LED4);
      PS3.setRumbleOn(10, 0, 30, 255); //turns on the vibration motors. 10 is duration in ms for left and right motors. 255 is the power
    }
  }
  else if (handicap != initialHandicap) {
    handicap = initialHandicap;
    turnhandicap = 1; // Remove the turn speed limitation when the speed governor is put back in place.
    PS3.setLedOff(LED4);
    PS3.setRumbleOn(20, 255, 10, 0); //turns on the vibration motors. 10 is duration in ms for left and right motors. 255 is the power
  }
}
*/
