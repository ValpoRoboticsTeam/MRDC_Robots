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
  Technology Owners: Matt Bull

  /////Revision History/////

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


  Revisions needed to code (may not include everything)
  ---------------------------------
  Ernie only has two drive motors  ~Matt
  Ernie will soon have four drive motors... ~Matt
  ---------------------------------

  Parts List
  ---------------------------------
  Uses USBHostShield 2.0
  Arduino Mega ADK
  USB Bluetooth Dongle - some models not supported
  PS3 Controller - name brand recommended
  ---------------------------------

  Pin Layout
  ---------------------------------
  9: Left Front Motor
  10: Left Rear Motor
  6: Right Front Motor
  8: Right Rear Motor
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

#define ENABLE_DRIVE
#define ENABLE_GOVERNOR_CHANGE //comment out this line to lock the governor
#define ARM_READY //uncomment this to enable the left arm shoulder motor
#define BUCKET_READY //uncomment this to enable the bucket motor
//#define ENABLE_MOTOR_ADJUST //disabled to prevent accidental adjustment
//#define ENABLE_VOLT_MEASURE //disabled because the physical implementation occasionally killed an arduino...

//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Pin numbers for motors and servos set here.
#define LF_MOTOR 10
#define RF_MOTOR 8
#define LR_MOTOR 11
#define RR_MOTOR 9

#define LEFT_ARM 6
#define BALL_SERVO 5
#define WRIST_SERVO 3
#define BUCKET 2
#define WRIST 4
//----------------------------------------------------------------------

//If one of the motors is spinning when stopped, adjust it's offset here
#define LEFT_ADJUST 0
#define RIGHT_ADJUST 0
#define WRIST_DELAY 15 //this is the number of loops the program will go through between allowing wrist servo value to be changed
//------------------------------------------------------------
//This is for measuring battery voltage
#define NUM_SAMPLES 5
int sum = 0;                    // sum of samples taken
unsigned char sample_count = 0; // current sample number
float voltage = 0.0;            // calculated voltage
//------------------------------------------------------------

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
Servo leftArm;
Servo bucket;
Servo wrist;
//servo objects
Servo ballServo;
Servo wristServo;
//----------------------------------------------------------------------

//----------------------------------------------------------------------
#define initialMotorCorrect 3
#define initialHandicap 4
#define initialArmHoldPower 100
int driveMode = 0; // 0 for yx control, 1 for yy control
int motorCorrect = initialMotorCorrect;
int Drive = 0; //Initial speed before turning calculations
int Turn = 0; //Turn is adjustment to drive for each motor separately to create turns
int leftYinput = 0; //Tank Drive input variables
int rightYinput = 0;
int xInput = 0;
int left = 0;
int right = 0;
int handicap = initialHandicap; //speed limited mode
int noHandicap = 1; //not speed limited
int turnhandicap = 1; //This value gets changed by the governor function to reduce trun speed when speed governor is removed
int ballservoPos = 1;
int wristPos = 90;
int wristLoop = 0;
int armDown = 0, armUp = 0; //for fine control of robot arm
int armHold = 0; //toggle for holding arm
int armHoldPower = initialArmHoldPower;
//----------------------------------------------------------------------


void setup() {
  // put your setup code here, to run once:
  lfmotor.attach(LF_MOTOR);
  rfmotor.attach(RF_MOTOR);
  lrmotor.attach(LR_MOTOR);
  rrmotor.attach(RR_MOTOR);

#ifdef ARM_READY
  ballServo.attach(BALL_SERVO);
  ballServo.write(180); //ensure open at start
  leftArm.attach(LEFT_ARM, 1000, 2000);
  wrist.attach(WRIST, 1000, 2000);
  wristServo.attach(WRIST_SERVO, 1050, 1950); //1050 and 1950 are the min and max PWM width in μs
  wristServo.write(wristPos); //ensures at center position at startup
#endif
#ifdef BUCKET_READY
  bucket.attach(BUCKET, 1000, 2000);
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

#ifdef ENABLE_VOLT_MEASURE // if ENABLE_VOLT_MEASURE is defined do everything between here
  if (!(PS3.getButtonClick(SELECT)))voltMeasure(); //Calls the voltage measurement function
#endif //and here

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

#ifdef ARM_READY
    arm();
#endif

#ifdef ENABLE_GOVERNOR_CHANGE
    if (PS3.getButtonClick(CIRCLE)) governor(); //Shhh! It's a secret!
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
  leftYinput = map(PS3.getAnalogHat(LeftHatY), 0, 255, -90, 90); //left joystick y-axis
  //    rightYinput = map(PS3.getAnalogHat(RightHatY), 0, 255, -90, 90); //right joystick y-axis
  xInput = map(PS3.getAnalogHat(RightHatX), 0, 255, -90, 90); //x-axis
  //    Serial.print((PS3.getAnalogHat(LeftHatY)));
  //    Serial.print("\t");
  //    Serial.print((leftYinput));
  //    Serial.print("\t");
  //    Serial.print((left));
  //    Serial.print("\t");
  //    Serial.print((PS3.getAnalogHat(RightHatX)));
  //    Serial.print("\t");
  //    Serial.print((xInput));
  //    Serial.print("\t");
  //    Serial.print(right);

  //    Serial.print("\n");
  //    Serial.print(PS3.getAnalogButton(UP));
  //    Serial.print("\t");
  //    Serial.print(PS3.getAnalogButton(DOWN));
  //    Serial.print("\t");

  /*
     Joysticks have a "sticky" area around the middle of the joystick - this means they never go back
     to true zero. So we have to check and see if the joystick is "close enough", in which case we say
     it is zero.
  */
  if (abs(leftYinput) < 10)leftYinput = 0;
  if (abs(rightYinput) < 10)rightYinput = 0;
  if (abs(xInput) < 10)xInput = 0;

#ifdef ENABLE_MOTOR_ADJUST
  /*
     Press UP or DOWN on the D-pad to edit the correction values. Now with HAPTICK feedback! Whoohoo!
  */
  if (PS3.getButtonClick(RIGHT))
  {
    motorCorrect++;
    if (motorCorrect != initialMotorCorrect)PS3.setRumbleOn(10, 0, 15, 255); //turns on the vibration motors. 10 is duration in ms for left and right motors. 255 is the power
    else PS3.setRumbleOn(20, 255, 15, 255);
  }
  if (PS3.getButtonClick(LEFT))
  {
    motorCorrect--;
    if (motorCorrect != initialMotorCorrect)PS3.setRumbleOn(10, 255, 10, 0); //turns on the vibration motors. 10 is duration in ms for left and right motors. 255 is the power
    else PS3.setRumbleOn(20, 255, 20, 255);
  }
#endif
}

void drive()
{
  //Instead of following some sort of equation to slow down acceleration
  //We just increment the speed by one towards the desired speed.
  //The acceleration is then slowed because of the loop cycle time
  if (Drive < leftYinput)Drive++; //Accelerates
  else if (Drive > leftYinput) Drive--; //Decelerates

  if (Turn < xInput) Turn++;
  else if (Turn > xInput) Turn--;

  int ThrottleL = ((Drive - (Turn / turnhandicap)) / handicap) + LEFT_ADJUST; //This is the final variable that decides motor speed.
  int ThrottleR = ((Drive + (Turn / turnhandicap)) / handicap) + RIGHT_ADJUST;
  if (PS3.getButtonPress(L1)) { // This will flip the direction of the left stick to allow easier driving in reverse.
    ThrottleL = ((Drive + (Turn / turnhandicap)) / handicap) + LEFT_ADJUST;
    ThrottleR = ((Drive - (Turn / turnhandicap)) / handicap) + RIGHT_ADJUST;
    ThrottleL = -ThrottleL;
    ThrottleR = -ThrottleR;
  }
  if (ThrottleL > 90) ThrottleL = 90;
  if (ThrottleR > 90) ThrottleR = 90;

  left = (-ThrottleL + 90 + motorCorrect);
  right = (ThrottleR + 90 + motorCorrect);
  lfmotor.write(left); //Sending values to the speed controllers
  rfmotor.write(right);
  lrmotor.write(left); //Send values to the rear speed controllers.
  rrmotor.write(right);
}

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
  leftArm.writeMicroseconds(1500);
  wrist.writeMicroseconds(1500);
  bucket.writeMicroseconds(1500);
}

void arm()
{
  if (PS3.getButtonClick(START)) {//this toggles the armHold mode on and off
    if (armHold == 1) {//if armHold is on
      armHold = 0; //turn armHold off
      PS3.setLedOff(LED3); //turn off LED3
    }
    else {
      armHold = 1; //turn armHold on
      PS3.setLedOn(LED3); //turn on LED3 so user knows armHold mode is currently on
    }
  }
  armDown = map(PS3.getAnalogButton(L2), 0, 255, 90, 70);
  armUp = map(PS3.getAnalogButton(R2), 0, 255, 90, 115);
  if (armDown != 90) leftArm.write(armDown); //down
  else if (armUp != 90) leftArm.write(armUp); //up
  else if (armHold == 1) {
    leftArm.write(armHoldPower); //the armHold function is meant to counteract the force of gravity to hold the arm
    if (PS3.getButtonPress(SELECT)) {//while the select button is held...
      if (PS3.getButtonClick(CIRCLE) && armHoldPower < 115) armHoldPower++;//clicking circle will increase armHoldPower
      if (PS3.getButtonClick(CROSS)) armHoldPower = initialArmHoldPower;//clicking cross will set armHoldPower to the defined initialArmHoldPower
    }
  }
  else leftArm.writeMicroseconds(1500); //stop
//  Serial.print((armUp)); //allow
//  Serial.print("\t");
//  Serial.println((armHoldPower)); //allow

  if (PS3.getButtonClick(R1)) {
    if (ballservoPos == 0) {
      ballServo.write(180); //close
      ballservoPos = 1;
    }
    else if (ballservoPos == 1) {
      ballServo.write(90); //open
      ballservoPos = 0;
    }
  }
  if (wristLoop >= WRIST_DELAY) {
    if (PS3.getButtonPress(UP) && wristPos < 180) wristPos++;
    if (PS3.getButtonPress(DOWN) && wristPos > 0) wristPos--;
    wristLoop = 0;
  }
  else wristLoop++;
  wristServo.write(wristPos);
  if (PS3.getButtonPress(RIGHT)) wrist.write(180);
  else if (PS3.getButtonPress(LEFT)) wrist.write(0);
  else wrist.write(90);

  if (PS3.getButtonPress(SQUARE)) {
    bucket.write(0);
  }
  else if (PS3.getButtonPress(TRIANGLE)) {
    bucket.write(180);
  }
  else bucket.writeMicroseconds(1500);
}

void voltMeasure() //A0 is measurement pin
{
  // take a number of analog samples and add them up
  if (sample_count < NUM_SAMPLES) {
    sum += analogRead(A0);
    sample_count++;
  }
  else {
    // calculate the voltage
    // use 5.0 for a 5.0V ADC reference voltage
    // 5.015V is the calibrated reference voltage
    voltage = ((float)sum / (float)NUM_SAMPLES * 4.98) / 1024.0;
    // send voltage for display on Serial Monitor
    // voltage multiplied by 11 when using voltage divider that
    // divides by 11. 11.132 is the calibrated voltage divide
    // value
    Serial.print(voltage * 11.47);
    Serial.println (" V");
    sample_count = 0;
    sum = 0;
  }
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
