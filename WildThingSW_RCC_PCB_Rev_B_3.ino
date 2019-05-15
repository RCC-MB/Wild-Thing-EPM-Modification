 /*
   Name:      WildThingSW_RCC
   Revision:  B_3
   Date:      May 15, 2019

   Creator:   Shea Hunt. Adapted from code developed by Matthew Gale and Grant Kuhl
   Company:   Rehabilitation Centre for Children, Winnipeg MB, Canada

   Description:
   Takes input from various interfaces, processes the input, and sends the appropriate signals to the
   stock Fisher Price Wild Thing Controller (FPWTC). Signals to the FPWTC are voltages that mimic the
   original dual joystick system used on a stock Wild Thing.

   Troubleshooting:
   LED on Arduino control board will blink if an error is detected.
   LED Codes:
   3 - Joystick disconnected, joystick values out of bounds,or wrong interface selected
   4 - Button is not reaching neutral state
   5 - Left button value out of range
   6 - Right button value out of range

*/



#include <Wire.h>
#include <SoftwareSerial.h>
#include <math.h>
#include "mcp4728.h"

//Revision number. Please change as necessary.
#define VERSION "REV_PCB_B_3"

//Compile Options
//#define DEBUG
#define RAMP_SPEED


/*
  Left motor black wire on DAC0 (A)
  Left motor green wire on DAC1 (B)
  Right motor green wire on DAC2 (C)
  Right motor black wire on DAC3 (D)
*/



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Joystick Physical Parameters~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//Specific to current joystick used! Change as neccesary
const int joystickXMin = 391; //412
const int joystickXMax = 622; //606
const int joystickYMin = 400; //402
const int joystickYMax = 613; //603
const int outOfRange = 25;
const int xNeutralPoint = 509; //509
const int yNeutralPoint = 510; //509
const int joystickXRange = joystickXMax - joystickXMin;
const int joystickYRange = joystickYMax - joystickYMin;
const int joystickXMinCentered = xNeutralPoint - joystickXRange/2;
const int joystickXMaxCentered = xNeutralPoint + joystickXRange/2;
const int joystickYMinCentered = yNeutralPoint - joystickYRange/2;
const int joystickYMaxCentered = yNeutralPoint + joystickYRange/2;

//~~~~~~~~~~~~~~~~~~~~~~~~~~Joystick Neutral zone~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
const int neutralSize = 15; //Change this to increase the X and Y size of the neutral zone. Caution below value of 5 - could cause movement without input
const int xNeutralUpper = xNeutralPoint + neutralSize;
const int xNeutralLower = xNeutralPoint - neutralSize;
const int yNeutralUpper = yNeutralPoint + neutralSize;
const int yNeutralLower = yNeutralPoint - neutralSize;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Straight Away Deadband~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
const int xDeadBandSize = 10; //Change this to increase straight forward/reverse column
const int xDeadBandLowerBound = xNeutralPoint - xDeadBandSize;
const int xDeadBandUpperBound = xNeutralPoint + xDeadBandSize;


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Other Constants~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
const float slowDown = 0.75; //0-1. Slows down the straight away and zero-radius turn speeds
const int loopDelay = 1; //Delay in ms. Increasing this value will increase the time the system takes to ramp up to a speed
const int timeout = 5000; //in ms
const int max_errors = 3; //Maximum number of times either L/R button can go out of bounds before stopping vehicle

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Pinout~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
const int ledPin = 13;

const int speedPin = A3;
const int ldacPin = 2;
const int interfacePin = 5;

const int jSpdPin = A1;
const int jDirPin = A2;

const int right_buttonA = A1;
const int left_buttonA = A2;
const int forward_button = 10;
const int reverse_button = 11;


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Global Variables~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int speedKnob = 100;

boolean using_joystick = false;
boolean using_buttons = false;
int buttonNeutral = 0;
int buttonNeutralRange = 50;
int neutralTimeout = 0;
boolean forward = false;
boolean reverse = false;
boolean left = false;
boolean right = false;
int leftButtonErrorCount = 0;
int rightButtonErrorCount = 0;


int jSpd = 0;
int jDir = 0;
int xDir = 0;
int yDir = 0;
int leftMotorSpeed = 0; //-100 to 100
int rightMotorSpeed = 0; //-100 to 100
int targetLeftMotorSpeed = 0;
int targetRightMotorSpeed = 0;

int leftBlackWire = 0;
int leftGreenWire = 0;
int rightBlackWire = 0;
int rightGreenWire = 0;

mcp4728 dac = mcp4728(0);

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Setup~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void setup() {
  //Start serial and print out version
  Serial.begin(9600);
  Serial.print("Wild Thing Adaptation Software. RCC 2018. ");
  Serial.println(VERSION);
  Serial.println();
  Serial.println("Electrical and Software Designers: \nGrant Kuhl\nMatthew Gale\nShea Hunt\n");
  Serial.println("\nMechanical Designers: \nKristopher Isaak\nRyan Bohn\nMatthew Gale");
  Serial.println("\nJoystick Settings:");
  Serial.print("joystickXMin: ");
  Serial.println(joystickXMin);
  Serial.print("joystickXMax: ");
  Serial.println(joystickXMax);
  Serial.print("joystickYMin: ");
  Serial.println(joystickYMin);
  Serial.print("joystickYMax: ");
  Serial.println(joystickYMax);
  Serial.print("X Neutral Point: ");
  Serial.println(xNeutralPoint);
  Serial.print("Y Neutral Point: ");
  Serial.println(yNeutralPoint);
  Serial.print("Neutral size");
  Serial.println(neutralSize);
  Serial.print("X Deadband size: ");
  Serial.println(xDeadBandSize);

  //Start the DAC and set the reference voltage (vdd) in mV.
  dac.begin();
  dac.vdd(5200);
  
  //Pin Modes
  pinMode(ledPin, OUTPUT);
  pinMode(ldacPin, OUTPUT);
  pinMode(interfacePin, INPUT);

  boolean interface = digitalRead(interfacePin);
  if(interface == HIGH)
  {
    //Joystick
    using_joystick = true;
    Serial.println("Joystick Selected");
  }
  else if(interface == LOW)
  {
    //Buttons
    using_buttons = true;
    Serial.println("Buttons Selected");
  }
  
  if(using_joystick)
  {
    pinMode(jSpdPin, INPUT); //Do not use pullup here!
    pinMode(jDirPin, INPUT); //Do not use pullup here!
  }

  if(using_buttons)
  {
    pinMode(right_buttonA, INPUT_PULLUP);
    pinMode(left_buttonA, INPUT_PULLUP);
    pinMode(forward_button, INPUT_PULLUP);
    pinMode(reverse_button, INPUT_PULLUP);
    int rightButtonNeutral = analogRead(right_buttonA);
    int leftButtonNeutral = analogRead(left_buttonA);
    Serial.print("Right button neutral value: ");
    Serial.println(rightButtonNeutral);
    Serial.print("Left button neutral value: ");
    Serial.println(leftButtonNeutral);
    buttonNeutral = (rightButtonNeutral + leftButtonNeutral)/2;
    while((buttonNeutral > 700)||(buttonNeutral < 200))
    {
      //Waiting for button to be in neutral position (plugged in and not pressed)
      neutralTimeout++;
      if(neutralTimeout > timeout)
      {
        Serial.println("TIMEOUT");
        //Serial.println();
        errorMode(4, "ERROR - Button is not reaching neutral state");
      }
      buttonNeutral = analogRead(right_buttonA);
      delay(1);
    }
    Serial.print("Button neutral value selected: ");
    Serial.println(buttonNeutral);
    neutralTimeout = 0;
  }
  
  
  //Initial pin conditions
  digitalWrite(ldacPin, HIGH);
  digitalWrite(ledPin, HIGH);
  setSpeed(0, 0);
  
  
  #ifdef DEBUG
  Serial.print("xNeutralUpper: ");
  Serial.println(xNeutralUpper);
  Serial.print("xNeutralLower: ");
  Serial.println(xNeutralLower);
  Serial.print("yNeutralUpper: ");
  Serial.println(yNeutralUpper);
  Serial.print("yNeutralLower: ");
  Serial.println(yNeutralLower);
  #endif
  
  //Setup complete
  blinkLED(1);
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Loop~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void loop() {

  speedKnob = map(analogRead(speedPin), 1, 1023, 10, 3);
  
  #ifdef DEBUG
  //speedKnob = 1;
  
  #endif 


  
  if(using_joystick)
  {
    //Read the joystic positions
    jSpd = analogRead(jSpdPin);
    jDir = analogRead(jDirPin);
  
  
    //Check if joystick is out of bounds
    #ifndef DEBUG
    if(jSpd<(joystickYMin-outOfRange) || jSpd>(joystickYMax+outOfRange) || jDir<(joystickXMin-outOfRange) || jDir>(joystickXMax+outOfRange))
    {
      setSpeed(0, 0);
      errorMode(3, "ERROR - Joystick values are out of bounds");
    }
    #endif

    //Map the joystick values.
    xDir = map(jDir, joystickXMinCentered, joystickXMaxCentered, -100, 100);
    yDir = map(jSpd, joystickYMinCentered, joystickYMaxCentered, -100, 100);
  
  
    #ifdef DEBUG
    Serial.print("JSPD: ");
    Serial.println(jSpd);
    Serial.print("JDIR: ");
    Serial.println(jDir);
    Serial.println("\n");
    #endif
  
    //Neutral
    if(jDir>xNeutralLower && jDir<xNeutralUpper && jSpd>yNeutralLower && jSpd<yNeutralUpper)
    {
      #ifdef DEBUG
      Serial.println("Neutral");
      #endif
      setSpeed(0, 0); //Don't ramp. Immediately go back to zero
  
      //Reset the targets and current speeds
      targetLeftMotorSpeed = 0;
      targetRightMotorSpeed = 0;
      leftMotorSpeed = 0;
      rightMotorSpeed = 0;
    }
  
    //Straight Forward or Backward
    else if(jDir>xDeadBandLowerBound && jDir<xDeadBandUpperBound)
    {
      #ifdef DEBUG
      Serial.println("Straight away");
      #endif
      targetLeftMotorSpeed = yDir*slowDown;
      targetRightMotorSpeed = yDir*slowDown;
    }
  
    //Full left or full right
    else if(jSpd>yNeutralLower && jSpd<yNeutralUpper && (jDir<xNeutralLower || jDir>xNeutralUpper))
    {
      targetLeftMotorSpeed = xDir*slowDown;
      targetRightMotorSpeed = -xDir*slowDown;
    }
  
    //Mixing algorithm
    else
    {
      int magnitude = round(sqrt((sq(yDir)+sq(xDir))));
      float theta = map(round(atan2(abs(yDir), abs(xDir))*(180/3.1415926)), 0, 130, 0, 100);
      float direction1 = theta/100;
  
      
      #ifdef DEBUG
      Serial.print("Magnitude: ");
      Serial.println(magnitude);
      Serial.print("Theta: ");
      Serial.println(theta);
      Serial.print("Direction: ");
      Serial.println(direction1);
      #endif
      
      
      //Top Left Quadrant
      if(jSpd > yNeutralUpper && jDir < xNeutralLower)
      {
        targetLeftMotorSpeed = magnitude*direction1;
        targetRightMotorSpeed = magnitude;
      }
      
      //Top Right Quadrant
      else if(jSpd > yNeutralUpper && jDir > xNeutralUpper)
      {
        targetLeftMotorSpeed = magnitude;
        targetRightMotorSpeed = magnitude*direction1;
      }
      
      //Bottom Left Quadraft
      else if(jSpd < yNeutralLower && jDir < xNeutralLower)
      {
        targetLeftMotorSpeed = -magnitude;
        targetRightMotorSpeed = -magnitude*direction1;
      }
      
      //Bottom Right Quadrant
      else if(jSpd < yNeutralLower && jDir > xNeutralUpper)
      {
        targetLeftMotorSpeed = -magnitude*direction1;
        targetRightMotorSpeed = -magnitude;
      }
    }
  }//End if(using_joystick)

  else if(using_buttons)
  {
    //Reset button states
    forward = false;
    reverse = false;
    left = false;
    right = false;
//    
    //Read all the buttons
    int left_button_value = analogRead(left_buttonA);
    int right_button_value = analogRead(right_buttonA);
    #ifdef DEBUG
    Serial.print("left_button_value: ");
    Serial.println(left_button_value);
    Serial.print("right_button_value: ");
    Serial.println(right_button_value);
    //Serial.print("Forward button: ");
    //Serial.println(digitalRead(forward_button));
    //Serial.print("Reverse button: ");
    //Serial.println(digitalRead(reverse_button));
    #endif

    //Handle forward and reverse buttons
    if(digitalRead(forward_button) == LOW)
      forward = true;
    if(digitalRead(reverse_button) == LOW)
      reverse = true;

    //Handle left button
    if((left_button_value > (buttonNeutral - buttonNeutralRange)) && (left_button_value < (buttonNeutral + buttonNeutralRange)))
    {
      //In neutral range
      left = false;
      leftButtonErrorCount = 0;
    }
    else if(left_button_value < 50)
    {
      //Button is low
      left = true;
      leftButtonErrorCount = 0;
    }
    else
    {
      //Left button out of bounds
      Serial.print("WARNING - Left button value out of bounds: ");
      Serial.println(left_button_value);
      leftButtonErrorCount++;
    }
    if(leftButtonErrorCount >= max_errors)
    {
      errorMode(5, "ERROR - Left button value out of bounds (3 in a row)");
    }

    //Handle right button
    if((right_button_value > (buttonNeutral - buttonNeutralRange)) && (right_button_value < (buttonNeutral + buttonNeutralRange)))
    {
      //In neutral range
      right = false;
      rightButtonErrorCount = 0;
    }
    else if(right_button_value < 50)
    {
      //Button is low
      right = true;
      rightButtonErrorCount = 0;
    }
    else
    {
      //Right button out of bounds
      Serial.print("WARNING - Right button out of bounds: ");
      Serial.println(right_button_value);
      rightButtonErrorCount++;
    }
    if(rightButtonErrorCount >= max_errors)
    {
      errorMode(6, "ERROR - Right button value out of bounds (3 in a row)");
    }


    //Decide what to do
    if((forward == false) && (reverse == false) && left == false && right == false)
    {
      //Case 0: neutral
      #ifdef DEBUG
      //Serial.println("Neutral");
      #endif
      setSpeed(0,0);
      targetLeftMotorSpeed = 0;
      targetRightMotorSpeed = 0;
      leftMotorSpeed = 0;
      rightMotorSpeed = 0;
    }
    
    if(forward == true && reverse == false && left == false && right == false)
    {
      //Case 1: forward only
      #ifdef DEBUG
      //Serial.println("Forward");
      #endif
      targetLeftMotorSpeed = 80;
      targetRightMotorSpeed = 80;
    }
    else if(forward == false && reverse == true && left == false && right == false)
    {
      //Case 2: reverse only
      #ifdef DEBUG
      //Serial.println("Reverse");
      #endif
      targetLeftMotorSpeed = -80;
      targetRightMotorSpeed = -80;
    }
    else if(forward == false && reverse == false && left == true && right == false)
    {
      //Case 3: left only
      #ifdef DEBUG
      //Serial.println("left");
      #endif
      targetLeftMotorSpeed = 0;
      targetRightMotorSpeed = 80;
    }
    else if(forward == false && reverse == false && left == false && right == true)
    {
      //Case 4: right only
      #ifdef DEBUG
      //Serial.println("right");
      #endif
      targetLeftMotorSpeed = 80;
      targetRightMotorSpeed = 0;
    }
    else if(forward == true && reverse == true && left == false && right == false)
    {
      //Case 5: forward and reverse
      #ifdef DEBUG
      //Serial.println("F+B");
      #endif
      targetLeftMotorSpeed = 0;
      targetRightMotorSpeed = 0;
    }
    else if(forward == false && reverse == false && left == true && right == true)
    {
      //Case 6: left and right
      #ifdef DEBUG
      //Serial.println("L+R");
      #endif
      targetLeftMotorSpeed = 0;
      targetRightMotorSpeed = 0;
    }
    else if(forward == true && reverse == false && left == true && right == false)
    {
      //Case 7: forward and left
      #ifdef DEBUG
      //Serial.println("Slow left");
      #endif
      targetLeftMotorSpeed = 30;
      targetRightMotorSpeed = 80;
    }
    else if(forward == true && reverse == false && left == false && right == true)
    {
      //Case 8: forward and right
      #ifdef DEBUG
      //Serial.println("slow right");
      #endif
      targetLeftMotorSpeed = 80;
      targetRightMotorSpeed = 30;
    }
    else if(forward == false && reverse == true && left == true && right == false)
    {
      //Case 9: Reverse and left
      #ifdef DEBUG
      //Serial.println("slow back left");
      #endif
      targetLeftMotorSpeed = -30;
      targetRightMotorSpeed = -80;
    }
    else if(forward == false && reverse == true && left == false && right == true)
    {
      //Case 10: Reverse and right
      #ifdef DEBUG
      //Serial.println("slow back right");
      #endif
      targetLeftMotorSpeed = -80;
      targetRightMotorSpeed = -30;
    }
    else
    {
      #ifdef DEBUG
      //Serial.println("multiple");
      #endif
      targetLeftMotorSpeed = 0;
      targetRightMotorSpeed = 0;
    }
  }//End if(using_buttons)
  

  #ifdef RAMP_SPEED
  if((leftMotorSpeed != targetLeftMotorSpeed) || (rightMotorSpeed != targetRightMotorSpeed))
  {
    #ifdef DEBUG
//    Serial.print("Left Target: ");
//    Serial.print(targetLeftMotorSpeed);
//    Serial.print(" Actual: ");
//    Serial.println(leftMotorSpeed);
//    Serial.print("Right Target: ");
//    Serial.print(targetRightMotorSpeed);
//    Serial.print(" Actual: ");
//    Serial.println(rightMotorSpeed);
    #endif
    
    if(leftMotorSpeed < targetLeftMotorSpeed)
      leftMotorSpeed++;
    else if(leftMotorSpeed > targetLeftMotorSpeed)
      leftMotorSpeed--;
    if(rightMotorSpeed < targetRightMotorSpeed)
      rightMotorSpeed++;
    else if(rightMotorSpeed > targetRightMotorSpeed)
      rightMotorSpeed--;
  }

  setSpeed(leftMotorSpeed, rightMotorSpeed);
  #endif //End RAMP_SPEED

  #ifndef RAMP_SPEED
  setSpeed(targetLeftMotorSpeed, targetRightMotorSpeed);
  #endif
  
  
  delay(loopDelay);
  
  
  #ifdef DEBUG
  //delay(50);
  #endif
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Functions~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void setSpeed(float left, float right)
{
  /*
   * 
   * WARNING: Only change this function if you know what you're doing. Voltages are only capped by software. Voltages above 3.3v could
  damage the Wild Thing controller. 
  -100 = full reverse
  0 = neutral
  100 = full forward

  
  */
  
  left = constrain(left/speedKnob, -100, 100);
  right = constrain(right/speedKnob, -100, 100);
  
  //Neutral
  if(left == 0 && right == 0)
  {
    leftBlackWire = 1500;
    leftGreenWire = 1600;
    rightBlackWire = 1500;
    rightGreenWire = 1600;
  }
  
  if (left < 0)
  {
    leftBlackWire = 3200;
    leftGreenWire = map(left, -1, -100, 1580, 0010);
  }
  else if (left > 0)
  {
    leftBlackWire = 3200;
    leftGreenWire = map(left, 1, 100, 1620, 3200);
  }
  
  if (right < 0)
  {
    rightBlackWire = 3200;
    rightGreenWire = map(right, -1, -100, 1620, 3200);
  }
  else if (right > 0)
  {
    rightBlackWire = 3200;
    rightGreenWire = map(right, 1, 100, 1580, 0010);
  }
  
  //Have to bring the ldac pin high while registers are updated, then bringing ldac pin low will update all registers simultaneously
  //Without this, unexpected results have been observed
  digitalWrite(ldacPin, HIGH);
  delay(5);
  dac.voutWrite(leftBlackWire, leftGreenWire, rightGreenWire, rightBlackWire);
  delay(5);
  digitalWrite(ldacPin, LOW);
  delay(5);
  
  #ifdef DEBUG
//  printStatus();
//  Serial.print("LEFT MOTOR: ");
//  Serial.println(left);
//  Serial.print("RIGHT MOTOR: ");
//  Serial.println(right);
  #endif
}


void printStatus()
{
  /*
   * Prints the status of the DAC. Taken from mcp4728 library. 
   */
  Serial.println("NAME     Vref  Gain  PowerDown  Value");
  for (int channel=0; channel <= 3; channel++)
  {
    Serial.print("DAC");
    Serial.print(channel,DEC);
    Serial.print("   ");
    Serial.print("    ");
    Serial.print(dac.getVref(channel),BIN);
    Serial.print("     ");
    Serial.print(dac.getGain(channel),BIN);
    Serial.print("       ");
    Serial.print(dac.getPowerDown(channel),BIN);
    Serial.print("       ");
    Serial.println(dac.getValue(channel),DEC);
    
    Serial.print("EEPROM");
    Serial.print(channel,DEC);
    Serial.print("    ");
    Serial.print(dac.getVrefEp(channel),BIN);
    Serial.print("     ");
    Serial.print(dac.getGainEp(channel),BIN);
    Serial.print("       ");
    Serial.print(dac.getPowerDownEp(channel),BIN);
    Serial.print("       ");
    Serial.println(dac.getValueEp(channel),DEC);
  }
  Serial.println(" ");
}

void blinkLED(int numTimes)
{
  int blinkDelay = 300; //ms
  
  digitalWrite(ledPin, LOW);
  delay(500);
  for(int i=0; i<numTimes; i++)
  {
    digitalWrite(ledPin, HIGH);
    delay(blinkDelay);
    digitalWrite(ledPin, LOW);
    delay(500);
  }
  digitalWrite(ledPin, HIGH);
}

void errorMode(int mode, String code)
{
  setSpeed(0,0);
  while(true) //Requires a power reset
  {
    blinkLED(mode);
    digitalWrite(ledPin, LOW);
    Serial.println(code);
    delay(1000);
  }
}




