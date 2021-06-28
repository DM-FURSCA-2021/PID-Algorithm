//Include Header File Statements ----------------------------------------------------
#include <fstream>
#include <iostream>
#include <cmath>
#include <math.h>
#include <stdio.h>
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "Adafruit_FRAM_I2C.h"

/*INCLUDE FULL FILE PATHWAY IF THE PROGRAM CANNOT FIND THE ARDUINOPID.h FILE*/
#include "ArduinoPID.h"
//-------------------------------------------------------------------------------------

using namespace std; //Declares steandard namespace usage / standard name terminology

uint16_t BNO055_SAMPLERATE_DELAY_MS = 10; //Specific value for time in between BNO data samples. Should be same as time step in PID header file

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x29); //Sets i2c address for BNO

int servoPin1=6; //Declares the Arduino pin we want for servo1
int servoPos1=90; //Initial value for servo1 angle

int servoPin2=5; //Declares the Arduino pin we want for servo2
int servoPos2=90; //Initial value for servo2 angle

Servo myServo1; //Declares servo1 as a servo via header file command
Servo myServo2; //Declares servo2 as a servo via header file command

Adafruit_FRAM_I2C fram     = Adafruit_FRAM_I2C(); // Activates FRAM chip

void setup(void) //Setup function to be ran by the Arduino only once
{

  
  Serial.begin(115200); //Setup command to begin sending information to the Serial monitor at the baudrate specified
  
  myServo1.attach(servoPin1); //Activates servo1 on the pin specified in the command earlier
  myServo1.write(servoPos1); //Sets servo1 angle to initial value
  myServo2.attach(servoPin2); //Activates servo2 on the pin specified in the command earlier
  myServo2.write(servoPos2); //Sets servo2 angle to initial value

  if (fram.begin()) {  // you can stick the new i2c addr in here, e.g. begin(0x51); //IF statement to print whether the Arduino has found the FRAM chip or not
    Serial.println("Found I2C FRAM");
  } else {
    Serial.println("I2C FRAM not identified ... check your connections?\r\n");
    Serial.println("Will continue in case this processor doesn't support repeated start\r\n");
  }
  
  if(!bno.begin()) //IF statement to print whether the Arduino has found the BNO chip or not
    {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  bno.setExtCrystalUse(true); //Sets the timing source for the BNO chip as an external source (the Arduino) instead of the internal crystal inside the chip -- this is to measure everything at the same time through the arduino

//  WRITING VALUES TO FRAM CHIP
//  short myNumber = 13;
//  fram.write8(1, myNumber);

//  READING VALUES FROM FRAM CHIP
//  fram.read8(1)

    //F15 PID gains------
    double KP = 900; //Proportional Gain
    double KI = 0.05; //Integral Gain
    double KD = 210; //Derivative Gain

    for (short i = 0; i < 2; i++) { //Loop to set initial theta values for ease of use in simpler formulas in PID loop
      
      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER); //Sets the object "euler" equal to the 3-dimensional euler (position) vector from each BNO measurement (must be done every time BNO makes a measurement)
      
      runtime[i] = i*10; //Sets the index value in the runtime equal to the number of milliseconds that have elapsed
      
      thetaY[i] = euler.y(); //Sets the index value in the thetaY array equal to the y value in the BNO position vector
      thetaZ[i] = euler.z(); //Sets the index value in the thetaZ array equal to the Z value in the BNO position vector


      // Print Statement for all relevant information while for loop is active -------------------------------------------------------------------------------------------------------------------------------------------------------------------
      
      Serial.print(F("n= "));Serial.print(i);Serial.print(F(" runtime= "));Serial.print(runtime[i]);Serial.print(F(" thetaY= "));Serial.print(thetaY[i]);Serial.print(F(" thetaZ= "));Serial.print(thetaZ[i]);
      Serial.print(F(" G-angleY= "));Serial.print(gimbal_angleY[i]);Serial.print(F(" G-angleZ= "));Serial.print(gimbal_angleZ[i]);Serial.print(F(" S-angleY= "));Serial.print(servoPos1);Serial.print(F(" S-angleZ= "));Serial.println(servoPos2);
      delay(BNO055_SAMPLERATE_DELAY_MS);
      //------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

    }

    float runningSumY = thetaY[0]+thetaY[1]; //Declares runningSum variable to keep track of rocket's total angle offset in Y axis
    float proportional_errorY = 0.0; //Declares proportional_error variable for Y axis
    float integral_errorY = 0.0; //Declares integral_error variable for Y axis
    float derivative_errorY = 0.0; //Declares derivative_error variable for Y axis
    float outputY = 0.0; //Declares PID output variable for Y axis

    float runningSumZ = thetaZ[0]+thetaZ[1]; //Declares runningSum variable to keep track of rocket's total angle offset in Z axis
    float proportional_errorZ = 0.0; //Declares proportional_error variable for Z axis
    float integral_errorZ = 0.0; //Declares integral_error variable for Z axis
    float derivative_errorZ = 0.0; //Declares derivative_error variable for Y axis
    float outputZ = 0.0; //Declares PID output variable for Z axis

for (int n = 2; n <= sim_time; n++) { //Loop for actual PID algorithm math

        runtime[n] = n*10; //Sets the index value in the runtime equal to the number of milliseconds that have elapsed (just like in the other for loop but for the remainder of the runtime)
      
        imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER); //Sets the object "euler" equal to the 3-dimensional euler (position) vector from each BNO measurement
        thetaY[n] = euler.y(); //Sets the index value in the thetaY array equal to the y value in the BNO position vector
        thetaZ[n] = euler.z(); //Sets the index value in the thetaZ array equal to the Z value in the BNO position vector

      //PID control
        runningSumY = runningSumY + (thetaY[n]-theta0); //Adds new value to running sum in Y axis rocket angle for current index
        runningSumZ = runningSumZ + (thetaZ[n]-theta0); //Adds new value to running sum in Z axis rocket angle for current index

        proportional_errorY = thetaY[n]-theta0; //Proportional error for Y axis - difference between current measured angle and the desired value (perfectly vertical in this case)
        integral_errorY = runningSumY; //Integral error for Y axis - equal to the running sum of rocket angles for Y axis
        derivative_errorY = (thetaY[n]-thetaY[n-1])/time_step; //Derivative error for Y axis - Equal to the change in Y axis rocket angle for current time step only

        proportional_errorZ = thetaZ[n]-theta0; //Proportional error for Z axis - difference between current measured angle and the desired value (perfectly vertical in this case)
        integral_errorZ = runningSumZ; //Integral error for Z axis - equal to the running sum of rocket angles for Z axis
        derivative_errorZ = (thetaZ[n]-thetaZ[n-1])/time_step; //Derivative error for Y axis - Equal to the change in Y axis rocket angle for current time step only

        outputY = - KP*proportional_errorY - KI*integral_errorY - KD*derivative_errorY; //Actual PID equation math or Y axis
        outputZ = - KP*proportional_errorZ - KI*integral_errorZ - KD*derivative_errorZ; //Actual PID equation math or Z axis


        //If statement to keep the output value rate of change from exceeding the maximum turn speed for servo in Y axis
        if ((outputY - gimbal_angleY[n-1]) > maxRotationPerStep) { 
            gimbal_angleY[n] = gimbal_angleY[n-1] + maxRotationPerStep;
        }
        else if ((gimbal_angleY[n-1] - outputY) > maxRotationPerStep) {
            gimbal_angleY[n] = gimbal_angleY[n-1] - maxRotationPerStep;
        }
        else {
            gimbal_angleY[n] = outputY;
        }
        //--------------------------------------------------------------------------------------------------------------
        

        //If statement to keep the maximum angle value for servos within their mobility range for Y axis
        if (gimbal_angleY[n] > max_angle) { 
            gimbal_angleY[n] = max_angle;
        }
        if (gimbal_angleY[n] < - max_angle){
            gimbal_angleY[n] = - max_angle;
        }
        //----------------------------------------------------------------------------------------------
        

        //If statement to keep the output value rate of change from exceeding the maximum turn speed for servo in Z axis
        if ((outputZ - gimbal_angleZ[n-1]) > maxRotationPerStep) { 
            gimbal_angleZ[n] = gimbal_angleZ[n-1] + maxRotationPerStep;
        }
        else if ((gimbal_angleZ[n-1] - outputZ) > maxRotationPerStep) { 
            gimbal_angleZ[n] = gimbal_angleZ[n-1] - maxRotationPerStep;
        }
        else {
            gimbal_angleZ[n] = outputZ;
        }
        //--------------------------------------------------------------------------------------------------------------
        

        //If statement to keep the maximum angle value for servos within their mobility range for Z axis
        if (gimbal_angleZ[n] > max_angle) { 
            gimbal_angleZ[n] = max_angle;
        }
        if (gimbal_angleZ[n] < - max_angle){
            gimbal_angleZ[n] = - max_angle;
        }
        //----------------------------------------------------------------------------------------------

        servoPos1 = 90+5*gimbal_angleY[n]; //Calculates the servo angle based on the given gimbal angle for servo1
        myServo1.write(servoPos1); //Applies the calculated angle for servo1

        servoPos2 = 90+5*gimbal_angleZ[n]; //Calculates the servo angle based on the given gimbal angle for servo2
        myServo2.write(servoPos2); //Applies the calculated angle for servo2

        // Print Statement for all relevant information while PID loop is active ------------------------------------------------------------------------------------------------------------------------------------------------------------------
        
        Serial.print(F("n= "));Serial.print(n);Serial.print(F(" runtime= "));Serial.print(runtime[n]);Serial.print(F(" thetaY= "));Serial.print(thetaY[n]);Serial.print(F(" thetaZ= "));Serial.print(thetaZ[n]);
        Serial.print(F(" G-angleY= "));Serial.print(gimbal_angleY[n]);Serial.print(F(" G-angleZ= "));Serial.print(gimbal_angleZ[n]);Serial.print(F(" S-angleY= "));Serial.print(servoPos1);Serial.print(F(" S-angleZ= "));Serial.println(servoPos2);
        delay(BNO055_SAMPLERATE_DELAY_MS);
        //-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    }









  
}

















void loop() { //Not planning on using void loop function for anything because I want to have all flight stages in the same program.
}
