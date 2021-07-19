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
#include <Adafruit_BMP280.h>
Adafruit_BMP280 bmp;// I2C

/*INCLUDE FULL FILE PATHWAY IF THE PROGRAM CANNOT FIND THE ARDUINOPID.h FILE*/
#include "ArduinoPID.h"

//-------------------------------------------------------------------------------------

using namespace std; //Declares steandard namespace usage / standard name terminology

uint16_t BNO055_SAMPLERATE_DELAY_MS = 10; //Specific value for time in between BNO data samples. Should be same as time step in PID header file

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x29); //Sets i2c address for BNO

short servoPin1=6; //Declares the Arduino pin we want for servo1
short servoPos1=90; //Initial value for servo1 angle

short servoPin2=5; //Declares the Arduino pin we want for servo2
short servoPos2=90; //Initial value for servo2 angle

Servo myServo1; //Declares servo1 as a servo via header file command
Servo myServo2; //Declares servo2 as a servo via header file command

Adafruit_FRAM_I2C fram     = Adafruit_FRAM_I2C(); // Activates FRAM chip

void setup(void)//Setup function to be ran by the Arduino only once
{

  flightStage = 2;
  
  Serial.begin(115200); //Setup command to begin sending information to the Serial monitor at the baudrate specified
  
  myServo1.attach(servoPin1); //Activates servo1 on the pin specified in the command earlier
  myServo1.write(servoPos1); //Sets servo1 angle to initial value
  myServo2.attach(servoPin2); //Activates servo2 on the pin specified in the command earlier
  myServo2.write(servoPos2); //Sets servo2 angle to initial value

  if (fram.begin()) {  // you can stick the new i2c addr in here, e.g. begin(0x51); //IF statement to print whether the Arduino has found the FRAM chip or not
    Serial.println("Found I2C FRAM");
  } 
  else {
    Serial.println("I2C FRAM not identified ... check your connections?\r\n");
    Serial.println("Will continue in case this processor doesn't support repeated start\r\n");
  }
  
  if(!bno.begin()) //IF statement to print whether the Arduino has found the BNO chip or not
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  
  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                    "try a different address!"));
    while (1) delay(10);
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  bno.setExtCrystalUse(true); //Sets the timing source for the BNO chip as an external source (the Arduino) instead of the internal crystal inside the chip -- this is to measure everything at the same time through the arduino


    //F15 PID gains------
  float KP = 0.7; //Proportional Gain
  float KI = 0.00005; //Integral Gain
  float KD = .4; //Derivative Gain

  for (short n = 0; n < 2; n++) { //Loop to set initial theta values for ease of use in simpler formulas in PID loop

    runtime[n] = n*10; //Sets the index value in the runtime equal to the number of milliseconds that have elapsed
      
    thetaY[2] = thetaY[1]; //Array rearrangement
    thetaY[1] = thetaY[0];

    thetaZ[2] = thetaZ[1];
    thetaZ[1] = thetaZ[0];

    gimbal_angleY[1] = gimbal_angleY[0];
    gimbal_angleZ[1] = gimbal_angleZ[0];

    //BNO READINGS AND HEX SEPARATION
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER); //Sets the object "euler" equal to the 3-dimensional euler (position) vector from each BNO measurement (must be done every time BNO makes a measurement)

    thetaX = euler.x(); //Sets thetaX equal to the most recent x-axis orientation measurement
    thetaY[0] = euler.y() + thetaY_offset; //Sets first element in thetaY array equal to most recent y-axis orientation measurement
    thetaZ[0] = euler.z() + thetaZ_offset; //Sets first element in thetaZ array equal to most recent z-axis orientation measurement -- (-90 included to zero the sensor to earth's normal) 
    
    if (thetaX >= 0) { //If statement to separate the X-axis orientation into 3, single-byte values to be recorded on the FRAM chip -- Statements separated by whether the angle is positive or negative
      thetaIntX = floor(thetaX); //Separates integer value of X-axis orientation measurement
      thetaDecimalX = 100*(thetaX-floor(thetaX)); //Separates decimal value of X-axis orientation measurement
      thetaFlagX = 0; //Flag set to 0 to recognize as positive number
    }
    else {
      thetaIntX = -1*ceil(thetaX); //Separates the ineger value of the X-axis orientation measurement -- Temporarily changes stored data value to a positive integer. Reverted back to negative value in the memory-reading program
      thetaDecimalX = -100*(thetaX-ceil(thetaX)); //Separates decimal value of X-axis orientation measurement -- Temporarily changes stored data value to a positive integer. Reverted back to negative value in the memory-reading program
      thetaFlagX = 1; //Flag set to 1 to recognize as negative number
    }
    if (thetaY[0] >= 0) { //If statement to separate the Y-axis orientation into 3, single-byte values to be recorded on the FRAM chip -- Statements separated by whether the angle is positive or negative
      thetaIntY = floor(thetaY[0]); //Separates integer value of Y-axis orientation measurement
      thetaDecimalY = 100*(thetaY[0]-floor(thetaY[0])); //Separates decimal value of Y-axis orientation measurement
      thetaFlagY = 0; //Flag set to 0 to recognize as positive number
    }
    else {
      thetaIntY = -1*ceil(thetaY[0]); //Separates the ineger value of the Y-axis orientation measurement -- Temporarily changes stored data value to a positive integer. Reverted back to negative value in the memory-reading program
      thetaDecimalY = -100*(thetaY[0]-ceil(thetaY[0])); //Separates decimal value of Y-axis orientation measurement -- Temporarily changes stored data value to a positive integer. Reverted back to negative value in the memory-reading program
      thetaFlagY = 1; //Flag set to 1 to recognize as negative number
    }      
    if (thetaZ[0] >= 0) { //If statement to separate the Z-axis orientation into 3, single-byte values to be recorded on the FRAM chip -- Statements separated by whether the angle is positive or negative
      thetaIntZ = floor(thetaZ[0]); //Separates integer value of Z-axis orientation measurement
      thetaDecimalZ = 100*(thetaZ[0]-floor(thetaZ[0])); //Separates decimal value of Z-axis orientation measurement
      thetaFlagZ = 0; //Flag set to 0 to recognize as positive number
    }
    else {
      thetaIntZ = -1*ceil(thetaZ[0]); //Separates the ineger value of the Z-axis orientation measurement -- Temporarily changes stored data value to a positive integer. Reverted back to negative value in the memory-reading program
      thetaDecimalZ = -100*(thetaZ[0]-ceil(thetaZ[0])); //Separates decimal value of Z-axis orientation measurement -- Temporarily changes stored data value to a positive integer. Reverted back to negative value in the memory-reading program
      thetaFlagZ = 1; //Flag set to 1 to recognize as negative number
    }

    //BMP READINGS AND HEX SEPARATION
    altitude = bmp.readAltitude(); //Altitude measurement
    pressure = bmp.readPressure()-90000; //Pressure measurement -- subtracts 90,000Pa from the pressure measurements for easier storage on FRAM chip. Added back in chip reading program

    altitudeFirst = floor(altitude/100); //Separates first 2 digits of altitude measurement
    altitudeMid = (floor(altitude) - 100*altitudeFirst); //Separates middle 2 digits of altitude measurement
    altitudeLast = 100*(altitude - floor(altitude)); //Separates decimal digits of altitude measurement

    pressureFirst = floor(pressure/100); //Separates first 2 digits of pressure measurement
    pressureMid = (floor(pressure) - 100*pressureFirst); //Separates middle 2 digits of pressure measurement
    pressureLast = 100*(pressure - floor(pressure)); //Separates decimal digits of pressure measurement
   
      
    //Writing values to FRAM chip -- 22 values stored per measurement loop
    fram.write8(n*22, flightStage); //FLIGHT STAGE
    
    //ORIENTATION
    fram.write8(n*22+1, thetaIntX); //Integer value X-axis
    fram.write8(n*22+2, thetaDecimalX); //Decimal value X-axis
    fram.write8(n*22+3, thetaFlagX); //Positive / Negative Flag X-axis
    fram.write8(n*22+4, thetaIntY); //Integer value Y-axis
    fram.write8(n*22+5, thetaDecimalY); //Decimal value Y-axis
    fram.write8(n*22+6, thetaFlagY); //Positive / Negative Flag Y-axis
    fram.write8(n*22+7, thetaIntZ); //Integer value Z-axis
    fram.write8(n*22+8, thetaDecimalZ); //Decimal value Z-axis
    fram.write8(n*22+9, thetaFlagZ); //Positive / Negative Flag Z-axis

    //GIMBAL ANGLES
    fram.write8(n*22+10, gimbal_angleIntY); //Integer value Y-axis
    fram.write8(n*22+11, gimbal_angleDecimalY); //Decimal value Y-axis
    fram.write8(n*22+12, gimbal_angleIntZ); //Integer value Z-axis
    fram.write8(n*22+13, gimbal_angleDecimalZ); //Decimal value Z-axis

    //SERVO ANGLES
    fram.write8(n*22+14, servoPos1); //Y-axis
    fram.write8(n*22+15, servoPos2); //Z-axis

    //ALTITUDE
    fram.write8(n*22+16, altitudeFirst); //Digits in the hundreds and thousands place
    fram.write8(n*22+17, altitudeMid); //Digits in the Tens and Ones place
    fram.write8(n*22+18, altitudeLast); //Digits in the Tenths and Hundredths place

    //PRESSURE
    fram.write8(n*22+19, pressureFirst); //Digits in the hundreds and thousands place
    fram.write8(n*22+20, pressureMid); //Digits in the Tens and Ones place
    fram.write8(n*22+21, pressureLast); //Digits in the Tenths and Hundredths place
      

    
    // Print Statement for all relevant information while for loop is active -------------------------------------------------------------------------------------------------------------------------------------------------------------------
      
    Serial.print(F("n= "));
    Serial.println(n);
//    Serial.print(F(" runtime= "));
//    Serial.print(runtime[n]);
//    Serial.print(F(" ThetaX= "));
//    Serial.print(thetaX);
//    Serial.print(F(" thetaIntX= "));
//    Serial.print(thetaIntX);
//    Serial.print(F(" thetaDecimalX= "));
//    Serial.print(thetaDecimalX);
//    Serial.print(F(" thetaFlagX= "));
//    Serial.println(thetaFlagX);

//    Serial.print(F("thetaY= "));
//    Serial.print(thetaY[0]);
//    Serial.print(F(" thetaIntY= "));
//    Serial.print(thetaIntY);
//    Serial.print(F(" thetaDecimalY= "));
//    Serial.print(thetaDecimalY);
//    Serial.print(F(" thetaFlagY= "));
//    Serial.print(thetaFlagY);
//    Serial.print(F(" thetaZ= "));
//    Serial.print(thetaZ[0]);
//    Serial.print(F(" thetaIntZ= "));
//    Serial.print(thetaIntZ);
//    Serial.print(F(" thetaDecimalZ= "));
//    Serial.print(thetaDecimalZ);
//    Serial.print(F(" thetaFlagZ= "));
//    Serial.println(thetaFlagZ);
      
//    Serial.print(F("gimbal_angleY= "));
//    Serial.print(gimbal_angleY[0]);
//    Serial.print(F(" gimbal_angleIntY= "));
//    Serial.print(gimbal_angleIntY);
//    Serial.print(F(" gimbal_angleDecimalY= "));
//    Serial.print(gimbal_angleDecimalY);
//    Serial.print(F(" gimbal_angleZ= "));
//    Serial.println(gimbal_angleZ[0]);
//    Serial.print(F(" gimbal_angleIntZ= "));
//    Serial.print(gimbal_angleIntZ);
//    Serial.print(F(" gimbal_angleDecimalZ= "));
//    Serial.print(gimbal_angleDecimalZ);
//    Serial.print(F(" Servo angleY= "));
//    Serial.print(servoPos1);
//    Serial.print(F(" Servo angleZ= "));
//    Serial.println(servoPos2);

//    Serial.print(F("altitude= "));
//    Serial.print(altitude);
//    Serial.print(F(" altitudeLast= "));
//    Serial.print(altitudeLast);
//    Serial.print(F(" altitudeMid= "));
//    Serial.print(altitudeMid);
//    Serial.print(F(" altitudeFirst= "));
//    Serial.println(altitudeFirst);
//
//    Serial.print(F("pressure= "));
//    Serial.print(pressure);
//    Serial.print(F(" pressureFirst= "));
//    Serial.print(pressureFirst);
//    Serial.print(F(" pressureMid= "));
//    Serial.print(pressureMid);
//    Serial.print(F(" pressureLast= "));
//    Serial.println(pressureLast);
      
    //delay(BNO055_SAMPLERATE_DELAY_MS);
    //------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

  }

  //float runningSumY = thetaY[0]+thetaY[1]; //Declares runningSum variable to keep track of rocket's total angle offset in Y axis
  //float runningSumZ = thetaZ[0]+thetaZ[1]; //Declares runningSum variable to keep track of rocket's total angle offset in Z axis


  for (int n = 2; n <= motor_time; n++) { //Loop for actual PID algorithm math

    runtime[n] = n*10; //Sets the index value in the runtime equal to the number of milliseconds that have elapsed (just like in the other for loop but for the remainder of the runtime)

    thetaY[2] = thetaY[1]; //Array rearrangement
    thetaY[1] = thetaY[0];

    gimbal_angleY[1] = gimbal_angleY[0];

    errorY[1] = errorY[0];

    thetaZ[2] = thetaZ[1];
    thetaZ[1] = thetaZ[0];

    gimbal_angleZ[1] = gimbal_angleZ[0];

    errorZ[1] = errorZ[0];
      
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER); //Sets the object "euler" equal to the 3-dimensional euler (position) vector from each BNO measurement

    thetaX = euler.x(); //Sets thetaX equal to the most recent X-axis orientation measurement
    thetaY[0] = euler.y() + thetaY_offset; //Sets first element in thetaY array equal to most recent Y-axis orientation measurement
    thetaZ[0] = euler.z() + thetaZ_offset; //Sets first element in thetaZ array equal to most recent Z-axis orientation measurement -- (-90 included to zero the sensor to earth's normal)
      
    if (thetaX >= 0) { //If statement to separate the X-axis orientation into 3, single-byte values to be recorded on the FRAM chip -- Statements separated by whether the angle is positive or negative
      thetaIntX = floor(thetaX); //Separates integer value of X-axis orientation measurement
      thetaDecimalX = 100*(thetaX-floor(thetaX)); //Separates decimal value of X-axis orientation measurement
      thetaFlagX = 0; //Flag set to 0 to recognize as positive number
    }
    else {
      thetaIntX = -1*ceil(thetaX); //Separates the ineger value of the X-axis orientation measurement -- Temporarily changes stored data value to a positive integer. Reverted back to negative value in the memory-reading program
      thetaDecimalX = -100*(thetaX-ceil(thetaX)); //Separates decimal value of X-axis orientation measurement -- Temporarily changes stored data value to a positive integer. Reverted back to negative value in the memory-reading program
      thetaFlagX = 1; //Flag set to 1 to recognize as negative number
    }
    if (thetaY[0] >= 0) { //If statement to separate the Y-axis orientation into 3, single-byte values to be recorded on the FRAM chip -- Statements separated by whether the angle is positive or negative
      thetaIntY = floor(thetaY[0]); //Separates integer value of Y-axis orientation measurement
      thetaDecimalY = 100*(thetaY[0]-floor(thetaY[0])); //Separates decimal value of Y-axis orientation measurement
      thetaFlagY = 0; //Flag set to 0 to recognize as positive number
    }
    else {
      thetaIntY = -1*ceil(thetaY[0]); //Separates the ineger value of the Y-axis orientation measurement -- Temporarily changes stored data value to a positive integer. Reverted back to negative value in the memory-reading program
      thetaDecimalY = -100*(thetaY[0]-ceil(thetaY[0])); //Separates decimal value of Y-axis orientation measurement -- Temporarily changes stored data value to a positive integer. Reverted back to negative value in the memory-reading program
      thetaFlagY = 1; //Flag set to 1 to recognize as negative number
    }      
    if (thetaZ[0] >= 0) { //If statement to separate the Z-axis orientation into 3, single-byte values to be recorded on the FRAM chip -- Statements separated by whether the angle is positive or negative
      thetaIntZ = floor(thetaZ[0]); //Separates integer value of Z-axis orientation measurement
      thetaDecimalZ = 100*(thetaZ[0]-floor(thetaZ[0])); //Separates decimal value of Z-axis orientation measurement
      thetaFlagZ = 0; //Flag set to 0 to recognize as positive number
    }
    else {
      thetaIntZ = -1*ceil(thetaZ[0]); //Separates the ineger value of the Z-axis orientation measurement -- Temporarily changes stored data value to a positive integer. Reverted back to negative value in the memory-reading program
      thetaDecimalZ = -100*(thetaZ[0]-ceil(thetaZ[0])); //Separates decimal value of Z-axis orientation measurement -- Temporarily changes stored data value to a positive integer. Reverted back to negative value in the memory-reading program
      thetaFlagZ = 1; //Flag set to 1 to recognize as negative number
    }
    
    //PID control
    errorY[0] = theta0 - thetaY[0];
    errorZ[0] = theta0 - thetaZ[0];

    proportional_errorY = errorY[0]; //Proportional error for Y axis - difference between current measured angle and the desired value (perfectly vertical in this case)
    integral_errorY += errorY[0] * time_step; //Integral error for Y axis - equal to the running sum of rocket angles for Y axis
    derivative_errorY = (errorY[0] - errorY[1])/time_step; //Derivative error for Y axis - Equal to the change in Y axis rocket angle for current time step only

    proportional_errorZ = errorZ[0]; //Proportional error for Z axis - difference between current measured angle and the desired value (perfectly vertical in this case)
    integral_errorZ += errorZ[0] * time_step; //Integral error for Z axis - equal to the running sum of rocket angles for Z axis
    derivative_errorZ = (errorZ[0] - errorZ[1])/time_step; //Derivative error for Y axis - Equal to the change in Y axis rocket angle for current time step only

    outputY = + KP*proportional_errorY + KI*integral_errorY + KD*derivative_errorY; //Actual math for PID equation in Y axis
    outputZ = + KP*proportional_errorZ + KI*integral_errorZ + KD*derivative_errorZ; //Actual math for PID equation in Z axis
    
    //If statement to keep the output value rate of change from exceeding the maximum turn speed for servo in Y axis
    if ((outputY - gimbal_angleY[1]) > maxRotationPerStep) { 
      gimbal_angleY[0] = gimbal_angleY[1] + maxRotationPerStep;
    }
    else if ((gimbal_angleY[1] - outputY) > maxRotationPerStep) {
      gimbal_angleY[0] = gimbal_angleY[1] - maxRotationPerStep;
    }
    else {
      gimbal_angleY[0] = outputY;
    }
    //--------------------------------------------------------------------------------------------------------------
        

    //If statement to keep the maximum angle value for servos within their mobility range for Y axis
    if (gimbal_angleY[0] > max_angle) { 
      gimbal_angleY[0] = max_angle;
    }
    if (gimbal_angleY[0] < - max_angle){
      gimbal_angleY[0] = - max_angle;
    }
    //----------------------------------------------------------------------------------------------

    //If statement to keep the output value rate of change from exceeding the maximum turn speed for servo in Z axis
    if ((outputZ - gimbal_angleZ[1]) > maxRotationPerStep) { 
      gimbal_angleZ[0] = gimbal_angleZ[1] + maxRotationPerStep;
    }
    else if ((gimbal_angleZ[1] - outputZ) > maxRotationPerStep) { 
      gimbal_angleZ[0] = gimbal_angleZ[1] - maxRotationPerStep;
    }
    else {
      gimbal_angleZ[0] = outputZ;
    }
    //--------------------------------------------------------------------------------------------------------------
        

    //If statement to keep the maximum angle value for servos within their mobility range for Z axis
    if (gimbal_angleZ[0] > max_angle) { 
      gimbal_angleZ[0] = max_angle;
    }
    if (gimbal_angleZ[0] < - max_angle){
      gimbal_angleZ[0] = - max_angle;
    }
    //----------------------------------------------------------------------------------------------


    if (gimbal_angleY[0] >= 0) {
      gimbal_angleIntY = floor(gimbal_angleY[0]);
      gimbal_angleDecimalY = 100*(gimbal_angleY[0]-floor(gimbal_angleY[0]));
    }
    else {
      gimbal_angleIntY = -1*ceil(gimbal_angleY[0]);
      gimbal_angleDecimalY = 100-100*(gimbal_angleY[0]-ceil(gimbal_angleY[0]));
    }
    if (gimbal_angleZ[0] >= 0) {
      gimbal_angleIntZ = floor(gimbal_angleZ[0]);
      gimbal_angleDecimalZ = 100*(gimbal_angleZ[0]-floor(gimbal_angleZ[0]));
    }
    else {
      gimbal_angleIntZ = -1*ceil(gimbal_angleZ[0]);
      gimbal_angleDecimalZ = 100-100*(gimbal_angleZ[0]-ceil(gimbal_angleZ[0]));
    }

    //----------------------------------------------------------------------------------------------

    servoPos1 = 90+findBottomAngleRatio(gimbal_angleY[0])*gimbal_angleY[0]; //Calculates the servo angle based on the given gimbal angle for servo1
    myServo1.write(servoPos1); //Applies the calculated angle for servo1

    servoPos2 = 90+findTopAngleRatio(gimbal_angleZ[0])*gimbal_angleZ[0]; //Calculates the servo angle based on the given gimbal angle for servo2
    myServo2.write(servoPos2); //Applies the calculated angle for servo2

    //----------------------------------------------------------------------------------------------

    //BMP READINGS AND HEX SEPARATION
    altitude = bmp.readAltitude(); //Altitude measurement
    pressure = bmp.readPressure()-90000; //Pressure measurement -- subtracts 90,000Pa from the pressure measurements for easier storage on FRAM chip. Added back in chip reading program

    altitudeLast = 100*(altitude - floor(altitude)); //Separates first 2 digits of altitude measurement
    altitudeMid = 100*(floor(altitude)/100 - floor(floor(altitude)/100)); //Separates middle 2 digits of altitude measurement
    altitudeFirst = floor(floor(altitude)/100); //Separates decimal digits of altitude measurement

    pressureLast = 100*(pressure-floor(pressure)); //Separates first 2 digits of pressure measurement
    pressureMid = 100*(floor(pressure)/100 - floor(floor(pressure)/100)); //Separates middle 2 digits of pressure measurement
    pressureFirst = floor(floor(pressure)/100); //Separates decimal digits of pressure measurement

    //Writing values to FRAM chip -- 22 values stored per measurement loop
    fram.write8(n*22, flightStage); //FLIGHT STAGE
    
    //ORIENTATION
    fram.write8(n*22+1, thetaIntX); //Integer value X-axis
    fram.write8(n*22+2, thetaDecimalX); //Decimal value X-axis
    fram.write8(n*22+3, thetaFlagX); //Positive / Negative Flag X-axis
    fram.write8(n*22+4, thetaIntY); //Integer value Y-axis
    fram.write8(n*22+5, thetaDecimalY); //Decimal value Y-axis
    fram.write8(n*22+6, thetaFlagY); //Positive / Negative Flag Y-axis
    fram.write8(n*22+7, thetaIntZ); //Integer value Z-axis
    fram.write8(n*22+8, thetaDecimalZ); //Decimal value Z-axis
    fram.write8(n*22+9, thetaFlagZ); //Positive / Negative Flag Z-axis

    //GIMBAL ANGLES
    fram.write8(n*22+10, gimbal_angleIntY); //Integer value Y-axis
    fram.write8(n*22+11, gimbal_angleDecimalY); //Decimal value Y-axis
    fram.write8(n*22+12, gimbal_angleIntZ); //Integer value Z-axis
    fram.write8(n*22+13, gimbal_angleDecimalZ); //Decimal value Z-axis

    //SERVO ANGLES
    fram.write8(n*22+14, servoPos1); //Y-axis
    fram.write8(n*22+15, servoPos2); //Z-axis

    //ALTITUDE
    fram.write8(n*22+16, altitudeFirst); //Digits in the hundreds and thousands place
    fram.write8(n*22+17, altitudeMid); //Digits in the Tens and Ones place
    fram.write8(n*22+18, altitudeLast); //Digits in the Tenths and Hundredths place

    //PRESSURE
    fram.write8(n*22+19, pressureFirst); //Digits in the hundreds and thousands place
    fram.write8(n*22+20, pressureMid); //Digits in the Tens and Ones place
    fram.write8(n*22+21, pressureLast); //Digits in the Tenths and Hundredths place
 

    // Print Statement for all relevant information while PID loop is active ------------------------------------------------------------------------------------------------------------------------------------------------------------------
        
    Serial.print(F("n= "));
    Serial.println(n);
//    Serial.print(F(" runtime= "));
//    Serial.print(runtime[n]);
//    Serial.print(F(" ThetaX= "));
//    Serial.print(thetaX);
//    Serial.print(F(" thetaIntX= "));
//    Serial.print(thetaIntX);
//    Serial.print(F(" thetaDecimalX= "));
//    Serial.print(thetaDecimalX);
//    Serial.print(F(" thetaFlagX= "));
//    Serial.println(thetaFlagX);
//
//    Serial.print(F("thetaY= "));
//    Serial.print(thetaY[0]);
//    Serial.print(F(" thetaIntY= "));
//    Serial.print(thetaIntY);
//    Serial.print(F(" thetaDecimalY= "));
//    Serial.print(thetaDecimalY);
//    Serial.print(F(" thetaFlagY= "));
//    Serial.print(thetaFlagY);
//    Serial.print(F(" thetaZ= "));
//    Serial.print(thetaZ[0]);
//    Serial.print(F(" thetaIntZ= "));
//    Serial.print(thetaIntZ);
//    Serial.print(F(" thetaDecimalZ= "));
//    Serial.print(thetaDecimalZ);
//    Serial.print(F(" thetaFlagZ= "));
//    Serial.println(thetaFlagZ);
      
//    Serial.print(F("gimbal_angleY= "));
//    Serial.print(gimbal_angleY[0]);
//    Serial.print(F(" gimbal_angleIntY= "));
//    Serial.print(gimbal_angleIntY);
//    Serial.print(F(" gimbal_angleDecimalY= "));
//    Serial.print(gimbal_angleDecimalY);
//    Serial.print(F(" gimbal_angleZ= "));
//    Serial.println(gimbal_angleZ[0]);
//    Serial.print(F(" gimbal_angleIntZ= "));
//    Serial.print(gimbal_angleIntZ);
//    Serial.print(F(" gimbal_angleDecimalZ= "));
//    Serial.print(gimbal_angleDecimalZ);
//    Serial.print(F(" Servo angleY= "));
//    Serial.print(servoPos1);
//    Serial.print(F(" Servo angleZ= "));
//    Serial.println(servoPos2);

//    Serial.print(F("altitude= "));
//    Serial.print(altitude);
//    Serial.print(F(" altitudeFirst= "));
//    Serial.print(altitudeFirst);
//    Serial.print(F(" altitudeMid= "));
//    Serial.print(altitudeMid);
//    Serial.print(F(" altitudeLast= "));
//    Serial.println(altitudeLast);
//
//    Serial.print(F("pressure= "));
//    Serial.print(pressure);
//    Serial.print(F(" pressureFirst= "));
//    Serial.print(pressureFirst);
//    Serial.print(F(" pressureMid= "));
//    Serial.print(pressureMid);
//    Serial.print(F(" pressureLast= "));
//    Serial.println(pressureLast);
        
    //delay(BNO055_SAMPLERATE_DELAY_MS);
    //-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  }

}




void loop(void)
{
}
