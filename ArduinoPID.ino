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

/*INCLUDE FULL FILE PATHWAY IF THE PROGRAM CANNOT FIND THE ARDUINOPID.h FILE*/
#include "C:\\Users\\mdjmd\\OneDrive\\Documents\\FURSCA\\Flight Main\\ArduinoPID.h"

using namespace std;

/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 10;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x29);

int servoPin1=6;
int servoPos1=90;

int servoPin2=10;
int servoPos2=90;

Servo myServo1;
Servo myServo2;

void setup(void)
{

  
  Serial.begin(115200);
  
  myServo1.attach(servoPin1);
  myServo1.write(servoPos1);
  myServo2.attach(servoPin2);
  myServo2.write(servoPos2);
  
  if(!bno.begin())
    {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  bno.setExtCrystalUse(true);


    //F15 PID gains------
    double KP = 900;
    double KI = 0.05;
    double KD = 210;

    for (short i = 0; i <= sim_time; i++) {
        runtime[i] = i*10;
    }

    for (short i = 0; i < 2; i++) { //SET INITIAL THETA VALUES
      
      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      runtime[i] = i*10;
      thetaY[i] = euler.y();
      thetaZ[i] = euler.z();
      Serial.println(thetaY[i]);
      Serial.println(thetaZ[i]);
    }

    float runningSumY = thetaY[0]+thetaY[1];
    float proportional_errorY = 0.0;
    float integral_errorY = 0.0;
    float derivative_errorY = 0.0;
    float outputY = 0.0;

    float runningSumZ = thetaZ[0]+thetaZ[1];
    float proportional_errorZ = 0.0;
    float integral_errorZ = 0.0;
    float derivative_errorZ = 0.0;
    float outputZ = 0.0;

for (int n = 2; n <= sim_time; n++) { //ACTUAL PID
        
      
        imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
        thetaY[n] = euler.y();
        thetaZ[n] = euler.z();

      //PID control
        runningSumY = runningSumY + (thetaY[n]-theta0);
        runningSumZ = runningSumZ + (thetaZ[n]-theta0);

        proportional_errorY = thetaY[n]-theta0;
        integral_errorY = runningSumY;
        derivative_errorY = (thetaY[n]-thetaY[n-1])/time_step;

        proportional_errorZ = thetaZ[n]-theta0;
        integral_errorZ = runningSumZ;
        derivative_errorZ = (thetaZ[n]-thetaZ[n-1])/time_step;

        outputY = - KP*proportional_errorY - KI*integral_errorY - KD*derivative_errorY;
        outputZ = - KP*proportional_errorZ - KI*integral_errorZ - KD*derivative_errorZ;

        if ((outputY - gimbal_angleY[n-1]) > maxRotationPerStep) {
            gimbal_angleY[n] = gimbal_angleY[n-1] + maxRotationPerStep;
        }
        else if ((gimbal_angleY[n-1] - outputY) > maxRotationPerStep) {
            gimbal_angleY[n] = gimbal_angleY[n-1] - maxRotationPerStep;
        }
        else {
            gimbal_angleY[n] = outputY;
        }
        
        if (gimbal_angleY[n] > max_angle) {
            gimbal_angleY[n] = max_angle;
        }
        if (gimbal_angleY[n] < - max_angle){
            gimbal_angleY[n] = - max_angle;
        }

        if ((outputZ - gimbal_angleZ[n-1]) > maxRotationPerStep) {
            gimbal_angleZ[n] = gimbal_angleZ[n-1] + maxRotationPerStep;
        }
        else if ((gimbal_angleZ[n-1] - outputZ) > maxRotationPerStep) {
            gimbal_angleZ[n] = gimbal_angleZ[n-1] - maxRotationPerStep;
        }
        else {
            gimbal_angleZ[n] = outputZ;
        }
        
        if (gimbal_angleZ[n] > max_angle) {
            gimbal_angleZ[n] = max_angle;
        }
        if (gimbal_angleZ[n] < - max_angle){
            gimbal_angleZ[n] = - max_angle;
        }

        servoPos1 = 90+5*gimbal_angleY[n];
        myServo1.write(servoPos1);

        servoPos2 = 90+5*gimbal_angleZ[n];
        myServo2.write(servoPos2);

    
        Serial.print(F("n= "));Serial.print(n);Serial.print(F(" runtime= "));Serial.print(runtime[n]);Serial.print(F(" thetaY= "));Serial.print(thetaY[n]);Serial.print(F(" thetaZ= "));Serial.print(thetaZ[n]);
        Serial.print(F(" G-angleY= "));Serial.print(gimbal_angleY[n]);Serial.print(F(" G-angleZ= "));Serial.print(gimbal_angleZ[n]);Serial.print(F(" S-angleY= "));Serial.print(servoPos1);Serial.print(F(" S-angleZ= "));Serial.println(servoPos2);
        delay(BNO055_SAMPLERATE_DELAY_MS);
    }









  
}

















void loop() {
}
