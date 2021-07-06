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
//#include <SPI.h>

/*INCLUDE FULL FILE PATHWAY IF THE PROGRAM CANNOT FIND THE ARDUINOPID.h FILE*/
#include "ArduinoPID.h"

Adafruit_FRAM_I2C fram     = Adafruit_FRAM_I2C();

short servoPos1 = 0;
short servoPos2 = 0;

void setup(void) {
  Serial.begin(115200);
  
  if (fram.begin()) {
    Serial.println("Found I2C FRAM");
  } else {
    Serial.println("I2C FRAM not identified ... check your connections?\r\n");
    Serial.println("Will continue in case this processor doesn't support repeated start\r\n");
  }

  uint8_t value;
  for (uint16_t n = 0; n < 1020; n++) {

    thetaX = fram.read8(n*22+1) + .01*fram.read8(n*22+2);
    if (fram.read8(n*22+3) == 1) {
      thetaX = thetaX*(-1);
    }
    thetaY[0] = fram.read8(n*22+4) + .01*fram.read8(n*22+5);
    if (fram.read8(n*22+6) == 1) {
      thetaY[0] = thetaY[0]*(-1);
    }
    thetaZ[0] = fram.read8(n*22+7) + .01*fram.read8(n*22+8);
    if (fram.read8(n*22+9) == 1) {
      thetaZ[0] = thetaZ[0]*(-1);
    }

    flightStage = fram.read8(n*22);

    gimbal_angleDecimalY = fram.read8(n*22+11);
    gimbal_angleDecimalZ = fram.read8(n*22+13);

    if (gimbal_angleDecimalY >= 100) {
      gimbal_angleDecimalY = (gimbal_angleDecimalY-100);
      gimbal_angleY[0] = -1*(fram.read8(n*22+12) + .01*gimbal_angleDecimalY);
    }
    else {
      gimbal_angleY[0] = fram.read8(n*22+10) + .01*gimbal_angleDecimalY;
    }
    
    if (gimbal_angleDecimalZ >= 100) {
      gimbal_angleDecimalZ = (gimbal_angleDecimalZ-100);
      gimbal_angleZ[0] = -1*(fram.read8(n*22+12) + .01*gimbal_angleDecimalZ);
    }
    else {
      gimbal_angleZ[0] = fram.read8(n*22+12) + .01*gimbal_angleDecimalZ;
    }


    servoPos1 = fram.read8(n*22+14);
    servoPos2 = fram.read8(n*22+15);

    altitude = 100*fram.read8(n*22+16) + fram.read8(n*22+17) + .01*fram.read8(n*22+18);
    pressure = 90000+100*fram.read8(n*22+19) + fram.read8(n*22+20) + .01*fram.read8(n*22+21);

    Serial.print(F("n= "));
    Serial.print(n);
    Serial.print(F(" flightStage= "));
    Serial.print(flightStage);
    Serial.print(F(" thetaX= "));
    Serial.print(thetaX);
    Serial.print(F(" thetaY= "));
    Serial.print(thetaY[0]);
    Serial.print(F(" thetaZ= "));
    Serial.println(thetaZ[0]);
    
    Serial.print(F("gimbal_angleY= "));
    Serial.print(gimbal_angleY[0]);
    Serial.print(F(" gimbal_angleZ= "));
    Serial.print(gimbal_angleZ[0]);
    Serial.print(F(" servoPos1= "));
    Serial.print(servoPos1);
    Serial.print(F(" servoPos2= "));
    Serial.print(servoPos2);
    Serial.print(F(" altitude= "));
    Serial.print(altitude);
    Serial.print(F(" pressure= "));
    Serial.println(pressure);

    
  }

}




void loop() {
}
