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
#include "ArduinoPID.h" 

using namespace std;

/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 10;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x29);

int servoPin1=7;
int servoPos1=90;

Servo myServo1;

void setup(void)
{

  
  Serial.begin(115200);
  
  myServo1.attach(servoPin1);
  myServo1.write(servoPos1);
  
  if(!bno.begin())
    {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  bno.setExtCrystalUse(true);

//  for (int n = 2; n <= 100; n++) {
//
//    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
//    Serial.print("n=");Serial.print(n);Serial.print("\t\t");Serial.println(euler.y());
//    delay(BNO055_SAMPLERATE_DELAY_MS);
//  }

    //F15 PID gains------
    double KP = 900.0;
    double KI = 0.05;
    double KD = 120.0;

    for (int i = 0; i <= sim_time; i++) {
        runtime[i] = i*10;
    }

    for (int i = 0; i < 2; i++) { //SET INITIAL THETA VALUES
      
      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      runtime[i] = i*10;
      theta[i] = euler.y();  //*pi/180; //deg to radian conversion
      Serial.println(theta[i]);
    }

    double runningSum = theta[0]+theta[1];
    double proportional_error = 0.0;
    double integral_error = 0.0;
    double derivative_error = 0.0;
    double output = 0.0;

for (int n = 2; n <= sim_time; n++) { //ACTUAL PID
        
      //update governing dynamics
        torque[n-1] = d*forceF15(n*time_step)*sin( pi*gimbal_angle[n-1]/180);
      //OLD THETA CALCULATION -- using delayed torque to account for propagation time
      //theta[n] = 2*theta[n-1] - theta[n-2] + torque[n-2]*time_step*time_step/inertia; //+ 0.0005*numpy.random.standard_normal()
        imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
        theta[n] = euler.y();

      //PID control
        runningSum = runningSum + (theta[n]-theta0);

        proportional_error = theta[n]-theta0;
        integral_error = runningSum;
        derivative_error = (theta[n]-theta[n-1])/time_step;

        output = - KP*proportional_error - KI*integral_error - KD*derivative_error;
        //output = - KP*(theta[n]-theta0) - KD*(theta[n]-theta[n-1])/time_step - KI*runningSum*time_step
        if ((output - gimbal_angle[n-1]) > maxRotationPerStep) {
            gimbal_angle[n] = gimbal_angle[n-1] + maxRotationPerStep;
        }
        else if ((gimbal_angle[n-1] - output) > maxRotationPerStep) {
            gimbal_angle[n] = gimbal_angle[n-1] - maxRotationPerStep;
        }
        else {
            gimbal_angle[n] = output;
        }
        if (gimbal_angle[n] > max_angle) {
            gimbal_angle[n] = max_angle;
        }
        if (gimbal_angle[n] < - max_angle){
            gimbal_angle[n] = - max_angle;
        }

        servoPos1 = 90+5*gimbal_angle[n];
        myServo1.write(servoPos1);

    
        Serial.print("n= ");Serial.print(n);Serial.print(" runtime= ");Serial.print(runtime[n]);Serial.print(" theta= ");Serial.print(theta[n]);Serial.print(" gimbal angle= ");Serial.print(gimbal_angle[n]);Serial.print(" servo angle ");Serial.println(servoPos1);
        delay(BNO055_SAMPLERATE_DELAY_MS);
    }









  
}

















void loop() {
}
