#include <cmath>
#include <math.h>


short sim_time = 350;

float thetaY[350]; //Rocket pitch angle Y axis
float thetaZ[350]; //Rocket pitch angle Z axis
short runtime[350];

float gimbal_angleY[350]; //Gimbal Angle Y axis
float gimbal_angleZ[350]; //Gimbal Angle Z axis

double pi = atan(1)*4;

float max_angle = 8.0; //max angle for gimbal 
float time_step = 0.01; //update rate on BNO055 IMU ~10ms
float maxRotationPerStep = 20/0.1*time_step;  //TEMPORARY VALUE - 100ms/60deg - TO BE REPLACED WITH MEASURED VALUE

float theta0 = -0*pi/180; //0 degree pitch target


