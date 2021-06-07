import matplotlib.pyplot as plt
import time as t
import numpy as np
from pylab import *
#Global Variables--------

sim_time = 340 # sim_time * time_step = how much time in s
theta_i = 5 #initial rocket pitch offset angle
inertia = 0.02 #TEMPORARY VALUE TO BE REPLACED WITH ACTUAL CALCULATED VALUE FOR ROCKET'S INERTIA
time_step = 0.01 #update rate on BNO055 IMU ~10ms
max_angle = 5 #max angle for gimbal 
maxRotationPerStep = 20/0.1*time_step  #100ms/60deg per spec sheet, this is the max change in servo angle per step -- TO BE REPLACED WITH MAX CHANGE IN SERVO ANGLE PER TIME STEP
d = 0.2 #TEMP VALUE TO BE REPLACED WITH DISTANCE BETWEEN ROCKET CENTER OF GRAVITY AND GIMBAL

theta0 = -0*np.pi/180


#PID gains------
KP = 900
KI = 0.05
KD = 120

theta = np.zeros(sim_time) #rocket pitch angle
time = [i*10 for i in range(sim_time)]
torque = np.zeros(sim_time)
gimbal_angle = np.zeros(sim_time) #gimbal angle

theta[0] = theta_i*pi/180.   #must be in radians
theta[1] = theta_i*pi/180.   #deg


def forceF15(time):
    if time < 100:
        return 2.5
    elif time <200:
        return 7.5
    elif time < 300:
        return 13
    elif time < 400:
        return 20
    elif time < 500:
        return 23.75
    elif time < 600:
        return 20.75
    elif time < 700:
        return 18
    elif time < 800:
        return 17.25
    elif time < 900:
        return 16.625
    elif time < 1000:
        return 15.875
    elif time < 1100:
        return 15.375
    elif time < 1200:
        return 15.125
    elif time < 1300:
        return 15
    elif time < 1400:
        return 14.875
    elif time < 1500:
        return 15
    elif time < 1600:
        return 14.875
    elif time < 1700:
        return 14.75
    elif time < 1800:
        return 14.5
    elif time < 1900:
        return 14.375
    elif time < 2000:
        return 14.25
    elif time < 2100:
        return 13.9375
    elif time < 2200:
        return 13.8125
    elif time < 2300:
        return 13.625
    elif time < 2400:
        return 13.375
    elif time < 2500:
        return 13.3125
    elif time < 2600:
        return 13.25
    elif time < 2700:
        return 13.125
    elif time < 2800:
        return 13.125
    elif time < 2900:
        return 13.25
    elif time < 3000:
        return 13.25
    elif time < 3100:
        return 13.375
    elif time < 3200:
        return 13.375
    elif time < 3300:
        return 13.25
    elif time < 3400:
        return 13.125
    elif time < 3500:
        return 6.125

def forceE12(time):
    if time < 100:
        return 5
    elif time <200:
        return 17
    elif time < 300:
        return 28.5
    elif time < 400:
        return 23.25
    elif time < 500:
        return 13
    elif time < 600:
        return 12.125
    elif time < 700:
        return 11.3125
    elif time < 800:
        return 10.4375
    elif time < 900:
        return 9.875
    elif time < 1000:
        return 9.625
    elif time < 1100:
        return 9.6875
    elif time < 1200:
        return 9.6875
    elif time < 1300:
        return 9.625
    elif time < 1400:
        return 9.8125
    elif time < 1500:
        return 9.875
    elif time < 1600:
        return 9.875
    elif time < 1700:
        return 9.6875
    elif time < 1800:
        return 9.6875
    elif time < 1900:
        return 9.875
    elif time < 2000:
        return 9.6875
    elif time < 2100:
        return 9.5
    elif time < 2200:
        return 9.5625
    elif time < 2300:
        return 9.3125
    elif time < 2400:
        return 8.0625
    elif time < 2500:
        return 2

def forceD12(time):
    if time < 100:
        return 3.375
    elif time <200:
        return 13.75
    elif time < 300:
        return 25
    elif time < 400:
        return 16.5
    elif time < 500:
        return 11
    elif time < 600:
        return 10
    elif time < 700:
        return 9.4375
    elif time < 800:
        return 9.125
    elif time < 900:
        return 8.875
    elif time < 1000:
        return 8.625
    elif time < 1100:
        return 8.375
    elif time < 1200:
        return 8.125
    elif time < 1300:
        return 8
    elif time < 1400:
        return 7.625
    elif time < 1500:
        return 7.375
    elif time < 1600:
        return 7.25
    elif time < 1700:
        return 1.75

def forceF10(time):
    if time < 100:
        return 25.5
    elif time <200:
        return 23.5
    elif time < 300:
        return 22.75
    elif time < 400:
        return 22.5
    elif time < 500:
        return 21.25
    elif time < 600:
        return 20
    elif time < 700:
        return 17.5
    elif time < 800:
        return 16.25
    elif time < 900:
        return 15.125
    elif time < 1000:
        return 14.875
    elif time < 1100:
        return 14.25
    elif time < 1200:
        return 13.625
    elif time < 1300:
        return 13.25
    elif time < 1400:
        return 12.875
    elif time < 1500:
        return 12.125
    elif time < 1600:
        return 11.875
    elif time < 1700:
        return 11.625
    elif time < 1800:
        return 11.125
    elif time < 1900:
        return 10.75
    elif time < 2000:
        return 10.375
    elif time < 2100:
        return 10.125
    elif time < 2200:
        return 10
    elif time < 2300:
        return 9.675
    elif time < 2400:
        return 9.625
    elif time < 2500:
        return 9.5
    elif time < 2600:
        return 9.425
    elif time < 2700:
        return 9.375
    elif time < 2800:
        return 9.25
    elif time < 2900:
        return 9.125
    elif time < 3000:
        return 9.075
    elif time < 3100:
        return 9
    elif time < 3200:
        return 8.9375
    elif time < 3300:
        return 8.875
    elif time < 3400:
        return 8.6875
    elif time < 3500:
        return 8.425
    elif time < 3600:
        return 8.375
    elif time < 3700:
        return 8.25
    elif time < 3800:
        return 8.875
    elif time < 3900:
        return 9.5
    elif time < 4000:
        return 9.5
    elif time < 4100:
        return 9.5
    elif time < 4200:
        return 9.4375
    elif time < 4300:
        return 9.4375
    elif time < 4400:
        return 9.375
    elif time < 4500:
        return 9.25
    elif time < 4600:
        return 9.125
    elif time < 4700:
        return 9
    elif time < 4800:
        return 8.875
    elif time < 4900:
        return 8.75
    elif time < 5000:
        return 8.625
    elif time < 5100:
        return 8.5
    elif time < 5200:
        return 8.5
    elif time < 5300:
        return 8.4375
    elif time < 5400:
        return 8.375
    elif time < 5500:
        return 8.375
    elif time < 5600:
        return 8.375
    elif time < 5700:
        return 8.375
    elif time < 5800:
        return 8.25
    elif time < 5900:
        return 8.1875
    elif time < 6000:
        return 8
    elif time < 6100:
        return 7.9375
    elif time < 6200:
        return 8.125
    elif time < 6300:
        return 8.25
    elif time < 6400:
        return 7.9375
    elif time < 6500:
        return 7.25
    elif time < 6600:
        return 6.625
    elif time < 6700:
        return 6.125
    elif time < 6800:
        return 5.25
    elif time < 6900:
        return 4
    elif time < 7000:
        return 2.25
    elif time < 7100:
        return 1


def PID():
    runningSum = theta[0]+theta[1]
    proportional_error = 0
    integral_error = 0
    derivative_error = 0
    output = 0
    for n in range(2,sim_time):
	    #update governing dynamics
        torque[n-1] = d*forceF15(n*time_step)*sin( pi*gimbal_angle[n-1]/180)
	    #using delayed torque to account for propagation time
        theta[n] = 2*theta[n-1] - theta[n-2] + torque[n-2]*time_step*time_step/inertia #+ 0.0005*numpy.random.standard_normal()

	    #PID control
        runningSum = runningSum + (theta[n]-theta0)

        proportional_error = theta[n]-theta0
        integral_error = runningSum
        derivative_error = (theta[n]-theta[n-1])/time_step

        output = - KP*proportional_error - KI*integral_error - KD*derivative_error
        #output = - KP*(theta[n]-theta0) - KD*(theta[n]-theta[n-1])/time_step - KI*runningSum*time_step
        if((output - gimbal_angle[n-1]) > maxRotationPerStep):
            gimbal_angle[n] = gimbal_angle[n-1] + maxRotationPerStep
        elif((gimbal_angle[n-1] - output) > maxRotationPerStep):
            gimbal_angle[n] = gimbal_angle[n-1] - maxRotationPerStep
        else:
            gimbal_angle[n] = output
		 
        if( gimbal_angle[n] > max_angle):
            gimbal_angle[n] = max_angle
        if( gimbal_angle[n] < - max_angle):
            gimbal_angle[n] = - max_angle
		
        #t.sleep(time_step)
        print("n=" + str(n) + " theta=" + str(theta[n]) + " gimbal_angle=" + str(gimbal_angle[n]))


def main():

    PID()

    plt.plot(time, 180/np.pi*theta, 'k')
    plt.grid()
    plt.plot(time, gimbal_angle, 'b')
    plt.xlabel('time [ms]')
    plt.show()

    print("Sim ended")

main()