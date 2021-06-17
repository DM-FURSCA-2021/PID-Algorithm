#include <fstream>
#include <iostream>
#include <cmath>
#include <math.h>
#include <stdio.h>

using namespace std;

int forceF15(double motor_time) { //Force function for Estes F15 motor thrust curve
    if (motor_time < 100) {
        return 2.5;
    }
    else if (motor_time < 200) {
        return 7.5;
    }
    else if (motor_time < 300) {
        return 13;
    }
    else if (motor_time < 400) {
        return 20;
    }
    else if (motor_time < 500) {
        return 23.75;
    }
    else if (motor_time < 600) {
        return 20.75;
    }
    else if (motor_time < 700) {
        return 18;
    }
    else if (motor_time < 800) {
        return 17.25;
    }
    else if (motor_time < 900) {
        return 16.625;
    }
    else if (motor_time < 1000) {
        return 15.875;
    }
    else if (motor_time < 1100) {
        return 15.375;
    }
    else if (motor_time < 1200) {
        return 15.125;
    }
    else if (motor_time < 1300) {
        return 15;
    }
    else if (motor_time < 1400) {
        return 14.875;
    }
    else if (motor_time < 1500) {
        return 15;
    }
    else if (motor_time < 1600) {
        return 14.875;
    }
    else if (motor_time < 1700) {
        return 14.75;
    }
    else if (motor_time < 1800) {
        return 14.5;
    }
    else if (motor_time < 1900) {
        return 14.375;
    }
    else if (motor_time < 2000) {
        return 14.25;
    }
    else if (motor_time < 2100) {
        return 13.9375;
    }
    else if (motor_time < 2200) {
        return 13.8125;
    }
    else if (motor_time < 2300) {
        return 13.625;
    }
    else if (motor_time < 2400) {
        return 13.375;
    }
    else if (motor_time < 2500) {
        return 13.3125;
    }
    else if (motor_time < 2600) {
        return 13.25;
    }
    else if (motor_time < 2700) {
        return 13.125;
    }
    else if (motor_time < 2800) {
        return 13.125;
    }
    else if (motor_time < 2900) {
        return 13.25;
    }
    else if (motor_time < 3000) {
        return 13.25;
    }
    else if (motor_time < 3100) {
        return 13.375;
    }
    else if (motor_time < 3200) {
        return 13.375;
    }
    else if (motor_time < 3300) {
        return 13.25;
    }
    else if (motor_time < 3400) {
        return 13.125;
    }
    else if (motor_time < 3500) {
        return 6.125;
    }
}

int forceE12(double motor_time) {//Force function for Estes E12 motor thrust curve
    if (motor_time < 100) {
        return 5;
    }
    else if (motor_time <200) {
        return 17;
    }
    else if (motor_time < 300) {
        return 28.5;
    }
    else if (motor_time < 400) {
        return 23.25;
    }
    else if (motor_time < 500) {
        return 13;
    }
    else if (motor_time < 600) {
        return 12.125;
    }
    else if (motor_time < 700) {
        return 11.3125;
    }
    else if (motor_time < 800) {
        return 10.4375;
    }
    else if (motor_time < 900) {
        return 9.875;
    }
    else if (motor_time < 1000) {
        return 9.625;
    }
    else if (motor_time < 1100) {
        return 9.6875;
    }
    else if (motor_time < 1200) {
        return 9.6875;
    }
    else if (motor_time < 1300) {
        return 9.625;
    }
    else if (motor_time < 1400) {
        return 9.8125;
    }
    else if (motor_time < 1500) {
        return 9.875;
    }
    else if (motor_time < 1600) {
        return 9.875;
    }
    else if (motor_time < 1700) {
        return 9.6875;
    }
    else if (motor_time < 1800) {
        return 9.6875;
    }
    else if (motor_time < 1900) {
        return 9.875;
    }
    else if (motor_time < 2000) {
        return 9.6875;
    }
    else if (motor_time < 2100) {
        return 9.5;
    }
    else if (motor_time < 2200) {
        return 9.5625;
    }
    else if (motor_time < 2300) {
        return 9.3125;
    }
    else if (motor_time < 2400) {
        return 8.0625;
    }
    else if (motor_time < 2500) {
        return 2;
    }
}

int forceD12(double motor_time) { //Force function for Estes D12 motor thrust curve
    if (motor_time < 100) {
        return 3.375;
    }
    else if (motor_time < 200) {
        return 13.75;
    }
    else if (motor_time < 300) {
        return 25;
    }
    else if (motor_time < 400) {
        return 16.5;
    }
    else if (motor_time < 500) {
        return 11;
    }
    else if (motor_time < 600) {
        return 10;
    }
    else if (motor_time < 700) {
        return 9.4375;
    }
    else if (motor_time < 800) {
        return 9.125;
    }
    else if (motor_time < 900) {
        return 8.875;
    }
    else if (motor_time < 1000) {
        return 8.625;
    }
    else if (motor_time < 1100) {
        return 8.375;
    }
    else if (motor_time < 1200) {
        return 8.125;
    }
    else if (motor_time < 1300) {
        return 8;
    }
    else if (motor_time < 1400) {
        return 7.625;
    }
    else if (motor_time < 1500) {
        return 7.375;
    }
    else if (motor_time < 1600) {
        return 7.25;
    }
    else if (motor_time < 1700) {
        return 1.75;
    }
}

int forceF10(double motor_time) { //Force function for Apogee F10 motor thrust curve
    if (motor_time < 100) {
        return 25.5;
    }
    else if (motor_time < 200) {
        return 23.5;
    }
    else if (motor_time < 300) {
        return 22.75;
    }
    else if (motor_time < 400) {
        return 22.5;
    }
    else if (motor_time < 500) {
        return 21.25;
    }
    else if (motor_time < 600) {
        return 20;
    }
    else if (motor_time < 700) {
        return 17.5;
    }
    else if (motor_time < 800) {
        return 16.25;
    }
    else if (motor_time < 900) {
        return 15.125;
    }
    else if (motor_time < 1000) {
        return 14.875;
    }
    else if (motor_time < 1100) {
        return 14.25;
    }
    else if (motor_time < 1200) {
        return 13.625;
    }
    else if (motor_time < 1300) {
        return 13.25;
    }
    else if (motor_time < 1400) {
        return 12.875;
    }
    else if (motor_time < 1500) {
        return 12.125;
    }
    else if (motor_time < 1600) {
        return 11.875;
    }
    else if (motor_time < 1700) {
        return 11.625;
    }
    else if (motor_time < 1800) {
        return 11.125;
    }
    else if (motor_time < 1900) {
        return 10.75;
    }
    else if (motor_time < 2000) {
        return 10.375;
    }
    else if (motor_time < 2100) {
        return 10.125;
    }
    else if (motor_time < 2200) {
        return 10;
    }
    else if (motor_time < 2300) {
        return 9.675;
    }
    else if (motor_time < 2400) {
        return 9.625;
    }
    else if (motor_time < 2500) {
        return 9.5;
    }
    else if (motor_time < 2600) {
        return 9.425;
    }
    else if (motor_time < 2700) {
        return 9.375;
    }
    else if (motor_time < 2800) {
        return 9.25;
    }
    else if (motor_time < 2900) {
        return 9.125;
    }
    else if (motor_time < 3000) {
        return 9.075;
    }
    else if (motor_time < 3100) {
        return 9;
    }
    else if (motor_time < 3200) {
        return 8.9375;
    }
    else if (motor_time < 3300) {
        return 8.875;
    }
    else if (motor_time < 3400) {
        return 8.6875;
    }
    else if (motor_time < 3500) {
        return 8.425;
    }
    else if (motor_time < 3600) {
        return 8.375;
    }
    else if (motor_time < 3700) {
        return 8.25;
    }
    else if (motor_time < 3800) {
        return 8.875;
    }
    else if (motor_time < 3900) {
        return 9.5;
    }
    else if (motor_time < 4000) {
        return 9.5;
    }
    else if (motor_time < 4100) {
        return 9.5;
    }
    else if (motor_time < 4200) {
        return 9.4375;
    }
    else if (motor_time < 4300) {
        return 9.4375;
    }
    else if (motor_time < 4400) {
        return 9.375;
    }
    else if (motor_time < 4500) {
        return 9.25;
    }
    else if (motor_time < 4600) {
        return 9.125;
    }
    else if (motor_time < 4700) {
        return 9;
    }
    else if (motor_time < 4800) {
        return 8.875;
    }
    else if (motor_time < 4900) {
        return 8.75;
    }
    else if (motor_time < 5000) {
        return 8.625;
    }
    else if (motor_time < 5100) {
        return 8.5;
    }
    else if (motor_time < 5200) {
        return 8.5;
    }
    else if (motor_time < 5300) {
        return 8.4375;
    }
    else if (motor_time < 5400) {
        return 8.375;
    }
    else if (motor_time < 5500) {
        return 8.375;
    }
    else if (motor_time < 5600) {
        return 8.375;
    }
    else if (motor_time < 5700) {
        return 8.375;
    }
    else if (motor_time < 5800) {
        return 8.25;
    }
    else if (motor_time < 5900) {
        return 8.1875;
    }
    else if (motor_time < 6000) {
        return 8;
    }
    else if (motor_time < 6100) {
        return 7.9375;
    }
    else if (motor_time < 6200) {
        return 8.125;
    }
    else if (motor_time < 6300) {
        return 8.25;
    }
    else if (motor_time < 6400) {
        return 7.9375;
    }
    else if (motor_time < 6500) {
        return 7.25;
    }
    else if (motor_time < 6600) {
        return 6.625;
    }
    else if (motor_time < 6700) {
        return 6.125;
    }
    else if (motor_time < 6800) {
        return 5.25;
    }
    else if (motor_time < 6900) {
        return 4;
    }
    else if (motor_time < 7000) {
        return 2.25;
    }
    else if (motor_time < 7100) {
        return 1;
    }
}

int gnuplotstart(void) { //Function that resets the PIDdata.txt file in preparation for new data

    FILE *fp = NULL;
    FILE *gnupipe = NULL;

    fp = fopen ("PIDdata.txt", "w");
    return 0;
}

int gnuplotgraph() { //Function that graphs the data collected at the very end

    FILE *gnupipe = NULL;
    gnupipe = _popen("gnuplot -persistent", "w");

    char *GnuCommands [] = {"cd 'C:\\Users\\mdjmd\\OneDrive\\Documents\\FURSCA\\Flight Main'", "plot 'PIDdata.txt' using 1:3 with lines title 'Gimbal Angle', 'PIDdata.txt' using 1:2 with lines title 'Rocket Pitch Angle'", "set xlabel 'Time (ms)'", "set ylabel 'Angle (deg)'"};
    for (int i = 0; i < 4; i++) {
        fprintf(gnupipe, "%s\n", GnuCommands[i]);
    }
}

int main() {

    //Choosing text data file to write to
    ofstream myfile;
    myfile.open ("PIDdata.txt");

    gnuplotstart();

    double pi = atan(1)*4;
    int sim_time = 350; // sim_time * time_step = how much time in s
    double theta_i = 10.0; //initial rocket pitch offset angle
    double inertia = 0.02; //TEMPORARY VALUE TO BE REPLACED WITH ACTUAL CALCULATED VALUE FOR ROCKET'S INERTIA
    double time_step = 0.01; //update rate on BNO055 IMU ~10ms
    double max_angle = 5.0; //max angle for gimbal 
    double maxRotationPerStep = 20/0.1*time_step;  //TEMPORARY VALUE - 100ms/60deg - TO BE REPLACED WITH MEASURED VALUE
    double d = 0.2; //TEMP VALUE TO BE REPLACED WITH DISTANCE BETWEEN ROCKET CENTER OF GRAVITY AND GIMBAL
    double theta0 = -0*pi/180; //0 degree pitch target

    //F15 PID gains------
    double KP = 900.0;
    double KI = 0.05;
    double KD = 120.0;

    //E12 PID gains------
    // double KP = 520.0;
    // double KI = 0.05;
    // double KD = 55.0;

    //D12 PID gains------
    // double KP = 890.0;
    // double KI = 0.05;
    // double KD = 110.0;

    //F10 PID gains------
    //double KP = 50.0;
    //double KI = 0.001;
    //double KD = 5.0;


    double theta[sim_time]; //rocket pitch angle
    double time[sim_time];
    for (int i = 0; i <= sim_time; i++) {
        time[i] = i*10;
    }
    double torque[sim_time];
    double gimbal_angle[sim_time]; //gimbal angle

    theta[0] = theta_i*pi/180; //deg to radian conversion
    theta[1] = theta_i*pi/180; 

    double runningSum = theta[0]+theta[1];
    double proportional_error = 0.0;
    double integral_error = 0.0;
    double derivative_error = 0.0;
    double output = 0.0;



    for (int n = 2; n <= sim_time; n++) {
	    //update governing dynamics
        torque[n-1] = d*forceF15(n*time_step)*sin( pi*gimbal_angle[n-1]/180);
	    //using delayed torque to account for propagation time
        theta[n] = 2*theta[n-1] - theta[n-2] + torque[n-2]*time_step*time_step/inertia; //+ 0.0005*numpy.random.standard_normal()

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
		
        //t.sleep(time_step)
        cout << "n=" << n << " theta=" << theta[n] << " gimbal_angle=" << gimbal_angle[n] << endl;
        
        //Writing to text data file
        myfile << n*10 << "\t" << theta[n]*180/pi << "\t\t" << gimbal_angle[n] << endl;
    }

    cout << "Sim ended" << endl;

    myfile.close();
    gnuplotgraph();

}