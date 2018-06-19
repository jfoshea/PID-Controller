# PID-Controller 

## Overview 
This project implements a PID controller in C++ to maneuver a vehicle around the track in a simulator. The simulator will provide the cross track error (CTE) and the velocity (mph) in order to compute the appropriate steering angle. The PID controller also has to be tuned either manually or automatically to find the appropriate coefficients for Kp, Ki, Kd in order to minimize cte, reduce oscillations and compensate for sudden changes to the system.

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros

## Build Instructions 
1. Clone the PID-Controller git repository
    ```  
    $ git clone https://github.com/jfoshea/PID-Controller.git
    ```
2. This project involves the Term 2 Simulator which can be downloaded here [link](https://github.com/udacity/self-driving-car-sim/releases)

3. Build the project using cmake or using the scipts below 
    ```  
    $ ./clean.sh 
    $ ./build.sh 
    ```
4. Run the PID-Controller 
    1. Launch the simulator and select PID-Controller 
    2. Change to build directory
    ```  
    $ cd build 
    ```
    2. Run the pid controller with default params (Kp=0.1, Ki=0.0003, Kd=0.5) 
    ```  
    $ ./pid
    ```
    OR
5. Run the PID-Controller with user defined (Kp, Ki, Kd) 
    ```  
    For example:
    $ ./pid 0.1 0.0001 0.5 
    ```
