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
## Tuning PID Controller & Reflections 
The goal of tuning a PID-Controller is to adjust its output in such as way as to minimize the error as quickly as possible, minimize overshoot, and then maintain a stable system. A PID Controller can be tuned manually or automatically. Some known methods such as ziegler-nichols method have been successfully used to tune a PID Controller.  [link](https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method). Another method is coordinate ascent or "Twiddle" which is an iterative approach to finding Kp,Ki,Kd that minimizes error below a specficied tolerance level. The algorithm sets initial values for p={Kp, Ki, Kd} to 0 or very small values. Next the algorithm sequentially goes through each coefficent in P and increases it a little, resample error and if the error is smaller keep it, otherwise P is decreased and the error sampled again. This process is repeated over and over until the best coefficients are found. 

Before attempting Auto-tuning, I wanted find a systematic way to manually tune rather than selecting terms randomly. The following reference was a helpful in this process
[link]( https://www.crossco.com/blog/basics-tuning-pid-loops )

### Manual tuning steps:
  1. Start with a low proportional and no integral or derivative.
  2. Double the proportional until it begins to oscillate, then halve it.
  3. Implement a small integral.
  4. Double the integral until it starts oscillating, then halve it.
  That will get the constants close to where they need to be for fine adjustment.

#### Steps 1,2:
I started off with Kp=0.5, Ki=0, Kd=0, and the CTE was converging towards zero but oscilating as expected. I then tried Kp=0.2 and then Kp=0.1 

#### Steps 3,4:
I then added Ki = 0.1 and noticed the car started oscillate more sharply. I then kept halving it and noticed the car behaved best with Ki between 0.0001 and 0.0005. I settled on Ki=0.0003 as I noticed the oscillations were minimized and smoother.  The reason for this is that Ki term is a running sum of previous errors and when CTE from Kp term is small the Ki term helps prevent sharp overshoot. Over time the Ki running sum can get too large (Integral windup) and can cause instability. I have capped my Ki to 40 and reset it to 0. 

#### Kd Term
I then add the Kd term, The Kd counteracts the Kp term help smooth out the contribution from the Kp term and predict the next error. The Kd term is computed by finding the differnce betwween current error and the previous error. I started off with 0.1 and found no change, I kept increasing Kd to 0.5 and noticed the car was most stable. Increasing Kd >0.5 caused the large oscillations again.

I also wrote a function called AutoTuneController(). This essentially takes the twiddle algorithm and implements state machine. The reason for this is that after every increase in P vector we want to resample the error again. A state machine allows the controller to remember the current state and vector index and go to the correct next state when a new error is sampled. In the main loop the overall error is monitored and when it exceeds the tolerance level after some sample interval the auto tune takes over and finds Kp,Ki,Kd again. However I found that the manual tuning method worked sufficiently for this task. The final coefficents are Ki=0.1, Ki=0.0003, Kd=0.5, and these are set as defaults if no arguments are passed in on the command line.

 
