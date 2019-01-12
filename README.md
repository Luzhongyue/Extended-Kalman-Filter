# Vehicle tracking with Extended Kalman Filter 

## Extended Kalman Filter Project Starter Code

This project utilizes a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. 

This project involves a Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. 

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.

INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)


OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

---

## Other Important Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `

## Code Style

[Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## File structure
* scr:a directory with the project code:  \
       1. main.cpp - communicates with the 2 Simulator receiving data measurements, calls a function to run the Kalman filter, calls a                        function to calculate RMSE\
       2. FusionEKF.cpp - initializes the filter, calls the predict function, calls the update function\
       3. kalman_filter.cpp- defines the predict function, the update function for lidar, and the update function for radar\
       4. tools.cpp- function to calculate RMSE and the Jacobian matrix
* data:a directory with two input files, provided by Udacity
* Docs:contains details about the structure of the code templates
* CMakeLists.txt: a file will be used when compiling the codes 

## Results

Lidar measurements are red circles, radar measurements are blue circles with an arrow pointing in the direction of the observed angle, and estimation markers are green triangles. 

### Dataset1
![](https://github.com/Luzhongyue/Extended-Kalman-Filter/blob/master/image/Dataset1.png)
### Dataset2
![](https://github.com/Luzhongyue/Extended-Kalman-Filter/blob/master/image/Dataset2.png)

## Overview of a Kalman Filter: Initialize, Predict, Update

* initializing Kalman filter variables
* predicting where our object is going to be after a time step \Delta{t}Δt
* updating where our object is based on sensor measurement

Then the prediction and update steps repeat themselves in a loop.
To measure how well the Kalman filter performs, we will then calculate root mean squared error comparing the Kalman filter results with the provided ground truth.
These three steps (initialize, predict, update) plus calculating RMSE encapsulate the entire extended Kalman filter project.

### LIDAR

![](https://github.com/Luzhongyue/Extended-Kalman-Filter/blob/master/image/Lidar.png)

### RADAR

![](https://github.com/Luzhongyue/Extended-Kalman-Filter/blob/master/image/Rader.png)

### How does the EKF work

![](https://github.com/Luzhongyue/Extended-Kalman-Filter/blob/master/image/EKF_work.png)

* first measurement - the filter will receive initial measurements of the bicycle's position relative to the car. These measurements                           will come from a radar or lidar sensor.
* initialize state and covariance matrices - the filter will initialize the bicycle's position based on the first measurement.
* then the car will receive another sensor measurement after a time period \Delta{t}Δt.
* predict - the algorithm will predict where the bicycle will be after time \Delta{t}Δt. One basic way to predict the bicycle location               after \Delta{t}Δt is to assume the bicycle's velocity is constant; thus the bicycle will have moved velocity * \Delta{t}Δt.             In the extended Kalman filter lesson, we will assume the velocity is constant.
* update - the filter compares the "predicted" location with what the sensor measurement says. The predicted location and the measured              location are combined to give an updated location. The Kalman filter will put more weight on either the predicted location or            the measured location depending on the uncertainty of each value.
* then the car will receive another sensor measurement after a time period \Delta{t}Δt. The algorithm then does another predict and       update step.

### EKF vs KF 

![](https://github.com/Luzhongyue/Extended-Kalman-Filter/blob/master/image/EKFvsKF.png)

In order to help you following along with the derivations of the Kalman Filter equations, you can visit the following links. (https://s3.amazonaws.com/video.udacity-data.com/topher/2018/June/5b327c11_sensor-fusion-ekf-reference/sensor-fusion-ekf-reference.pdf)
