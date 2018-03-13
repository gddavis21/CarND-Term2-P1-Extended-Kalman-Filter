# Extended Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

Utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower than the tolerance outlined in the project rubric. 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

---

## Implementation Overview

* Top-level __main()__ function (main.cpp):
  - Parse time-series of Lidar/Radar measurements and ground-truth position/velocity values.
  - Send measurements to FusionEKF instance, query FusionEKF for estimated position/velocity state.
  - For each measurement, report Root-Mean-Square-Error of estimated state, relative to ground-truth.
  - NOTE: main.cpp was (almost) entirely supplied from starter code.

* Class __FusionEKF__ (FusionEKF.h / FusionEKF.cpp):
  - Method ProcessMeasurement() receives Lidar/Radar measurements and updates estimated position/velocity state.
    + Initializes state estimate on first measurement.
    + For subsequent Lidar measurements, updates state estimate using standard Kalman Filter algorithm.
    + For subsequent Radar measurements, updates state estimate using Extended Kalman Filter algorithm.

  - Query methods GetCurrentPosition() and GetCurrentVelocity() to get current position/velocity state estimate.

* Function __GaussianFilters::KalmanFilter__ (kalman_filter.h / kalman_filter.cpp) implements the Kalman Filter state estimation algorithm, as described in lecture and in Probabilistic Robotics section 3.2.

* Function __GaussianFilters::ExtendedKalmanFilter_LinearPred__ (kalman_filter.h / kalman_filter.cpp) implements a version of the Extended Kalman Filter for state estimation, as described in lecture and in Probabilistic Robotics section 3.3. 

* Function __Tools::ComputeRMSE()__ (tools.h / tools.cpp) implements the Root-Mean-Square-Error metric for measuring accuracy of the state estimation. 

---

## Important Dependencies

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

