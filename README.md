# PID Controller Project
---

## Introduction

The purpose of this project is to build a lane keeping feature for a simulated car using PID controller. The cross track error is minimised by using PID controller. 

## PID Components

P stands for the proportional term which is acts on the direct quantity to be minimised. In this project I tried to minimise the cross track error which is the distance from the center of the lane to the car. The p component is multiplied with the CTE which means we will steer proportional to the CTE based on the P component.

D stands for the differential term which acts on the difference of the consecutive values of the quantity to be minimised. In the project the D component tried to minimise the consecutive cross track errors. This helps to smoothly reach the center line because the P component is direct and might cause the car to oscillate.

I stands to the integral term which acts on the accumulated sum of the quantity to be minimised. In this project the I component tried to minimise the sum of the cross track error. I component works on the steering drift or the systematic bias of the car. This helps to reach the center line correctly.

### Tuning PID Components


## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

