# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

### The Term3 Simulator can be downloaded from [releases tab](https://github.com/udacity/self-driving-car-sim/releases).

In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.


## Architecture

The Library consists of the following files.

- BehaviorPlanner: Decision maker (stay in lane, change lane) to achieve a smooth, satisfying and collisionfree trajectory
- TrajectoryPlanner: Generating Trajectories based on quintic lateral and longitudinal polynomials 
- World: contains track data (waypoints) as well as sensor data
- Vehicle: containing state and methods to manipulate it
- VehicleState: Header file defining vehicle state
- Others
  - Parameters
  - Utilities
  - Spline

## Continuity of Trajectory

The conversion function from global to frenet is not reliable enough to allow to use the current position at each cycle and plan from there. It leads to deviations resulting in jumpy behavior. That said, an approach was chosen here that reuses the remaining trajectory from the last cycle, cuts out what has been passsed and reuses it. This allows for a smooth behavior.

## Behavior-Planning

Each cycle, the behavior planning consists of the following steps:

1. Creating possible Goals
2. Trajectory Generation in Frenet System
3. Decision Making
4. Conversion from Frenet Reference System to Global Reference System

After that, the new trajectory is passed to the simulator in the main-function.

### Creating possible Goals

The behavior planner considers the following high-level decisions: 

- Switching one lane to the left

- Switching one lane to the right

- staying in the lane.

Thus, for each of those new lanes (if they are within the street), a possible goal is calculated. world.getVehicleMap() provides a map with all vehicles, which is used to check wether a possible goal would collide with another car. If so, a goal is chosen to adapt to the front car and engage in a following mode.

### Trajectory Generation in Frenet System

For each of those goals, a trajectory is created using createTrajectoryFrenet(). This function uses JMT() to create two jerk minimizing trajectories based on quintic polynoms for both lateral and longitudinal movement.

### Decision Making

Based on these possible trajectories in the s-d-system, the costs are calculated in costFuntion(). It penalizes
- deviation from the max speed, 
- lane changes
- if a vehicle is blocking the desired lane in order to prevent a crash
  - VehiclesBlockingLane() checks wether a vehicle is blocking, 

This could be easily extended to also penalize higher accerlation or jerk.

### Conversion from Frenet Reference System to Global Reference System

When a desired trajectory has been chosen, this trajectory is converted from frenet to the global system, as the simulator requires it. The function createTrajectoryXY in TrajectoryPlanner uses spline.h, an open-source library for spline interpolation.



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
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
