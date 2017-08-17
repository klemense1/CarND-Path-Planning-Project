// Copyright [2017] Klemens Esterle

#ifndef SRC_BEHAVIORPLANNER_H_
#define SRC_BEHAVIORPLANNER_H_

#include <tgmath.h>
#include "VehicleState.h"
#include "World.h"
#include "TrajectoryPlanner.h"

class BehaviorPlanner {
  
private:
  const double max_s = 6945.554;
  const double max_acc = 10.;
  const double velocity_max = 21;
  const double dt = 0.02;
  const int n_steps = 100;
  const double dist_safety = 2*5;
  const double horizont = dt*n_steps;
  const double costs_idle = 3;
  const int lane_max = 3;
  const int lane_width = 4;
  
  enum mode {keepLane, switchLeft, switchRight};
  
  mode currentMode;
  
  inline double bound_s(double s) {
    double bounded_s;
    if (s < 0 || s > max_s) {
      bounded_s = fmod(BehaviorPlanner::max_s + s, BehaviorPlanner::max_s);
    } else {
      bounded_s = s;
    }
    return bounded_s;
  }
  
  inline double bound_s_difference(double s1, double s2) {
    auto a = bound_s(s1);
    auto b = bound_s(s2);
    auto diff = a - b;
    if (diff < -max_s / 2) diff += max_s;
    if (diff > max_s / 2) diff -= max_s;
    return diff;
  }
  
public:
  int _last_lane;
  BehaviorPlanner();
  
  virtual ~BehaviorPlanner();
  
  TrajectoryPlanner trajplanner;
  
  int getVehicleIDInFront(VehicleState::state currentState, World world, int lane);
  VehicleState::state createGoalInLane(VehicleState::state currentState, World world, int desired_lane);
  TrajectoryPlanner::Path2d createBehavior(VehicleState::state currentState, World world);
  

};

#endif  // SRC_BEHAVIORPLANNER_H_
