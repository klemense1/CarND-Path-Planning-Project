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
  const double max_acc = 5.;
  const double velocity_max = 21;
  const double dt = 0.02;
  const int n_steps = 120;
  const double dist_safety = 4*5;
  const double horizont = dt*n_steps;
  const int lane_max = 3;
  const int lane_width = 4;
  const size_t n_delay = 5;
  
  
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
  
  bool VehiclesBlockingLane(const VehicleState::state &currentState, const World &world, int lane);
  int getVehicleIDInFront(const VehicleState::state &currentState, const World &world, int lane);
  VehicleState::state createGoalInLane(const VehicleState::state &currentState, const World &world, int desired_lane);
  double costFuntion(const TrajectoryPlanner::Path2d &PathFrenet, const VehicleState::state &currentState, const VehicleState::state &goal, const World &world, const int lanes_changed);
  
public:
  int _last_lane;
  
  BehaviorPlanner();
  
  virtual ~BehaviorPlanner();
  
  TrajectoryPlanner trajplanner;
  TrajectoryPlanner::Path2d createBehavior(const VehicleState::state &currentState, const World &world);
  TrajectoryPlanner::Path2d createTrajectory(const VehicleState::state &currentState, const VehicleState::state &goal, const World &world);
  TrajectoryPlanner::Path2d createFirstTrajectorySegment(const VehicleState::state &currentState, const World &world);
};

#endif  // SRC_BEHAVIORPLANNER_H_
