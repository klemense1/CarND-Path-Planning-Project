// Copyright [2017] Klemens Esterle

#ifndef SRC_BEHAVIORPLANNER_H_
#define SRC_BEHAVIORPLANNER_H_

#include "VehicleState.h"
#include "World.h"

namespace BehaviorPlanner {
  const double max_s = 6945.554;
  const double max_acc = 10.;
  const double velocity_max = 21;
  const double dt = 0.02;
  const int n_steps = 80;
  const double dist_safety = 2*5;
  const double horizont = dt*n_steps;
  
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
  
  int getVehicleIDInFront(VehicleState::state currentState, World world) {
    double front_s_diff = 10000.0;
    int front_id = -10000;
    auto VehicleMap = world.getVehicleMap();
    
    for (auto const &tp : VehicleMap) {
      int lane_tp = VehicleState::getLane(tp.second.state);
      int lane_ego = VehicleState::getLane(currentState);
      if (lane_tp == lane_ego) {
        double s_diff = bound_s_difference(tp.second.state.s, currentState.s);
        if (s_diff < front_s_diff && s_diff > 0) {
          // only consider vehicles in front
          front_s_diff = s_diff;
          front_id = tp.first;
          // std::cout<<"found tp with front_s_diff"<<front_s_diff<<"and front_id"<<front_id<<std::endl;
        }
      }
    }
    return front_id;
  }
  
  VehicleState::state createGoal(VehicleState::state currentState, World world) {
    VehicleState::state goal;
    
    double final_speed = std::min(BehaviorPlanner::velocity_max, currentState.s_d + BehaviorPlanner::max_acc * BehaviorPlanner::horizont);
    double travelled_distance = std::min(final_speed*horizont, currentState.s_d*BehaviorPlanner::horizont + 0.5*max_acc*std::pow(BehaviorPlanner::horizont,2));
    
    int front_id = getVehicleIDInFront(currentState, world);
    
    if (front_id > 0) {
      auto vmap = world.getVehicleMap();
      
      Vehicle front_vehic = vmap[front_id];
      
      VehicleState::state front_vehic_state_predicted = front_vehic.getVehicleStateIn(horizont);
      
      double s_diff_predicted = bound_s_difference(front_vehic_state_predicted.s - (BehaviorPlanner::dist_safety), currentState.s);
      
      if (s_diff_predicted > 0 && s_diff_predicted < travelled_distance) {
        final_speed = front_vehic_state_predicted.s_d;
        travelled_distance = s_diff_predicted;
      }
    }
    
    goal.s = bound_s(currentState.s + travelled_distance);
    goal.s_d = final_speed;
    goal.s_dd = 0;
    goal.d = 6;
    goal.d_d = 0;
    goal.d_dd = 0;
    return goal;
  }
  
  
  
  VehicleState::state createGoalFollowFrontVehicle(VehicleState::state currentState, World world) {
    VehicleState::state goal;
    
    goal.s = bound_s(currentState.s + BehaviorPlanner::velocity_max*BehaviorPlanner::dt*BehaviorPlanner::n_steps);
    goal.s_d = BehaviorPlanner::velocity_max;
    goal.s_dd = 0;
    goal.d = 6;
    goal.d_d = 0;
    goal.d_dd = 0;
    
    return goal;
  }
  
  std::map<mode, double> decideMode(const VehicleState::state &currentState, const World &world) {
    
  }
  
  double costForChangeLeft(const VehicleState::state &currentState, const World &world) {
    
  }
}  // namespace BehaviorPlanner

#endif  // SRC_BEHAVIORPLANNER_H_
