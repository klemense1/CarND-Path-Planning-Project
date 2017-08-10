// Copyright [2017] Klemens Esterle

#ifndef SRC_BEHAVIORPLANNER_H_
#define SRC_BEHAVIORPLANNER_H_

#include <tgmath.h>
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
  const double costs_idle = 3;
  const double lane_width = 4;
  
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
  
  int getVehicleIDInFront(VehicleState::state currentState, World world, int lane) {
    double front_s_diff = 10000.0;
    int front_id = -10000;
    auto VehicleMap = world.getVehicleMap();
    
    for (auto const &tp : VehicleMap) {
      int lane_tp = VehicleState::getLane(tp.second.state);
      if (lane_tp == lane) {
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
  
  double calculate_costs(const VehicleState::state &ego_state, const VehicleState::state &vehic_state_other) {
    
    double costs_speed_ego = std::max(0.0, BehaviorPlanner::velocity_max - ego_state.s_d);
    double costs_other = std::max(0.0, BehaviorPlanner::velocity_max - vehic_state_other.s_d);
    double costs = costs_speed_ego+costs_other;
    return costs;
  }
  
  double costForChangeLeft(const VehicleState::state &currentState, const World &world) {
    int current_lane = VehicleState::getLane(currentState);
    
    double costs;
    int vehic_id_left = getVehicleIDInFront(currentState, world, current_lane - 1);
    
    if (current_lane == 1) {
      // vehicle is already in left lane, cannot change left
      costs = 1e6;
    } else if (vehic_id_left > 0) {
      
      auto vmap = world.getVehicleMap();
      
      Vehicle vehic_left = vmap[vehic_id_left];
      
      if (BehaviorPlanner::bound_s_difference(vehic_left.state.s, currentState.s) < 60) {
        VehicleState::state vehic_state_left_predicted = vehic_left.getVehicleStateIn(horizont);
        
        costs = calculate_costs(currentState, vehic_state_left_predicted) * 1.5;
      } else {
        costs = BehaviorPlanner::costs_idle;
      }
    } else {
      costs = BehaviorPlanner::costs_idle;
    }
    return costs;
  }
  
  double costForChangeRight(const VehicleState::state &currentState, const World &world) {
    int current_lane = VehicleState::getLane(currentState);
    
    double costs;
    int vehic_id_right = getVehicleIDInFront(currentState, world, current_lane - 1);
    
    if (current_lane == 3) {
      // vehicle is already in right lane, cannot change left
      costs = 1e6;
    } else if (vehic_id_right > 0) {
      
      auto vmap = world.getVehicleMap();
      
      Vehicle vehic_right = vmap[vehic_id_right];
      
      if (BehaviorPlanner::bound_s_difference(vehic_right.state.s, currentState.s) < 60) {
        VehicleState::state vehic_state_right_predicted = vehic_right.getVehicleStateIn(horizont);
        
        costs = calculate_costs(currentState, vehic_state_right_predicted) * 1.5;
      } else {
        costs = BehaviorPlanner::costs_idle;
      }
    } else {
      costs = BehaviorPlanner::costs_idle;
    }
    return costs;
  }
  
  
  double costForStayInLane(const VehicleState::state &currentState, const World &world) {
    int current_lane = VehicleState::getLane(currentState);
    
    int vehic_id_front = getVehicleIDInFront(currentState, world, current_lane);
    
    auto vmap = world.getVehicleMap();
    
    Vehicle vehicle_front = vmap[vehic_id_front];
    
    double costs;
    
    if (vehic_id_front > 0) {
      VehicleState::state vehic_state_front_predicted = vehicle_front.getVehicleStateIn(horizont);
      costs = calculate_costs(currentState, vehic_state_front_predicted);
    } else {
      costs = 0;
    }
    return costs;
  }
  
  int decideMode(const VehicleState::state &currentState, const World &world) {
    double costs_left = costForChangeLeft(currentState, world);
    double costs_stay = costForStayInLane(currentState, world);
    double costs_right = costForChangeRight(currentState, world);
    std::cout << "Costs: stay=" << costs_stay << " | " << "go left=" << costs_left << " | " << "go right=" << costs_right << std::endl;
    
    int desired_lane;
    if (costs_left < costs_stay) {
      desired_lane = VehicleState::getLane(currentState) - 1;
    } else if (costs_right < costs_stay){
      desired_lane = VehicleState::getLane(currentState) + 1;
    } else {
      desired_lane = VehicleState::getLane(currentState);
    }
    return desired_lane;
  }
  
  VehicleState::state createGoal(VehicleState::state currentState, World world) {
    VehicleState::state goal;
    
    double final_speed = std::min(BehaviorPlanner::velocity_max, currentState.s_d + BehaviorPlanner::max_acc * BehaviorPlanner::horizont);
    double travelled_distance = std::min(final_speed*horizont, currentState.s_d*BehaviorPlanner::horizont + 0.5*max_acc*std::pow(BehaviorPlanner::horizont,2));
    
    int current_lane = VehicleState::getLane(currentState);
    int vehic_id_front = getVehicleIDInFront(currentState, world, current_lane);
    
    if (vehic_id_front > 0) {
      auto vmap = world.getVehicleMap();
      
      Vehicle front_vehic = vmap[vehic_id_front];
      
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
    goal.d = 0*lane_width + lane_width/2;
    goal.d_d = 0;
    goal.d_dd = 0;
    
    decideMode(currentState, world);
    
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
  
}  // namespace BehaviorPlanner

#endif  // SRC_BEHAVIORPLANNER_H_
