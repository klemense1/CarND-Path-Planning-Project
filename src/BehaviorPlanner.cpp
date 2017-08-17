//
//  BehaviorPlanner.cpp
//  Path_Planning
//
//  Created by Klemens on 17.08.17.
//
//

#include <stdio.h>

#include "BehaviorPlanner.h"
#include "TrajectoryPlanner.h"

BehaviorPlanner::BehaviorPlanner() {
  _last_lane = 2;
}

BehaviorPlanner::~BehaviorPlanner() {}

int BehaviorPlanner::getVehicleIDInFront(VehicleState::state currentState, World world, int lane) {
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

VehicleState::state BehaviorPlanner::createGoalInLane(VehicleState::state currentState, World world, int desired_lane) {
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
  goal.d = (desired_lane-1)*BehaviorPlanner::lane_width + BehaviorPlanner::lane_width/2;
  goal.d_d = 0;
  goal.d_dd = 0;
  
  
  return goal;
}

TrajectoryPlanner::Path2d BehaviorPlanner::createBehavior(VehicleState::state currentState, World world) {

  VehicleState::state bestGoal = createGoalInLane(currentState, world, _last_lane);
  TrajectoryPlanner::Path2d bestPath = this->trajplanner.createTrajectoryXY(currentState, bestGoal, world);
  double bestCosts = this->trajplanner.costFunction(bestPath);
  
  double lane_d = fabs(currentState.d - (getLane(currentState)-1) * BehaviorPlanner::lane_width - BehaviorPlanner::lane_width / 2);

  if ((getLane(currentState) != _last_lane) || (lane_d > BehaviorPlanner::lane_width / 4)) {
    std::cout << "BehaviorPlanner::createBehavior: Finishing maneuver towards lane " << _last_lane << " with deviation lane_d = " << lane_d << std::endl;
    return bestPath;
  }
  
  std::vector<int> lane_change = {0, -1, 1};
  int current_lane = VehicleState::getLane(currentState);

  for (auto i=0; i<lane_change.size(); ++i) {
    //int new_lane = current_lane + lane_change[i];
    int new_lane = 2;
    if (new_lane>0 && new_lane <= BehaviorPlanner::lane_max) {
      VehicleState::state goal = createGoalInLane(currentState, world, new_lane);
      TrajectoryPlanner::Path2d path = this->trajplanner.createTrajectoryXY(currentState, goal, world);
      double costs = this->trajplanner.costFunction(path);
      if(costs < bestCosts) {
        std::cout << "BehaviorPlanner::createBehavior: New maneuver towards lane " << new_lane << std::endl;
        bestCosts = costs;
        bestGoal = goal;
        bestPath = path;
      }
    }
  }
  
  _last_lane = getLane(bestGoal);
  
  return bestPath;
}
