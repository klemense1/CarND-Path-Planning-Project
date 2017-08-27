// Copyright [2017] Klemens Esterle

#include "BehaviorPlanner.h"

#include <stdio.h>
#include <algorithm>
#include <vector>

#include "TrajectoryPlanner.h"

BehaviorPlanner::BehaviorPlanner() {
  _last_lane = 2;
}


BehaviorPlanner::~BehaviorPlanner() {}


int BehaviorPlanner::getVehicleIDInFront(const VehicleState::state &currentState, const World &world, int lane) {
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


bool BehaviorPlanner::VehiclesBlockingLane(const VehicleState::state &currentState, const World &world, int lane) {
  bool blocked = false;
  
  auto VehicleMap = world.getVehicleMap();
  
  for (auto const &tp : VehicleMap) {
    int lane_tp = VehicleState::getLane(tp.second.state);
    if (lane_tp == lane) {
      double s_diff = bound_s_difference(tp.second.state.s, currentState.s);
      if (std::abs(s_diff) < dist_safety) {
        blocked = true;
        // std::cout<<"found tp with front_s_diff"<<front_s_diff<<"and front_id"<<front_id<<std::endl;
      }
    }
  }
  return blocked;
}


VehicleState::state BehaviorPlanner::createGoalInLane(const VehicleState::state &currentState, const World &world, int desired_lane) {
  VehicleState::state goal;
  
  double final_speed = std::min(BehaviorPlanner::velocity_max, currentState.s_d + BehaviorPlanner::max_acc * BehaviorPlanner::horizont);
  double acc = (final_speed - currentState.s_d) / BehaviorPlanner::horizont;
  double travelled_distance = std::min(final_speed * horizont, currentState.s_d * BehaviorPlanner::horizont + 0.5 * acc * std::pow(BehaviorPlanner::horizont, 2));
  
  int vehic_id_front = getVehicleIDInFront(currentState, world, desired_lane);
  
  if (vehic_id_front > 0) {
    auto vmap = world.getVehicleMap();
    
    Vehicle front_vehic = vmap[vehic_id_front];
    
    VehicleState::state front_vehic_state_predicted = front_vehic.getVehicleStateIn(horizont);
    
    double s_diff_predicted = bound_s_difference(front_vehic_state_predicted.s - BehaviorPlanner::dist_safety, currentState.s);
    
    if (s_diff_predicted > 0 && s_diff_predicted < travelled_distance) {
      final_speed = front_vehic_state_predicted.s_d;
      travelled_distance = s_diff_predicted;
    }
  }
  
  double target_d;
  if (desired_lane != getLane(currentState)) {
    target_d = (desired_lane-1) * BehaviorPlanner::lane_width + BehaviorPlanner::lane_width/2;
  } else {
    target_d = (getLane(currentState)-1) * BehaviorPlanner::lane_width + BehaviorPlanner::lane_width / 2;
  }
  
  goal.s = bound_s(currentState.s + travelled_distance);
  goal.s_d = final_speed;
  goal.s_dd = 0;
  goal.d = target_d;
  goal.d_d = 0;
  goal.d_dd = 0.8 * max_acc * (currentState.d - target_d) / BehaviorPlanner::lane_width;
  
  return goal;
}


double BehaviorPlanner::costFuntion(const TrajectoryPlanner::Path2d &PathFrenet, const VehicleState::state &currentState, const VehicleState::state &goal, const World &world, const int lanes_changed) {
  double costs = std::abs(velocity_max - goal.s_d);
  costs += abs(lanes_changed);
  if (VehiclesBlockingLane(currentState, world, VehicleState::getLane(goal)))
    costs += 1000;
  return costs;
}


TrajectoryPlanner::Path2d BehaviorPlanner::createBehavior(const VehicleState::state &currentState, const World &world) {
  VehicleState::state newGoal = createGoalInLane(currentState, world, _last_lane);
  TrajectoryPlanner::Path2d newPathFrenet = this->trajplanner.createTrajectoryFrenet(currentState, newGoal, BehaviorPlanner::n_steps);
  double newCosts = costFuntion(newPathFrenet, currentState, newGoal, world, 0);
  
  double lane_d = fabs(currentState.d - (getLane(currentState)-1) * BehaviorPlanner::lane_width - BehaviorPlanner::lane_width / 2);
  
  if ((getLane(currentState) != _last_lane) || (lane_d > BehaviorPlanner::lane_width / 4)) {
    std::cout << "BehaviorPlanner::createBehavior: Finishing maneuver towards lane " << _last_lane << " with deviation lane_d = " << lane_d << std::endl;
    TrajectoryPlanner::Path2d newPathXY = this->trajplanner.createTrajectoryXY(newPathFrenet, world);
    return newPathXY;
  }
  
  std::vector<int> lane_change = {0, -1, 1};
  int current_lane = VehicleState::getLane(currentState);
  
  for (auto i=0; i<lane_change.size(); ++i) {
    int new_lane = current_lane + lane_change[i];
    if (new_lane>0 && new_lane <= BehaviorPlanner::lane_max) {
      VehicleState::state goal = createGoalInLane(currentState, world, new_lane);
      TrajectoryPlanner::Path2d pathFrenet = createTrajectory(currentState, goal, world);
      double costs = costFuntion(pathFrenet, currentState, goal, world, lane_change[i]);
      std::cout << "costs for lane " << new_lane << ": " << costs << " with s_d " << goal.s_d << std::endl;
      
      if (costs < newCosts) {
        newCosts = costs;
        newGoal = goal;
        newPathFrenet = pathFrenet;
        std::cout << "BehaviorPlanner::createBehavior: New maneuver towards lane " << new_lane << std::endl;
        std::cout << "\t goal: s, s_d, s_dd, d, d_dd, d_dd" << goal.s << " " << goal.s_d << " " << goal.s_dd << " " << goal.d << " " << goal.d_d << " " << goal.d_dd << " " << std::endl;
      }
    }
  }
  
  _last_lane = getLane(newGoal);
  
  TrajectoryPlanner::Path2d newPathXY = this->trajplanner.createTrajectoryXY(newPathFrenet, world);
  return newPathXY;
}

TrajectoryPlanner::Path2d BehaviorPlanner::createTrajectory(const VehicleState::state &currentState, const VehicleState::state &goal, const World &world) {
  
  TrajectoryPlanner::Path2d completeTrajectory = createFirstTrajectorySegment(currentState, world);
  
  size_t n_steps_for_maneuver = BehaviorPlanner::n_steps - completeTrajectory[0].size();
  
  VehicleState::state startState;
  
  if(!completeTrajectory[0].empty()) {
    Vehicle vehic(currentState);
    vehic.move(completeTrajectory[0], completeTrajectory[1], completeTrajectory[0].size());
    startState = vehic.state;
  }
  else {
    startState = currentState;
  }
  
  TrajectoryPlanner::Path2d rearTrajectorySegment = this->trajplanner.createTrajectoryFrenet(startState, goal, n_steps_for_maneuver);
  
  for (size_t i = 1; i <= n_steps_for_maneuver; i++) {
    Vehicle vehic(startState);
    vehic.move(rearTrajectorySegment[0], rearTrajectorySegment[1], i);
    completeTrajectory[0].emplace_back(vehic.state.s);
    completeTrajectory[1].emplace_back(vehic.state.d);
  }
  return completeTrajectory;
}

TrajectoryPlanner::Path2d BehaviorPlanner::createFirstTrajectorySegment(const VehicleState::state &currentState, const World &world) {
  
  TrajectoryPlanner::Path2d lastTrajectory = this->trajplanner.getLastSentTrajectory();
  size_t n_delay = std::min(this->trajplanner.getLastSentTrajectoryLength(), BehaviorPlanner::n_delay);
  
  TrajectoryPlanner::Path2d firstTrajectorySegment(2);
  
  for (size_t i = 1; i <= n_delay; i++) {
    Vehicle vehic(currentState);
    vehic.move(lastTrajectory[0], lastTrajectory[1], i);
    firstTrajectorySegment[0].emplace_back(vehic.state.s);
    firstTrajectorySegment[1].emplace_back(vehic.state.d);
  }
  
  return firstTrajectorySegment;
}
