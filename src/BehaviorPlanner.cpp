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


bool BehaviorPlanner::VehiclesBlockingLane(VehicleState::state currentState, World world, int lane) {
  
  bool blocked = false;
  
  double front_s_diff = 10000.0;
  int front_id = -10000;
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


VehicleState::state BehaviorPlanner::createGoalInLane(VehicleState::state currentState, World world, int desired_lane) {
  VehicleState::state goal;
  
  double final_speed = std::min(BehaviorPlanner::velocity_max, currentState.s_d + BehaviorPlanner::max_acc * BehaviorPlanner::horizont);
  double acc = (final_speed - currentState.s_d) / BehaviorPlanner::horizont;
  double travelled_distance = std::min(final_speed * horizont, currentState.s_d * BehaviorPlanner::horizont + 0.5 * acc * std::pow(BehaviorPlanner::horizont,2));
  
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
  
  // Calculate best target position along d
  double target_d = (getLane(currentState)-1) * BehaviorPlanner::lane_width + BehaviorPlanner::lane_width / 2;
  
  if(desired_lane != getLane(currentState)) {
    target_d = (desired_lane-1)*BehaviorPlanner::lane_width + BehaviorPlanner::lane_width/2;
  }
  
  goal.s = bound_s(currentState.s + travelled_distance);
  goal.s_d = final_speed;
  goal.s_dd = 0;
  goal.d = target_d;
  goal.d_d = 0;
  goal.d_dd = 0.8 * max_acc * (currentState.d - target_d) / BehaviorPlanner::lane_width;
  
  return goal;
}

TrajectoryPlanner::Path2d BehaviorPlanner::createBehavior(VehicleState::state currentState, World world) {

  VehicleState::state bestGoal = createGoalInLane(currentState, world, _last_lane);
  TrajectoryPlanner::Path2d bestPath = this->trajplanner.createTrajectoryFrenet(currentState, bestGoal);
  //double bestCosts = costFunction(bestPath, 0);
  double bestCosts = costFuntion(bestPath, currentState, bestGoal, world, 0);

  
  double lane_d = fabs(currentState.d - (getLane(currentState)-1) * BehaviorPlanner::lane_width - BehaviorPlanner::lane_width / 2);

  if ((getLane(currentState) != _last_lane) || (lane_d > BehaviorPlanner::lane_width / 4)) {
    std::cout << "BehaviorPlanner::createBehavior: Finishing maneuver towards lane " << _last_lane << " with deviation lane_d = " << lane_d << std::endl;
    TrajectoryPlanner::Path2d bestPathXY = this->trajplanner.createTrajectoryXY(bestPath, world);
    return bestPathXY;
  }
  
  std::vector<int> lane_change = {0, -1, 1};
  int current_lane = VehicleState::getLane(currentState);

  for (auto i=0; i<lane_change.size(); ++i) {
    int new_lane = current_lane + lane_change[i];
    //int new_lane = 2;
    if (new_lane>0 && new_lane <= BehaviorPlanner::lane_max) {
      VehicleState::state goal = createGoalInLane(currentState, world, new_lane);
      TrajectoryPlanner::Path2d path = this->trajplanner.createTrajectoryFrenet(currentState, goal);
      //double costs = costFunction(path, lane_change[i]);
      double costs = costFuntion(path, currentState, goal, world, lane_change[i]);
      std::cout << "costs for lane " << new_lane << ": " << costs << " with s_d " << goal.s_d << std::endl;
      

      if(costs < bestCosts) {
        bestCosts = costs;
        bestGoal = goal;
        bestPath = path;
        std::cout << "BehaviorPlanner::createBehavior: New maneuver towards lane " << new_lane << std::endl;
        std::cout << "\t goal: s, s_d, s_dd, d, d_dd, d_dd" << goal.s << " " << goal.s_d << " " << goal.s_dd << " " << goal.d << " " << goal.d_d << " " << goal.d_dd << " " << std::endl;

      }
    }
  }
  
  _last_lane = getLane(bestGoal);
  
  TrajectoryPlanner::Path2d bestPathXY = this->trajplanner.createTrajectoryXY(bestPath, world);
  return bestPathXY;
}


double BehaviorPlanner::costsJerk(TrajectoryPlanner::Path2d Path) {
  std::vector<double> s_jerk = BehaviorPlanner::getThirdDerivative(Path[0]);
  std::vector<double> d_jerk = BehaviorPlanner::getThirdDerivative(Path[1]);
  
  double costs = 0;
  
  for (int i = 0; i < s_jerk.size(); i++) {
    double jerk = sqrt(s_jerk[i] * s_jerk[i] + d_jerk[i] * d_jerk[i]);
    costs += jerk;
  }
  return costs;
}

double BehaviorPlanner::costsAcceleration(TrajectoryPlanner::Path2d Path) {
  std::vector<double> s_dd = BehaviorPlanner::getSecondDerivative(Path[0]);
  std::vector<double> d_dd = BehaviorPlanner::getSecondDerivative(Path[1]);
  
  double costs = 0;
  
  for (int i = 0; i < s_dd.size(); i++) {
    double acc = sqrt(s_dd[i] * s_dd[i] + d_dd[i] * d_dd[i]);
    costs += acc;
  }
  return costs;
}

double BehaviorPlanner::costsVelocity(TrajectoryPlanner::Path2d Path) {
  std::vector<double> s_d = BehaviorPlanner::getFirstDerivative(Path[0]);
  std::vector<double> d_d = BehaviorPlanner::getFirstDerivative(Path[1]);
  
  double costs = 0;
  
  for (int i = 0; i < s_d.size(); i++) {
    double vel = sqrt(s_d[i] * s_d[i] + d_d[i] * d_d[i]);
    std::cout << "vel " << vel << std::endl;
    costs += std::abs(vel-BehaviorPlanner::velocity_max);
  }
  return costs;
}

double BehaviorPlanner::costFunction(TrajectoryPlanner::Path2d Path, int lanes_changed) {
  
  double costs_velocity = costsVelocity(Path);
  double costs_acc = costsAcceleration(Path)/10;
  double costs_jerk = costsJerk(Path);
  double costs_lanechange = 10 * abs(lanes_changed);
  double total_costs = costs_velocity + costs_acc + costs_jerk + costs_lanechange;

  std::cout << "BehaviorPlanner::costFunction: total " << total_costs << " vel " << costs_velocity << " acc " << costs_acc << " jerk " << costs_jerk << " lanechange " << costs_lanechange << std::endl;
  
  return total_costs;
}

double BehaviorPlanner::costFuntion(TrajectoryPlanner::Path2d Path, VehicleState::state currentState, VehicleState::state goal, World world, int lanes_changed) {
  double costs = std::abs(velocity_max - goal.s_d);
  costs += abs(lanes_changed);
  if (VehiclesBlockingLane(currentState, world, VehicleState::getLane(goal)))
    costs += 1000;
  return costs;
}
