//
//  BehaviorPlanner.h
//  Path_Planning
//
//  Created by Klemens on 07.08.17.
//
//

#ifndef BehaviorPlanner_h
#define BehaviorPlanner_h
#include "VehicleState.h"
#include "World.h"

namespace BehaviorPlanner {
  enum mode {keepLane, switchLeft, switchRight};
  mode currentMode;
  VehicleState::state createGoal(VehicleState::state currentState, World world) {
    VehicleState::state goal;
    
    goal.s = currentState.s + 22;//Parameters::velocity_max*Parameters::dt*Parameters::n_steps;
    goal.s_d = 22;//Parameters::velocity_max;
    goal.s_dd = 0;
    goal.d = 6;
    goal.d_d = 0;
    goal.d_dd = 0;
    
    return goal;
  };
};

#endif /* BehaviorPlanner_h */
