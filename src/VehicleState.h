//
//  VehicleState.h
//  Path_Planning
//
//  Created by Klemens on 06.08.17.
//
//

#ifndef VehicleState_h
#define VehicleState_h

namespace VehicleState {
  
  const double lane_width = 4;
  
  struct state {
    double s;
    double s_d;
    double s_dd;
    double d;
    double d_d;
    double d_dd;
  };

  inline int getLane(VehicleState::state currentState) {
    
    int lane = (currentState.d / lane_width);
        
    return lane;
  }

}

#endif /* VehicleState_h */
