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
  
  struct state {
    double s;
    double s_d;
    double s_dd;
    double d;
    double d_d;
    double d_dd;
  };
  
}

#endif /* VehicleState_h */
