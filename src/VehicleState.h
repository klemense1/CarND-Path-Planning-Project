// Copyright [2017] Klemens Esterle

#ifndef SRC_VEHICLESTATE_H_
#define SRC_VEHICLESTATE_H_

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
    int lane = (currentState.d / lane_width) + 1;
    return lane;
  }
  
}  // namespace VehicleState

#endif  // SRC_VEHICLESTATE_H_
