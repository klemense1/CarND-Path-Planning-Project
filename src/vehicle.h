// Copyright [2017] Klemens Esterle

#ifndef SRC_VEHICLE_H_
#define SRC_VEHICLE_H_

#include <vector>
#include <sstream>
#include <utility>

#include "VehicleState.h"

class Vehicle {
public:
  VehicleState::state state;
  
  int id;
  
  Vehicle();
  Vehicle(const VehicleState::state &initialState);
  
  virtual ~Vehicle();
  
  void move(const std::vector<double>& path_s, const std::vector<double>& path_d, size_t steps);
  
  void setVelocity(double s_dot, double d_dot);
  
  void setPosition(double s, double d);
  
  friend std::ostream& operator<<(std::ostream& stream, const Vehicle& matrix);

  VehicleState::state getVehicleState();
  
  VehicleState::state getVehicleStateIn(double dt);
};

#endif  // SRC_VEHICLE_H_
