//
//  vehicle.cpp
//  Path_Planning
//
//  Created by Klemens on 06.08.17.
//
//

#include "Vehicle.h"

Vehicle::Vehicle() {
  
  this->state.s = 0;
  this->state.s_d = 0;
  this->state.s_dd = 0;
  this->state.d = 0;
  this->state.d_d = 0;
  this->state.d_dd = 0;
}

Vehicle::~Vehicle() {}

void Vehicle::setPosition(double s, double d) {
  this->state.s = s;
  this->state.d = d;
}

void Vehicle::setVelocity(double s_d, double d_d) {
  this->state.s_d = s_d;
  this->state.d_d = d_d;
}


std::ostream &operator<<(std::ostream &stream, const Vehicle &vehicle) {
  streamsize init_precision = stream.precision();
  stream.precision(2);
  stream<<"Vehicle: ";
  stream<<"s("<<fixed<<vehicle.state.s<<", "<<fixed<<vehicle.state.s_d<<", "<<fixed<<vehicle.state.s_dd<<") | ";
  stream<<"d("<<fixed<<vehicle.state.d<<", "<<fixed<<vehicle.state.d_d<<", "<<fixed<<vehicle.state.d_dd<<") | ";
  stream<<"Lane "<<fixed<<VehicleState::getLane(vehicle.state)<<fixed<<" | ";;
  stream.precision(init_precision);
  return stream;
}

void Vehicle::move(const vector<double>& path_s, const vector<double>& path_d, size_t steps) {
  long idx = steps - 1;
  double dt = 0.02;
  if(idx > 0) {
    double s_d = (path_s[idx] - path_s[idx - 1]) / dt;
    double d_d = (path_d[idx] - path_d[idx - 1]) / dt;
    this->state.s_dd = (s_d - this->state.s_d) / ((idx + 1) * dt);
    this->state.d_dd = (d_d - this->state.d_d) / ((idx + 1) * dt);
    setPosition(path_s[idx], path_d[idx]);
    setVelocity(s_d, d_d);
  } else if(idx == 0) {
    double s_d = (path_s[idx], this->state.s) / dt;
    double d_d = (path_d[idx] - this->state.d) / dt;
    this->state.s_dd = (s_d - this->state.s_d) / dt;
    this->state.d_dd = (d_d - this->state.d_d) / dt;
    setPosition(path_s[idx], path_d[idx]);
    setVelocity(s_d, d_d);
  }
}

VehicleState::state Vehicle::getVehicleState() {
  return this->state;
}

VehicleState::state Vehicle::getVehicleStateIn(double dt) {
  
  VehicleState::state stateIn;
  stateIn = getVehicleState();
  
  stateIn.s = stateIn.s + dt * stateIn.s_d;
  
  return stateIn;
}
