// Copyright [2017] Klemens Esterle

#include "Vehicle.h"

#include <iostream>
#include <utility>
#include <vector>

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
  std::streamsize init_precision = stream.precision();
  stream.precision(2);
  stream << "Vehicle: ";
  stream << "s(" << std::fixed << vehicle.state.s << ", " << std::fixed << vehicle.state.s_d << ", " << std::fixed << vehicle.state.s_dd << ") | ";
  stream << "d(" << std::fixed << vehicle.state.d << ", " << std::fixed << vehicle.state.d_d << ", " << std::fixed << vehicle.state.d_dd << ") | ";
  stream << "Lane " << std::fixed << VehicleState::getLane(vehicle.state) << std::fixed << " | ";;
  stream.precision(init_precision);
  return stream;
}

void Vehicle::move(const std::vector<double>& path_s, const std::vector<double>& path_d, size_t steps) {
  int idx = steps - 1;
  double dt = 0.02;
  if (idx > 0) {
    double s_d = (path_s[idx] - path_s[idx - 1]) / dt;
    double d_d = (path_d[idx] - path_d[idx - 1]) / dt;
    this->state.s_dd = (s_d - this->state.s_d) / ((idx + 1) * dt);
    this->state.d_dd = (d_d - this->state.d_d) / ((idx + 1) * dt);
    setPosition(path_s[idx], path_d[idx]);
    setVelocity(s_d, d_d);
  } else if (idx == 0) {
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
