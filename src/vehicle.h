// Copyright [2017] Klemens Esterle

#ifndef SRC_VEHICLE_H_
#define SRC_VEHICLE_H_

#include <vector>
#include <sstream>
#include <utility>
#include <tgmath.h>

#include "VehicleState.h"

class Vehicle {
private:
  const double max_s = 6945.554;

  inline double bound_s(double s) {
    double bounded_s;
    if (s < 0 || s > max_s) {
      bounded_s = fmod(max_s + s, max_s);
    } else {
      bounded_s = s;
    }
    return bounded_s;
  }
  
  inline double bound_s_difference(double s1, double s2) {
    auto a = bound_s(s1);
    auto b = bound_s(s2);
    auto diff = a - b;
    if (diff < -max_s / 2) diff += max_s;
    if (diff > max_s / 2) diff -= max_s;
    return diff;
  }
  
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
