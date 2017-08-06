//
//  vehicle.cpp
//  Path_Planning
//
//  Created by Klemens on 06.08.17.
//
//

#include "Vehicle.h"

Vehicle::Vehicle() {
  
  this->s = 0;
  this->s_d = 0;
  this->s_dd = 0;
  this->d = 0;
  this->d_d = 0;
  this->d_dd = 0;
}

void Vehicle::setPosition(double s, double d) {
  this->s = s;
  this->d = d;
}

void Vehicle::setVelocity(double s_d, double d_d) {
  this->s_d = s_d;
  this->d_d = d_d;
}

Vehicle::~Vehicle() {}

void move(const vector<double>& path_s, const vector<double>& path_d, size_t steps) {
  // TODO
}
