//
//  vehicle.h
//  Path_Planning
//
//  Created by Klemens on 06.08.17.
//
//

#ifndef vehicle_h
#define vehicle_h

#include <vector>

using namespace std;

class Vehicle {
public:
  
  double s;
  double s_d;
  double s_dd;
  double d;
  double d_d;
  double d_dd;
  
  explicit Vehicle();
  
  void move(const vector<double>& path_s, const vector<double>& path_d, size_t steps);
  
  void setVelocity(double s_dot, double d_dot);
  
  void setPosition(double s, double d);
  
  virtual ~Vehicle();
};

#endif /* vehicle_h */
