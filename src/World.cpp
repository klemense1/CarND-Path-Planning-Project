//
//  Waypoints.cpp
//  Path_Planning
//
//  Created by Klemens on 06.08.17.
//
//

#include <stdio.h>
#include <string>
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <uWS/uWS.h>
#include <thread>
#include <math.h>

#include "spline.h"

#include "World.h"
#include "utilities.h"

using namespace std;

World::World(string file_name) {
  
  ifstream in_map_(file_name.c_str(), ifstream::in);
  
  
  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
    
  }
  
  
}

inline double euclidean(double dx, double dy) {
  return sqrt(dx * dx + dy * dy);
}

inline vector<double> cartesian2polar(double vx, double vy) {
  double speed = euclidean(vx, vy);
  double theta = atan2(vy, vx);
  if(theta < 0) theta += 2 * M_PI;
  return {speed, theta};
}


double World::distance(const double x1, const double y1, const double x2, const double y2)
{
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int World::ClosestWaypoint(const double x, const double y, const vector<double> &maps_x, const vector<double> &maps_y)
{
  
  double closestLen = 100000; //large number
  int closestWaypoint = 0;
  
  for(int i = 0; i < maps_x.size(); i++)
  {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if(dist < closestLen)
    {
      closestLen = dist;
      closestWaypoint = i;
    }
    
  }
  
  return closestWaypoint;
  
}

int World::NextWaypoint(const double x, const double y, const double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
  
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);
  
  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];
  
  double heading = atan2( (map_y-y),(map_x-x) );
  
  double angle = abs(theta-heading);
  
  if(angle > utilities::pi()/4)
  {
    closestWaypoint++;
  }
  
  return closestWaypoint;
  
}


void World::FitWaypointsDetail(const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y, const vector<double> &maps_dx, const vector<double> &maps_dy, vector<double> &maps_s2, vector<double> &maps_x2, vector<double> &maps_y2, vector<double> &maps_dx2, vector<double> &maps_dy2) {
  
  double s_max = maps_s[maps_s.size()-1];
  
  tk::spline spline_x;
  spline_x.set_points(maps_s, maps_x);
  
  tk::spline spline_y;
  spline_y.set_points(maps_s, maps_y);
  
  tk::spline spline_dx;
  spline_dx.set_points(maps_s, maps_dx);
  
  tk::spline spline_dy;
  spline_dy.set_points(maps_s, maps_dy);
  
  for (int i=0; i<s_max*1; i++) {
    double s = i*1;
    maps_s2.push_back(s);
    maps_x2.push_back(spline_x(s));
    maps_y2.push_back(spline_y(s));
    maps_dx2.push_back(spline_x(s));
    maps_dy2.push_back(spline_y(s));
  }
  
}


// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> World::getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
  
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);
  
  int prev_wp;
  prev_wp = next_wp-1;
  if(next_wp == 0)
  {
    prev_wp  = maps_x.size()-1;
  }
  
  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];
  
  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;
  
  double frenet_d = distance(x_x,x_y,proj_x,proj_y);
  
  //see if d value is positive or negative by comparing it to a center point
  
  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);
  
  if(centerToPos <= centerToRef)
  {
    frenet_d *= -1;
  }
  
  // calculate s value
  double frenet_s = 0;
  for(int i = 0; i < prev_wp; i++)
  {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }
  
  frenet_s += distance(0,0,proj_x,proj_y);
  
  return {frenet_s,frenet_d};
  
}

vector<double> World::getXYspline(double s, double d)
{
  /*
   function returns x,y world coordinates for given spatial (frenet) coordinates and x(s) and y(s)
   */
  
  // todo: only fit spline using a couple of waypoints
  
  tk::spline spline_x;
  spline_x.set_points(map_waypoints_s, map_waypoints_x);
  
  tk::spline spline_y;
  spline_y.set_points(map_waypoints_s, map_waypoints_y);
  
  double heading = atan2(spline_y.deriv(1,s),spline_x.deriv(1,s));
  double perp_heading = heading-utilities::pi()/2;
  
  double x = spline_x(s) + d*cos(perp_heading);
  double y = spline_y(s) + d*sin(perp_heading);
  
  return {x,y};
  
  /*
   double max_s = 6000;
   // Ensure s is [0, max_s]
   // Use log2(N) operations for finding the last passed waypoint
   const vector<double>::iterator &upper = std::upper_bound(map_waypoints_s.begin(), map_waypoints_s.end(), s);
   long prev_wp = upper - map_waypoints_s.begin();
   prev_wp -= 1;
   
   vector<double> nearest_s;
   vector<double> nearest_x;
   vector<double> nearest_y;
   
   for(int i = -3; i < 5; i++) {
   size_t n = map_waypoints_s.size();
   size_t wp = (n + prev_wp + i) % n;
   nearest_x.push_back(map_waypoints_x[wp] + d * map_waypoints_dx[wp]);
   nearest_y.push_back(map_waypoints_y[wp] + d * map_waypoints_dy[wp]);
   // Correct for circuit coordinates
   double temp_s = map_waypoints_s[wp];
   if(prev_wp + i < 0) {
   temp_s -= max_s;
   } else if(prev_wp + i >= n) {
   temp_s += max_s;
   }
   nearest_s.push_back(temp_s);
   }
   
   // Get the curve from the nearest 6 waypoints
   tk::spline curve_x;
   tk::spline curve_y;
   curve_x.set_points(nearest_s, nearest_x);
   curve_y.set_points(nearest_s, nearest_y);
   
   double x = curve_x(s);
   double y = curve_y(s);
   
   return {x, y};
   */
  
}


vector<double> World::getFrenetVelocity(double s, double d, double speed, double theta) {
  // Ensure s is [0, max_s]
  s = utilities::bound_s(s);
  
  // Use log2(N) operations for finding the last passed waypoint
  double max_s = 5000;
  const vector<double>::iterator &upper = std::upper_bound(map_waypoints_s.begin(), map_waypoints_s.end(), s);
  long prev_wp = upper - map_waypoints_s.begin();
  prev_wp -= 1;
  
  vector<double> nearest_s;
  vector<double> nearest_x;
  vector<double> nearest_y;
  
  for(int i = -3; i < 5; i++) {
    size_t n = map_waypoints_s.size();
    size_t wp = (n + prev_wp + i) % n;
    nearest_x.push_back(map_waypoints_x[wp] + d * map_waypoints_dx[wp]);
    nearest_y.push_back(map_waypoints_y[wp] + d * map_waypoints_dy[wp]);
    // Correct for circuit coordinates
    double temp_s = map_waypoints_s[wp];
    if(prev_wp + i < 0) {
      temp_s -= max_s;
    } else if(prev_wp + i >= n) {
      temp_s += max_s;
    }
    nearest_s.push_back(temp_s);
  }
  
  // Get the curve from the nearest 6 waypoints
  tk::spline curve_x;
  tk::spline curve_y;
  curve_x.set_points(nearest_s, nearest_x);
  curve_y.set_points(nearest_s, nearest_y);
  
  double x = curve_x(s);
  double y = curve_y(s);
  double x2 = curve_x(s + 1);
  double y2 = curve_y(s + 1);
  
  double road_angle = atan2(y2 - y, x2 - x);
  if(road_angle < 0) road_angle += 2 * M_PI;
  double diff = theta - road_angle;
  
  double s_dot = fabs(speed * cos(diff));
  double d_dot = speed * sin(diff);
  
  return {s_dot, d_dot};
}

void World::setCarMapData(const nlohmann::json j) {
  auto sensor_fusion = j[1]["sensor_fusion"];
  
  for(auto sensorData : sensor_fusion) {
    int idnbr = sensorData[0];
    if(this->vehicleMap.find(idnbr) == this->vehicleMap.end()) {
      this->vehicleMap[idnbr].id = 0;
    }
    
    // Update other cars' states
    this->vehicleMap[idnbr].setPosition(sensorData[5], sensorData[6]);
    auto polar = cartesian2polar(sensorData[3], sensorData[4]);
    auto other_sd_dot = getFrenetVelocity(this->vehicleMap[idnbr].state.s, this->vehicleMap[idnbr].state.d, polar[0], polar[1]);
    this->vehicleMap[idnbr].setVelocity(other_sd_dot[0], other_sd_dot[1]);
  }
}
