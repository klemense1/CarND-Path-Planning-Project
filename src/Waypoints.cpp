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

#include "json.hpp"
#include "utilities.h"
#include "spline.h"

#include "Waypoints.h"

using namespace std;

Waypoints::Waypoints(string file_name) {
  
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
double Waypoints::distance(const double x1, const double y1, const double x2, const double y2)
{
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int Waypoints::ClosestWaypoint(const double x, const double y, const vector<double> &maps_x, const vector<double> &maps_y)
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

int Waypoints::NextWaypoint(const double x, const double y, const double theta, const vector<double> &maps_x, const vector<double> &maps_y)
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


void Waypoints::FitWaypointsDetail(const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y, const vector<double> &maps_dx, const vector<double> &maps_dy, vector<double> &maps_s2, vector<double> &maps_x2, vector<double> &maps_y2, vector<double> &maps_dx2, vector<double> &maps_dy2) {
  
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
vector<double> Waypoints::getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
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

vector<double> Waypoints::getXYspline(double s, double d)
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
  
}

