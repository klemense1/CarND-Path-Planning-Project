//
//  Waypoints.h
//  Path_Planning
//
//  Created by Klemens on 06.08.17.
//
//

#ifndef Waypoints_h
#define Waypoints_h
#include <vector>

using namespace std;

class Waypoints {

private:
  double distance(const double x1, const double y1, const double x2, const double y2);
  
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;
public:
  
  explicit Waypoints(string file_name);
  int ClosestWaypoint(const double x, const double y, const vector<double> &maps_x, const vector<double> &maps_y);
  int NextWaypoint(const double x, const double y, const double theta, const vector<double> &maps_x, const vector<double> &maps_y);
  void FitWaypointsDetail(const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y, const vector<double> &maps_dx, const vector<double> &maps_dy, vector<double> &maps_s2, vector<double> &maps_x2, vector<double> &maps_y2, vector<double> &maps_dx2, vector<double> &maps_dy2);
  vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);
  vector<double> getXYspline(double s, double d);

};

#endif /* Waypoints_h */
