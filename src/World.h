// Copyright [2017] Klemens Esterle

#ifndef SRC_WORLD_H_
#define SRC_WORLD_H_

#include <vector>
#include <map>
#include <string>
#include "json.hpp"

#include "Vehicle.h"


class World {
private:
  double distance(const double x1, const double y1, const double x2, const double y2);
  
  std::vector<double> map_waypoints_x;
  std::vector<double> map_waypoints_y;
  std::vector<double> map_waypoints_s;
  std::vector<double> map_waypoints_dx;
  std::vector<double> map_waypoints_dy;
  
  std::map<int, Vehicle> vehicleMap;
  
public:
  struct Sensordata {
    int id;
    float x;
    float y;
    float vx;
    float vy;
    float s;
    float d;
  };
  
  explicit World(std::string file_name);
  int ClosestWaypoint(const double x, const double y, const std::vector<double> &maps_x, const std::vector<double> &maps_y);
  int NextWaypoint(const double x, const double y, const double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y);
  std::vector<double> getFrenet(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y);
  std::vector<double> getXYspline(double s, double d);
  std::vector<double> getFrenetVelocity(double s, double d, double speed, double theta);
  void setVehicleMapData(const nlohmann::json j);
  std::map<int, Vehicle> getVehicleMap() const;
};

#endif  // SRC_WORLD_H_
