// Copyright [2017] Klemens Esterle

#ifndef SRC_TRAJECTORYPLANNER_H_
#define SRC_TRAJECTORYPLANNER_H_

#include <vector>
#include <map>

#include "World.h"
#include "VehicleState.h"


class TrajectoryPlanner {
private:
  const double max_acc = 10.;
  const double velocity_max = 21;
  const int lane_width = 4;
  
  std::vector<double> last_trajectory_s;
  std::vector<double> last_trajectory_d;
  
  std::vector<double> JMT(const std::vector< double> &start, const std::vector<double> &end, const double T);
  double evalCoefficients(std::vector<double> poly, double t);

public:
  typedef std::vector<std::vector<double>> Path2d;
  
  const std::vector<std::vector<double>> getLastSentTrajectory();
  int getLastSentTrajectoryLength();
  void chopLastSentTrajectory(size_t prev_path_length);
  
  Path2d createTrajectoryFrenet(const VehicleState::state &currentState, const VehicleState::state &goalState);
  Path2d createTrajectoryXY(const TrajectoryPlanner::Path2d &PathFrenet, const World &world);
  
};

#endif  // SRC_TRAJECTORYPLANNER_H_
