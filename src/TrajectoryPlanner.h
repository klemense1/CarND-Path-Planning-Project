// Copyright [2017] Klemens Esterle

#ifndef SRC_TRAJECTORYPLANNER_H_
#define SRC_TRAJECTORYPLANNER_H_

#include <vector>

#include "World.h"
#include "VehicleState.h"

class TrajectoryPlanner {
private:
  std::vector<double> last_trajectory_s;
  std::vector<double> last_trajectory_d;
  
  std::vector<std::vector<double>> createTrajectoryFrenet(VehicleState::state currentState, VehicleState::state goalState);
  std::vector<double> JMT(const std::vector< double> start, const std::vector <double> end, double T);
  double evalCoefficients(std::vector<double> poly, double t);

public:
  std::vector<std::vector<double>> createTrajectoryXY(VehicleState::state currentState, VehicleState::state goalState, World world);
  const std::vector<std::vector<double>> getLastSentTrajectory();
  int getLastSentTrajectoryLength();
  void chopLastSentTrajectory(size_t prev_path_length);
};

#endif  // SRC_TRAJECTORYPLANNER_H_
