//
//  TrajectoryPlanner.h
//  Path_Planning
//
//  Created by Klemens on 06.08.17.
//
//

#ifndef TrajectoryPlanner_h
#define TrajectoryPlanner_h

#include "World.h"
#include "VehicleState.h"

using namespace std;
class TrajectoryPlanner {
  
private:
  vector<double> last_trajectory_s;
  vector<double> last_trajectory_d;
  
  vector<vector<double>> createTrajectoryFrenet(VehicleState::state currentState, VehicleState::state goalState);
  vector<double> JMT(const vector< double> start, const vector <double> end, double T);
  double evalCoefficients(vector<double> poly, double t);

public:
  vector<vector<double>> createTrajectoryXY(VehicleState::state currentState, VehicleState::state goalState, World world);
  const vector<vector<double>> getLastSentTrajectory();
  int getLastSentTrajectoryLength();
  void chopLastSentTrajectory(size_t prev_path_length);
};

#endif /* TrajectoryPlanner_h */
