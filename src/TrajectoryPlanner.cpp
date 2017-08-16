// Copyright [2017] Klemens Esterle

#include "TrajectoryPlanner.h"

#include <stdio.h>
#include <vector>
#include <algorithm>
#include <cmath>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"

#include "Parameters.h"

TrajectoryPlanner::Path2d TrajectoryPlanner::createTrajectoryFrenet(VehicleState::state currentState, VehicleState::state goalState) {
  size_t delay = 0;  //min(last_trajectory_s.size(), Parameters::n_steps_react);
  
  // Copy the path for the delay
  std::vector<double> next_path_s(last_trajectory_s.begin(), last_trajectory_s.begin() + delay);
  std::vector<double> next_path_d(last_trajectory_d.begin(), last_trajectory_d.begin() + delay);
  
  size_t n_future_steps = Parameters::n_steps - delay;
  double time_prediction = n_future_steps * Parameters::dt;
  
  std::vector<double> poly_s = JMT({currentState.s, currentState.s_d, currentState.s_dd}, {goalState.s, goalState.s_d, goalState.s_dd}, time_prediction);
  std::vector<double> poly_d = JMT({currentState.d, currentState.d_d, currentState.d_dd}, {goalState.d, goalState.d_d, goalState.d_dd}, time_prediction);
  
  double last_s = currentState.s;
  
  for (int i = 1; i < n_future_steps; i++) {
    double s = std::max(evalCoefficients(poly_s, i*Parameters::dt), last_s + 0.02);  // TODO verbessern
    double d = evalCoefficients(poly_d, i*Parameters::dt);
    
    next_path_s.push_back(s);
    next_path_d.push_back(d);
    last_s = s;
  }
  return {next_path_s, next_path_d};
}

double TrajectoryPlanner::evalCoefficients(std::vector<double> poly, double t) {
  return poly[0] + poly[1] * t + poly[2] * pow(t, 2) + poly[3] * pow(t, 3) + poly[4] * pow(t, 4) + poly[5] * pow(t, 5);
}

std::vector<double> TrajectoryPlanner::JMT(const std::vector< double> start, const std::vector<double> end, double T) {
  /*
   Calculate the Jerk Minimizing Trajectory that connects the initial state
   to the final state in time T.
   
   INPUTS
   
   start - the vehicles start location given as a length three array
   corresponding to initial values of [s, s_dot, s_double_dot]
   
   end   - the desired end state for vehicle. Like "start" this is a
   length three array.
   
   T     - The duration, in seconds, over which this maneuver should occur.
   
   OUTPUT
   an array of length 6, each value corresponding to a coefficent in the polynomial
   s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
   
   EXAMPLE
   
   > JMT( [0, 10, 0], [10, 10, 0], 1)
   [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
   */
  
  Eigen::MatrixXd A = Eigen::MatrixXd(3, 3);
  A << T*T*T, T*T*T*T, T*T*T*T*T,
  3*T*T, 4*T*T*T, 5*T*T*T*T,
  6*T, 12*T*T, 20*T*T*T;
  
  Eigen::MatrixXd B = Eigen::MatrixXd(3, 1);
  B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
  end[1]-(start[1]+start[2]*T),
  end[2]-start[2];
  
  Eigen::MatrixXd Ai = A.inverse();
  
  Eigen::MatrixXd C = Ai*B;
  
  std::vector <double> result = {start[0], start[1], .5*start[2]};
  for (int i = 0; i < C.size(); i++) {
    result.push_back(C.data()[i]);
  }
  
  return result;
}

const TrajectoryPlanner::Path2d TrajectoryPlanner::getLastSentTrajectory() {
  return {last_trajectory_s, last_trajectory_d};
}

int TrajectoryPlanner::getLastSentTrajectoryLength() {
  return last_trajectory_s.size();
}

void TrajectoryPlanner::chopLastSentTrajectory(size_t prev_path_length) {
  size_t n_processed_steps = last_trajectory_s.size() - prev_path_length;
  
  if (prev_path_length > 0 && n_processed_steps > 0) {
    last_trajectory_s.erase(last_trajectory_s.begin(), last_trajectory_s.begin() + n_processed_steps);
    last_trajectory_d.erase(last_trajectory_d.begin(), last_trajectory_d.begin() + n_processed_steps);
  } else if (prev_path_length == 0) {
    last_trajectory_s.clear();
    last_trajectory_d.clear();
  }
}

TrajectoryPlanner::Path2d TrajectoryPlanner::createTrajectoryXY(VehicleState::state currentState, VehicleState::state goalState, World world) {
  TrajectoryPlanner::Path2d path = createTrajectoryFrenet(currentState, goalState);
  
  last_trajectory_s = path[0];
  last_trajectory_d = path[1];
  
  std::vector<double> next_path_x;
  std::vector<double> next_path_y;
  
  for (int i = 0; i < getLastSentTrajectoryLength(); i++) {
    std::vector<double> positionXY = world.getXYspline(last_trajectory_s[i], last_trajectory_d[i]);  // TODO überprüfen
    next_path_x.push_back(positionXY[0]);
    next_path_y.push_back(positionXY[1]);
  }
  
  return {next_path_x, next_path_y};
}

TrajectoryPlanner::Path2d TrajectoryPlanner::createBestTrajectoryXY(VehicleState::state currentState, std::map<int, VehicleState::state> GoalMap, World world) {
  
  double best_costs = 1000000;
  TrajectoryPlanner::Path2d newPath;
  VehicleState::state newGoal;
  
  double lane_d = fabs(currentState.d - getLane(currentState));
  
  // Finish previous maneuver before new ones
  if ((getLane(currentState) != _last_lane) || (lane_d > TrajectoryPlanner::lane_width / 4)) {
    VehicleState::state newGoal = GoalMap[_last_lane];
    newPath = createTrajectoryXY(currentState, newGoal, world);
  } else {
    for (auto& item: GoalMap) {
      VehicleState::state goalState = item.second;
      TrajectoryPlanner::Path2d currentPath = createTrajectoryXY(currentState, goalState, world);
      double current_costs = TrajectoryPlanner::costFunction(currentPath);
      std::cout << "Lane " << item.first << " with costs " << current_costs << std::endl;
      if (item.first == 2) {
        newPath = currentPath;
        newGoal = goalState;
      }
    }
  }
  _last_lane = getLane(newGoal);
  
  return newPath;
}

std::vector<double> getFirstDerivative(std::vector<double> pts) {
  
  std::vector<double> pts_d(pts.size());
  adjacent_difference(pts.begin(), pts.end(), pts_d.begin());
  
  pts_d.erase(pts_d.begin());
  return pts_d;
}

std::vector<double> getSecondDerivative(std::vector<double> points) {
  
  std::vector<double> points_d = getFirstDerivative(points);
  std::vector<double> points_dd = getFirstDerivative(points_d);
  
  return points_dd;
}

std::vector<double> getThirdDerivative(std::vector<double> points) {
  
  std::vector<double> points_dd = getSecondDerivative(points);
  std::vector<double> points_ddd = getFirstDerivative(points_dd);
  
  return points_ddd;
}

double TrajectoryPlanner::costsJerk(TrajectoryPlanner::Path2d Path) {
  std::vector<double> s_jerk = getThirdDerivative(Path[0]);
  std::vector<double> d_jerk = getThirdDerivative(Path[1]);
  
  double costs = 0;
  
  for (int i = 0; i < s_jerk.size(); i++) {
    double jerk = sqrt(s_jerk[i] * s_jerk[i] + d_jerk[i] * d_jerk[i]);
    costs += jerk;
  }
  return costs;
}

double TrajectoryPlanner::costsAcceleration(TrajectoryPlanner::Path2d Path) {
  std::vector<double> s_dd = getSecondDerivative(Path[0]);
  std::vector<double> d_dd = getSecondDerivative(Path[1]);
  
  double costs = 0;
  
  for (int i = 0; i < s_dd.size(); i++) {
    double acc = sqrt(s_dd[i] * s_dd[i] + d_dd[i] * d_dd[i]);
    costs += acc;
  }
  return costs;
}

double TrajectoryPlanner::costsVelocity(TrajectoryPlanner::Path2d Path) {
  std::vector<double> s_d = getFirstDerivative(Path[0]);
  std::vector<double> d_d = getFirstDerivative(Path[1]);
  
  double costs = 0;
  
  for (int i = 0; i < s_d.size(); i++) {
    double vel = sqrt(s_d[i] * s_d[i] + d_d[i] * d_d[i]);
    costs += std::abs(vel-TrajectoryPlanner::velocity_max);
  }
  return costs;
}

double TrajectoryPlanner::costFunction(TrajectoryPlanner::Path2d Path) {
  
  double total_costs = 0;
  total_costs += costsVelocity(Path);
  total_costs += costsAcceleration(Path);
  total_costs += costsJerk(Path);
  
  return total_costs;
}
