#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

#include "Eigen-3.3/Eigen/Dense"
#include "spline.h"
#include <numeric>      // std::adjacent_difference
#include <chrono>

std::chrono::steady_clock::time_point last_time = std::chrono::steady_clock::now();

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
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

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
  
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);
  
  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];
  
  double heading = atan2( (map_y-y),(map_x-x) );
  
  double angle = abs(theta-heading);
  
  if(angle > pi()/4)
  {
    closestWaypoint++;
  }
  
  return closestWaypoint;
  
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
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

/*
 from https://stackoverflow.com/questions/11734322/matlab-type-arrays-in-c
 */
vector<double> linspace(double a, double b, int n) {
  vector<double> array;
  if ((n == 0) || (n == 1) || (a == b))
    array.push_back(b);
  else if (n > 1) {
    double step = (b - a) / (n - 1);
    int count = 0;
    while(count < n) {
      array.push_back(a + count*step);
      ++count;
    }
  }
  return array;
}


void fitToDetailedCurve(const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y, const vector<double> &maps_dx, const vector<double> &maps_dy, vector<double> &maps_s2, vector<double> &maps_x2, vector<double> &maps_y2, vector<double> &maps_dx2, vector<double> &maps_dy2) {
  
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

vector<double> getXYspline(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
  /*
  function returns x,y world coordinates for given spatial (frenet) coordinates and x(s) and y(s)
   */
  
  // todo: only fit spline using a couple of waypoints
  
  tk::spline spline_x;
  spline_x.set_points(maps_s, maps_x);
  
  tk::spline spline_y;
  spline_y.set_points(maps_s, maps_y);
  
  double heading = atan2(spline_y.deriv(1,s),spline_x.deriv(1,s));
  double perp_heading = heading-pi()/2;
  
  double x = spline_x(s) + d*cos(perp_heading);
  double y = spline_y(s) + d*sin(perp_heading);
  
  return {x,y};
  
}

vector<double> JMT(const vector< double> start, const vector <double> end, double T)
{
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
  3*T*T, 4*T*T*T,5*T*T*T*T,
  6*T, 12*T*T, 20*T*T*T;
		
  Eigen::MatrixXd B = Eigen::MatrixXd(3,1);
  B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
  end[1]-(start[1]+start[2]*T),
  end[2]-start[2];
  
  Eigen::MatrixXd Ai = A.inverse();
  
  Eigen::MatrixXd C = Ai*B;
  
  vector <double> result = {start[0], start[1], .5*start[2]};
  for(int i = 0; i < C.size(); i++)
  {
    result.push_back(C.data()[i]);
  }
  
  return result;
  
}

void print(const vector<double> &path) {
  for (int i=0; i<path.size(); i++) {
    std::cout << path[i] << ", ";
  }
  std::cout << std::endl;
}

void planner_follow_waypoints(vector<double> &next_x_vals, vector<double> &next_y_vals, const double car_x, const double car_y, const double car_yaw, const vector<double> map_waypoints_x, const vector<double> map_waypoints_y, const vector<double> map_waypoints_s)
{
  vector<double> frenet_sd;
  frenet_sd = getFrenet(car_x, car_y, car_yaw, map_waypoints_x, map_waypoints_y);
  
  double dist_inc = 0.5;
  for(int i = 0; i < 50; i++)
  {
    double s = frenet_sd[0] + dist_inc*(i+1);
    double d = 2+4;
    
    vector<double> global_xy;
    global_xy = getXYspline(s, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    
    next_x_vals.push_back(global_xy[0] + cos(car_yaw));
    next_y_vals.push_back(global_xy[1] + sin(car_yaw));

    std::cout << "i = " << i << "next_x_vals: " << next_x_vals[i] << "next_y_vals: " << next_y_vals[i] << std::endl;

  }
}


void planner_follow_waypoints(vector<double> &next_x_vals, vector<double> &next_y_vals, const double car_x, const double car_y, const double car_yaw, const vector<double> map_waypoints_x, const vector<double> map_waypoints_y, const vector<double> map_waypoints_dx, const vector<double> map_waypoints_dy, const vector<double> map_waypoints_s, const vector<double> previous_path_x, const vector<double> previous_path_y)
{

  double dist_inc = 0.44; // making roughly 50mph
  
  double pos_x;
  double pos_y;
  double angle;
  
  int prev_path_size = previous_path_x.size();
  int keep_path_size = std::min(prev_path_size, 5);

  std::cout << "old" << std::endl;

  // reusing old path
  for(int i = 0; i < keep_path_size; i++)
  {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
    std::cout << "i = " << i << "next_x_vals: " << next_x_vals[i] << "next_y_vals: " << next_y_vals[i] << std::endl;
  }
  
  if(keep_path_size == 0)
  {
    pos_x = car_x;
    pos_y = car_y;
    angle = deg2rad(car_yaw);
  }
  else
  {
    // calculate angle
    pos_x = previous_path_x[keep_path_size-1];
    pos_y = previous_path_y[keep_path_size-1];
    
    double pos_x2 = previous_path_x[keep_path_size-2];
    double pos_y2 = previous_path_y[keep_path_size-2];
    angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
  }
  
  vector<double> map_waypoints_s2;
  vector<double> map_waypoints_x2;
  vector<double> map_waypoints_y2;
  vector<double> map_waypoints_dx2;
  vector<double> map_waypoints_dy2;
  
  fitToDetailedCurve(map_waypoints_s, map_waypoints_x, map_waypoints_y, map_waypoints_dx, map_waypoints_dy, map_waypoints_s2, map_waypoints_x2, map_waypoints_y2, map_waypoints_dx2, map_waypoints_dy2);
  
  vector<double> frenet_sd;
  frenet_sd = getFrenet(pos_x, pos_y, angle, map_waypoints_x2, map_waypoints_y2);
  
  std::cout << "new" << std::endl;
  
  for(int i = 0; i < 50-keep_path_size; i++)
  {
    
    double s = frenet_sd[0] + dist_inc*i;
    double d = 2+4;
    
    vector<double> global_xy = getXYspline(s, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    
    double pos_x2 = pos_x;
    double pos_y2 = pos_y;
    
    pos_x = global_xy[0];
    pos_y = global_xy[1];

    //angle = atan2(pos_y-pos_y2,pos_x-pos_x2);

    //std::cout << "angle: " << angle << std::endl;
    
    next_x_vals.push_back(pos_x+dist_inc*cos(angle));
    next_y_vals.push_back(pos_y+dist_inc*sin(angle));
    
    std::cout << "i = " << keep_path_size + i << "next_x_vals: " << pos_x+dist_inc*cos(angle) << "next_y_vals: " << pos_y+dist_inc*sin(angle) << std::endl;
  }
}

void planner_quintic_polynomials(vector<double> &next_x_vals, vector<double> &next_y_vals, const double car_x, const double car_y, const double car_yaw, const vector<double> map_waypoints_x, const vector<double> map_waypoints_y, const vector<double> map_waypoints_dx, const vector<double> map_waypoints_dy, const vector<double> map_waypoints_s, const vector<double> previous_path_x, const vector<double> previous_path_y)
{
  
  
  double dist_inc = 0.44; // making roughly 50mph
  
  double pos_x;
  double pos_y;
  double angle;
  
  int prev_path_size = previous_path_x.size();
  int keep_path_size = std::min(prev_path_size, 5);
  
  std::cout << "old" << std::endl;
  
  // reusing old path
  for(int i = 0; i < keep_path_size; i++)
  {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
    std::cout << "i = " << i << "next_x_vals: " << next_x_vals[i] << "next_y_vals: " << next_y_vals[i] << std::endl;
  }
  
  if(keep_path_size == 0)
  {
    pos_x = car_x;
    pos_y = car_y;
    angle = deg2rad(car_yaw);
  }
  else
  {
    // calculate angle
    pos_x = previous_path_x[keep_path_size-1];
    pos_y = previous_path_y[keep_path_size-1];
    
    double pos_x2 = previous_path_x[keep_path_size-2];
    double pos_y2 = previous_path_y[keep_path_size-2];
    angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
  }
  
  vector<double> map_waypoints_s2;
  vector<double> map_waypoints_x2;
  vector<double> map_waypoints_y2;
  vector<double> map_waypoints_dx2;
  vector<double> map_waypoints_dy2;
  
  fitToDetailedCurve(map_waypoints_s, map_waypoints_x, map_waypoints_y, map_waypoints_dx, map_waypoints_dy, map_waypoints_s2, map_waypoints_x2, map_waypoints_y2, map_waypoints_dx2, map_waypoints_dy2);
  
  vector<double> frenet_sd;
  frenet_sd = getFrenet(pos_x, pos_y, angle, map_waypoints_x2, map_waypoints_y2);
  
  std::cout << "new" << std::endl;
  
  for(int i = 0; i < 50-keep_path_size; i++)
  {
    
    double s = frenet_sd[0] + dist_inc*i;
    double d = 2+4;
    
    vector<double> global_xy = getXYspline(s, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    
    double pos_x2 = pos_x;
    double pos_y2 = pos_y;
    
    pos_x = global_xy[0];
    pos_y = global_xy[1];
    
    //angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
    
    //std::cout << "angle: " << angle << std::endl;
    
    next_x_vals.push_back(pos_x+dist_inc*cos(angle));
    next_y_vals.push_back(pos_y+dist_inc*sin(angle));
    
    std::cout << "i = " << keep_path_size + i << "next_x_vals: " << pos_x+dist_inc*cos(angle) << "next_y_vals: " << pos_y+dist_inc*sin(angle) << std::endl;
  }
}


void smooth_trajectory(const vector<double> x_points, const vector<double> y_points, vector<double> &x_new, vector<double> &y_new) {
  /*
    take planned segments and linearly space them to apply some smoothing
  */
  vector<double> s_points(x_points.size());
  double angle = 0;
  for (int i=0; i<50; i++) {
    vector<double> frenet_sd;
    frenet_sd = getFrenet(x_points[i], y_points[i], angle, x_points, y_points);
    s_points[i] = frenet_sd[0];
  }
  
  s_points[0] = 0; // this should not be necessary, there is a bug in the getFrenet function
  
  tk::spline spline_x;
  spline_x.set_points(s_points, x_points);
  
  tk::spline spline_y;
  spline_y.set_points(s_points, y_points);
  
  vector<double> s_points_linear_spaced;
  s_points_linear_spaced = linspace(s_points[0], s_points[s_points.size()-1], 50);
  
  for (int i=0; i<50; i++) {
    double s = s_points_linear_spaced[i];
    x_new.push_back(spline_x(s));
    y_new.push_back(spline_y(s));
  }
  
  x_new[0] = x_points[0]; // reusing first point
  y_new[0] = y_points[0];
  
}

int main() {
  uWS::Hub h;
  
  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;
  
  // Waypoint map to read from
  
  string map_file_ = "/Users/Klemens/Udacity_Nano_Car/CarND-Path-Planning-Project/data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;
  
  ifstream in_map_(map_file_.c_str(), ifstream::in);
  
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
  
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                                                                                                       uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      
      auto s = hasData(data);
      
      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();
          double elapsed_secs = std::chrono::duration_cast<std::chrono::microseconds>(current_time - last_time).count() /1000000.0;
          last_time = std::chrono::steady_clock::now();
          
          std::cout<<elapsed_secs<<std::endl;
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];
          
          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];
          
          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          
          json msgJson;
        
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          vector<double> next_x_vals_dummy;
          vector<double> next_y_vals_dummy;
          
          std::cout << "new cycle" << std::endl;
          
          // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          
          planner_follow_waypoints(next_x_vals_dummy, next_y_vals_dummy, car_x, car_y, car_yaw, map_waypoints_x, map_waypoints_y, map_waypoints_dx, map_waypoints_dy, map_waypoints_s, previous_path_x, previous_path_y);
          
          smooth_trajectory(next_x_vals_dummy, next_y_vals_dummy,next_x_vals, next_y_vals);
          //next_x_vals = next_x_vals_dummy;
          //next_y_vals = next_y_vals_dummy;
          /*
          double dist_inc = 0.5;
          
          vector<double> map_waypoints_s2;
          vector<double> map_waypoints_x2;
          vector<double> map_waypoints_y2;
          vector<double> map_waypoints_dx2;
          vector<double> map_waypoints_dy2;
          
          fitToDetailedCurve(map_waypoints_s, map_waypoints_x, map_waypoints_y, map_waypoints_dx, map_waypoints_dy, map_waypoints_s2, map_waypoints_x2, map_waypoints_y2, map_waypoints_dx2, map_waypoints_dy2);
          
          vector<double> frenet_sd;
          frenet_sd = getFrenet(car_x, car_x, car_yaw, map_waypoints_x2, map_waypoints_y2);
          
          vector< double> start;
          double speed_target = 22;
          double time_prediction = 1;
          start = {car_s+car_speed*0.02, speed_target, 0};
          
          vector <double> goal;
          goal = {car_s+speed_target*time_prediction+car_speed*0.02, speed_target, 0};
          
          vector<double> poly;
          poly = JMT(start, goal, time_prediction);
          std::cout << "Coefficients: " << poly[0] << ", " << poly[1] << ", " << poly[2] << ", " << poly[3] << ", " << poly[4] << ", " << poly[5] << std::endl;
          
          for(int i = 0; i < 50; i++)
          {
            float t = 0.02*(i+1);
            float s = poly[0] + poly[1] * t + poly[2] * pow(t,2) + poly[3] * pow(t,3) + poly[4] * pow(t,4) + poly[5] * pow(t,5);
            float d = 2+4;
            
            vector<double> global_xy;
            global_xy = getXYspline(s, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            
            next_x_vals.push_back(global_xy[0]);
            next_y_vals.push_back(global_xy[1]);
            
          }
          
          std::cout << "start: ";
          print(start);
          
          std::cout << "goal: ";
          print(goal);
          
          std::cout << "previous_path_x: ";
          print(previous_path_x);
          
          std::cout << "next_x_vals: ";
          print(next_x_vals);
          
          std::cout << "previous_path_y: ";
          print(previous_path_y);
          
          std::cout << "next_y_vals: ";
          print(next_y_vals);
          
          std::cout << "current x = " << car_x << ", y = " << car_y << std::endl;
          std::cout << "next x = " << next_x_vals[0] << ", y = " << next_y_vals[0] << std::endl;
          //std::cout << "car_s = " << car_s << ", frenet_sd[0] = " << frenet_sd[0] << std::endl;
          */
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;
          
          auto msg = "42[\"control\","+ msgJson.dump()+"]";
          
          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });
  
  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });
  
  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });
  
  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });
  
  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
















































































