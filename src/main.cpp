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
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
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

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
  int prev_wp = -1;
  
  while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
  {
    prev_wp++;
  }
  
  int wp2 = (prev_wp+1)%maps_x.size();
  
  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);
  
  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);
  
  double perp_heading = heading-pi()/2;
  
  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);
  
  return {x,y};
  
}


vector<double> getXYspline(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
  // todo: only fit spline using a couple of waypoints
  vector<double> maps_dx(maps_x.size());
  vector<double> maps_dy(maps_y.size());
  
  adjacent_difference(maps_x.begin(), maps_x.end(), maps_dx.begin());
  adjacent_difference(maps_y.begin(), maps_y.end(), maps_dy.begin());
  
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

vector<double> JMT(vector< double> start, vector <double> end, double T)
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

void planner_follow_waypoints(vector<double> &next_x_vals, vector<double> &next_y_vals, const double car_x, const double car_y, const double car_yaw, const vector<double> map_waypoints_x, const vector<double> map_waypoints_y, const vector<double> map_waypoints_s)
{
  vector<double> frenet_sd;
  frenet_sd = getFrenet(car_x, car_y, car_yaw, map_waypoints_x, map_waypoints_y);
  
  double dist_inc = 0.5;
  for(int i = 0; i < 50; i++)
  {
    double s = frenet_sd[0] + dist_inc*i;
    double d = 2+4;
    
    vector<double> global_xy;
    global_xy = getXYspline(s, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    
    next_x_vals.push_back(global_xy[0]);
    next_y_vals.push_back(global_xy[1]);
  }
}

void print(const vector<double> &path) {
  for (int i=0; i<path.size(); i++) {
    std::cout << path[i] << ", ";
  }
  std::cout << std::endl;
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
          
          
          // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          
          planner_follow_waypoints(next_x_vals, next_y_vals, car_x, car_y, car_yaw, map_waypoints_x, map_waypoints_y, map_waypoints_s);
          
          double dist_inc = 0.5;
          
          vector<double> frenet_sd;
          frenet_sd = getFrenet(car_x, car_y, car_yaw, map_waypoints_x, map_waypoints_y);
          /*
          vector< double> start;
          start = {car_s, car_speed, 0};
          
          vector <double> end;
          end = {car_s+25, 25, 0};
          
          vector<double> poly;
          poly = JMT(start, end, 1);
          std::cout << "Coefficients: " << poly[0] << ", " << poly[1] << ", " << poly[2] << ", " << poly[3] << ", " << poly[4] << ", " << poly[5] << std::endl;
          
          for(int i = 0; i < 50; i++)
          {
            float t = 0.02*i;
            float s = poly[0] + poly[1] * t + poly[2] * pow(t,2) + poly[3] * pow(t,3) + poly[4] * pow(t,4) + poly[5] * pow(t,5);
            float d = 2+4;
            
            vector<double> global_xy;
            global_xy = getXYspline(s, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            
            next_x_vals.push_back(global_xy[0]);
            next_y_vals.push_back(global_xy[1]);
            
          }
          
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
          std::cout << "car_s = " << car_s << ", frenet_sd[0] = " << frenet_sd[0] << std::endl;
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
















































































