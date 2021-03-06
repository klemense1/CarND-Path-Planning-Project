#include <fstream>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#include "json.hpp"

#include <numeric>      // std::adjacent_difference
#include <chrono>
#include "World.h"
#include "TrajectoryPlanner.h"
#include "Vehicle.h"
#include "VehicleState.h"
#include "BehaviorPlanner.h"
//#include "Parameters.h"

constexpr double pi() { return M_PI; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }

std::chrono::steady_clock::time_point last_time = std::chrono::steady_clock::now();

using namespace std;

// for convenience
using json = nlohmann::json;

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


void print(const vector<double> &path) {
  for (int i=0; i<path.size(); i++) {
    std::cout << path[i] << ", ";
  }
  std::cout << std::endl;
}


int main() {
  uWS::Hub h;
  
  // Waypoint map to read from
  string map_file_ = "/Users/Klemens/Udacity_Nano_Car/CarND-Path-Planning-Project/data/highway_map.csv";
  
  World world(map_file_);
  BehaviorPlanner behaviorPlanner;
  
  Vehicle egovehicle;
  int n_cycle = 0;
  
  h.onMessage([&world, &behaviorPlanner, &egovehicle, &n_cycle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
          world.setVehicleMapData(j);
          
          n_cycle++;
          if (n_cycle > 20)
            world._initphase = false;
          
          std::cout << std::endl << "main: new cycle " << n_cycle << std::endl;
          
          // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          
          const size_t n_progressed_points = behaviorPlanner.trajplanner.getLastSentTrajectoryLength() - previous_path_x.size();
          
          if(n_progressed_points > 0) {
            const vector<vector<double>> trajectory = behaviorPlanner.trajplanner.getLastSentTrajectory();
            egovehicle.move(trajectory[0], trajectory[1], n_progressed_points);
          } else {
            const vector<double> &frenetVelocity = world.getFrenetVelocity(j[1]["s"], j[1]["d"], 0.44704 * ((double) j[1]["speed"]), deg2rad(j[1]["yaw"]));
            egovehicle.setPosition(j[1]["s"], j[1]["d"]);
            egovehicle.setVelocity(frenetVelocity[0], frenetVelocity[1]);
          }
          
          behaviorPlanner.trajplanner.chopLastSentTrajectory(previous_path_x.size());
          
          VehicleState::state start = egovehicle.getVehicleState();
          
          vector<vector<double>> path = behaviorPlanner.createBehavior(start, world);
          
          cout<<egovehicle;
          
          json msgJson;
          msgJson["next_x"] = path[0];
          msgJson["next_y"] = path[1];
          
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
















































































