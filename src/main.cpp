#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

// returns the absolute velocity of a vehicle
double get_vehicle_speed(vector<double> vehicle) {
  return sqrt(vehicle[3]*vehicle[3] + vehicle[4]*vehicle[4]);
}

// returns the closest vehicle in a given lane, that is within a distance buffer either forward or backward
vector<double> get_vehicle(double s,
                           int lane,
                           vector<vector<double>> sensor_fusion,
                           int prev_size,
                           double buffer) {
  // find ref_v to use
  vector<vector<double>> found_vehicles;
  for (int i = 0; i < sensor_fusion.size(); i++) {
    // car is in my lane
    float d = sensor_fusion[i][6];
    if (d < (2 + 4*lane + 2) && d > (2 + 4*lane - 2)) {
      double check_speed = get_vehicle_speed(sensor_fusion[i]);
      double check_car_s = sensor_fusion[i][5];
      // project s value to the next cycle
      check_car_s += (double)prev_size * .02 * check_speed;
      // check s values greater than mine and less than an s gap
      // checking vehicles ahead
      if (buffer >= 0 && check_car_s > s && check_car_s - s < buffer){
        found_vehicles.push_back(sensor_fusion[i]);
      }
      // buffer < 0, checking vehicles behind
      else if (buffer < 0 && check_car_s < s && s - check_car_s < buffer){
        found_vehicles.push_back(sensor_fusion[i]);
      }
    }
  }
  // sort found vehicles based on s value
  if (buffer >= 0) { // vehicles ahead of us
    std::sort(found_vehicles.begin(),
              found_vehicles.end(),
              [](const vector<double>& veh1, vector<double>& veh2) {
      return veh1[5] < veh2[5]; // ascending order based on s, hence first element is closest
    });
  } else { // vehicles behind us
    std::sort(found_vehicles.begin(),
              found_vehicles.end(),
              [](const vector<double>& veh1, vector<double>& veh2) {
      return veh1[5] > veh2[5]; // descending order based on s, hence first element is closest
    });
  }
  vector<double> empty;
  // std::cout << "found vehicles " << found_vehicles.size() << std::endl;
  if (found_vehicles.size() > 0) return found_vehicles.front();
  else return empty;
}

// Decides reference velocity and best lane, based on sensor fusion information
void behavior(double s,
              double d,
              vector<vector<double>> sensor_fusion,
              double &ref_vel,
              int &lane,
              int prev_size,
              double buffer = 30.0) {

  vector<double> left_front_car = get_vehicle(s, 0, sensor_fusion, prev_size, buffer);
  vector<double> mid_front_car = get_vehicle(s, 1, sensor_fusion, prev_size, buffer);
  vector<double> right_front_car = get_vehicle(s, 2, sensor_fusion, prev_size, buffer);
  vector<double> left_back_car = get_vehicle(s, 0, sensor_fusion, prev_size, -buffer/3);
  vector<double> mid_back_car = get_vehicle(s, 1, sensor_fusion, prev_size, -buffer/3);
  vector<double> right_back_car = get_vehicle(s, 2, sensor_fusion, prev_size, -buffer/3);
  
  double left_cost = 0.0;
  double mid_cost = 0.0;
  double right_cost = 0.0;
  
  if (!left_front_car.empty()) left_cost += 49.5 - get_vehicle_speed(left_front_car);
  if (!mid_front_car.empty()) mid_cost += 49.5 - get_vehicle_speed(mid_front_car);
  if (!right_front_car.empty()) right_cost += 49.5 - get_vehicle_speed(right_front_car);
  if (lane == 0) left_cost -= 3;
  if (lane == 1) mid_cost -= 3;
  if (lane == 2) right_cost -= 3;
  
  // it's possible and worth changing lanes to right
  if (lane == 0 && mid_cost < left_cost && mid_back_car.empty()) lane++;
  if (lane == 1 && right_cost < mid_cost && right_cost < left_cost && right_back_car.empty()) lane++;
  // it's possible and worth changing lanes to left
  if (lane == 2 && mid_cost < right_cost && mid_back_car.empty()) lane--;
  if (lane == 1 && left_cost < mid_cost && left_cost < right_cost && left_back_car.empty()) lane--;
  
  vector<double> target_vehicle = get_vehicle(s, lane, sensor_fusion, prev_size, buffer);
  if (!target_vehicle.empty()) {
    double target_speed = get_vehicle_speed(target_vehicle);
    if (ref_vel/2.24 > target_speed) {
      ref_vel -= .224;
    } else if (ref_vel/2.24 < target_speed - 0.5) {
      ref_vel += .224;
    }
  }
  else if (ref_vel < 49.5) {
    ref_vel += .224;
  }
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
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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
  
  // TODO: Initializing parameters
  // start in lane 1
  int lane = 1;
  
  // have a reference velocity to target
  double ref_vel = 0.0; // mph
  // END OF TODO
  
  h.onMessage([&ref_vel,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&lane]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
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
          vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];
          
          // TODO: define a path made up of (x,y) points that the car will visit

          int prev_size = previous_path_x.size();
          std::cout << prev_size << std::endl;
          if (prev_size > 0) {
            car_s = end_path_s;
          }

          behavior(car_s, car_d, sensor_fusion, ref_vel, lane, prev_size);
          
          // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
          vector<double> ptsx;
          vector<double> ptsy;
          
          // reference x,y, yaw states
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // if previous size is almost empty, use the car as starting reference
          if(prev_size < 2){
            // use 2 points that make the path tangent to the car
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          } else { // use the previous path's end point as starting reference
            // redefine reference state as previous path end point
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];
            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }
          
          // in Frenet add evenly 30m spaced points ahead of the starting reference
          vector<double> next_wp0 = getXY(car_s+30,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
          
          for (int i = 0; i < ptsx.size(); i++) {
            // shift car reference angle to 0 degrees
            double shift_x = ptsx[i]-ref_x;
            double shift_y = ptsy[i]-ref_y;
            ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
            ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
          }

          // create a spline
          tk::spline s;
          // set (x,y) points to the spline
          s.set_points(ptsx,ptsy);

          // define the actual (x,y) points we will use for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // start with all the previous path points from last time
          for (int i = 0; i < prev_size; i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          // calculate how to break up spline points so that we travel at our desired reference velocity
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x * target_x) + (target_y * target_y));
          double x_add_on = 0;
          
          // fill up the rest of our path planner, after the previous points, up to 50
          for (int i = 1; i <= 50 - prev_size; i++) {
            double N = target_dist/(.02*ref_vel/2.24);
            double x_point = x_add_on + target_x / N;
            double y_point = s(x_point);
            x_add_on = x_point;
            double x_ref = x_point;
            double y_ref = y_point;
            // rotating back to normal
            x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);
            x_point += ref_x;
            y_point += ref_y;
            
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }
          
          // END OF TODO

          json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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