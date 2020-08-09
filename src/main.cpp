#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <iterator>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "configuration.h"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int car_lane = 1;
string car_state = "KL";
double time_since_last_lc = 0;

vector<string> successor_states(int current_lane) {
  // Provides the possible next states given the current state for the FSM 

  vector<string> possible_states;
    
  possible_states.push_back("KL");
  
  if (current_lane == 0)
  {
    possible_states.push_back("LCR");
  }
  else if (current_lane == 1)
  {
    possible_states.push_back("LCL");
    possible_states.push_back("LCR");
  }
  else
  {
    possible_states.push_back("LCL");
  }    
    
  return possible_states;
}

double calculate_speed_cost(int target_lane, double my_speed, vector<int> front_target_id, vector<int> behind_target_id, vector<vector<double>> sensor_fusion) {
  // calculate the future speed based on car in front
  double cost;
            /**
             * Sensor Fusion Data
             * ["sensor_fusion"] A 2d vector of cars and then that car's
             * [0] car's unique ID,
             * [1] car's x position in map coordinates,
             * [2] car's y position in map coordinates,
             * [3] car's x velocity in m/s,
             * [4] car's y velocity in m/s,
             * [5] car's s position in frenet coordinates, 
             * [6] car's d position in frenet coordinates. 
             */
  int target_id = front_target_id[target_lane];
  
  if (target_id == -1)
  {
    cost = 0;
  }
  else
  {
    bool target_found = false;
    double target_x;
    double target_y;
    double target_dx;
    double target_dy;
    double target_s;
    double target_d;

    for (int i = 0; i < sensor_fusion.size(); ++i)
    {
      if (target_id == sensor_fusion[i][0])
      {
        /* loop through to locate our target */
        target_x = sensor_fusion[i][1];
        target_y = sensor_fusion[i][2];
        target_dx = sensor_fusion[i][3];
        target_dy = sensor_fusion[i][4];
        target_s = sensor_fusion[i][5];
        target_d = sensor_fusion[i][6];
        target_found = true;
      }
    }
    
    if (!target_found)
    {
      /* something went wrong shouldve found target */
      cost = 0;
    }
    else
    {
      double target_speed = sqrt(target_dx*target_dx+target_dy*target_dy);
      
      if ((target_speed - my_speed) > Mph2Mps(10.0))
      {
        cost = 0;
      }
      else if (target_speed < Mph2Mps(35))
      {
        /* super slow */
        cost = 1.0;
      }
      else
      {
        cost = (Mph2Mps(35) - target_speed) / (Mph2Mps(SPEED_LIMIT) - Mph2Mps(35));
        cost = std::max(cost,0.0);
      }
    }
  }
  
  return cost;
}

double calculate_contact_cost (int target_lane, double my_speed, double my_s, vector<int> front_target_id, vector<int> behind_target_id, vector<vector<double>> sensor_fusion) {
  /**
    * calculate cost vehicle too close to others
    */
  double cost_1;
  double cost_2;
  double clearance;
  
            /**
             * Sensor Fusion Data
             * ["sensor_fusion"] A 2d vector of cars and then that car's
             * [0] car's unique ID,
             * [1] car's x position in map coordinates,
             * [2] car's y position in map coordinates,
             * [3] car's x velocity in m/s,
             * [4] car's y velocity in m/s,
             * [5] car's s position in frenet coordinates, 
             * [6] car's d position in frenet coordinates. 
             */
  int target_id = front_target_id[target_lane];
  
  if (target_id == -1)
  {
    cost_1 = 0;
  }
  else
  {
    bool target_found = false;
    double target_x;
    double target_y;
    double target_dx;
    double target_dy;
    double target_s;
    double target_d;
    double target_speed;
    
    for (int i = 0; i < sensor_fusion.size(); ++i)
    {
      if (target_id == sensor_fusion[i][0])
      {
        /* loop through to locate our target */
        target_x = sensor_fusion[i][1];
        target_y = sensor_fusion[i][2];
        target_dx = sensor_fusion[i][3];
        target_dy = sensor_fusion[i][4];
        target_s = sensor_fusion[i][5];
        target_d = sensor_fusion[i][6];
        target_speed = sqrt(target_dx*target_dx+target_dy*target_dy);
        target_found = true;
      }
    }
    
    if (!target_found)
    {
      /* something went wrong shouldve found target */
      cost_1 = 0;
    }
    else
    {
      clearance = (target_s + target_speed * 2) - (my_s + my_speed * 2);
      if (clearance < 20.0)
      {
        cost_1 = 1;
      }
      else
      {
        cost_1 = (40.0 - clearance)/20;
        cost_1 = std::max(cost_1,0.0);
      }
    }
  }
  
  /* check from behind */
  target_id = behind_target_id[target_lane];
  
  if (target_id == -1)
  {
    cost_2 = 0;
  }
  else
  {
    bool target_found = false;
    double target_x;
    double target_y;
    double target_dx;
    double target_dy;
    double target_s;
    double target_d;
    double target_speed;
    
    for (int i = 0; i < sensor_fusion.size(); ++i)
    {
      if (target_id == sensor_fusion[i][0])
      {
        /* loop through to locate our target */
        target_x = sensor_fusion[i][1];
        target_y = sensor_fusion[i][2];
        target_dx = sensor_fusion[i][3];
        target_dy = sensor_fusion[i][4];
        target_s = sensor_fusion[i][5];
        target_d = sensor_fusion[i][6];
        target_speed = sqrt(target_dx*target_dx+target_dy*target_dy);
        target_found = true;
      }
    }
    
    if (!target_found)
    {
      /* something went wrong shouldve found target */
      cost_2 = 0;
    }
    else
    {
      clearance = (my_s + my_speed * 2) - (target_s + target_speed * 2); 
      if (clearance < 20.0)
      {
        cost_2 = 1;
      }
      else
      {
        cost_2 = (40.0 - clearance)/20;
        cost_2 = std::max(cost_2,0.0);
      }
    }
  }
  
  double cost = std::max(cost_1,cost_2);
  
  return cost;
}

double calculate_trouble_cost (string state, double time) {
  /**
    * calculate cost vehicle too close to others
    */
  double cost;
  
  if (state.compare("KL") == 0)
  {
    cost = 0;
  }
  else
  {
    if (time < 5)
    {
      cost = 1;
    }
    else
    {
      cost = (15 - time) / 10;
      cost = std::max(cost,0.0);    
    }
  }
  /*
  if (state.compare("LCL") == 0)
  {
    cost += 0.01;
  }
  else if (state.compare("LCR") == 0)
  {
    cost += 0.05;
  }*/
   
  cost = std::max(cost,0.0);   
  cost = std::min(cost,1.0);  
  return cost;
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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
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

          car_speed = Mph2Mps(car_speed); // main operates in m/s 
          
          
          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          /* target selection */
          
          vector<int> front_target_id = {-1,-1,-1}; /* the immediate front vehicle of each lane */ 
          vector<int> behind_target_id = {-1,-1,-1}; /* the immediate behind vehicle of each lane */
          
          std::cout<<"number target from sensor fusion" << sensor_fusion.size() << std::endl;
          
          for (int lane_id = 0; lane_id < 3; ++lane_id)
          {
            /* for each lane, fine the immediate front car id */
            double min_front_s;
            double max_behind_s;
            
            for (int i = 0; i < sensor_fusion.size(); ++i)
            {
              /**
                * Sensor Fusion Data
                * ["sensor_fusion"] A 2d vector of cars and then that car's
                * [0] car's unique ID,
                * [1] car's x position in map coordinates,
                * [2] car's y position in map coordinates,
                * [3] car's x velocity in m/s,
                * [4] car's y velocity in m/s,
                * [5] car's s position in frenet coordinates, 
                * [6] car's d position in frenet coordinates. 
                */
              int target_id = sensor_fusion[i][0];
              double s = sensor_fusion[i][5];
              s = CorrectS(s,car_s);
              float d = sensor_fusion[i][6];
              double ds = s - car_s; /* relative S of target to ego */
              
     //         if (ds > (MAX_S/2.0))
     //         {
                /* car_s has wrapped around track, car_s has not */
    //            ds -= MAX_S;
    //          }
     //         else if (ds < (-MAX_S/2.0))
     //         {
                /* s has wrapped around track, car_s has not */
     //           ds += MAX_S;
      //        }
              
              /*debug*/ /*
              if (lane_id == car_lane)
              {
                std::cout<<"target id " << target_id << std::endl;
                std::cout<<"ds " << ds << std::endl;
                std::cout<< "affecting lane?   " << IsAffectingLane(lane_id,d) << std::endl;
              }
              
              */
              
              if (IsAffectingLane(lane_id,d))
              {
                if ((ds > 0) && (ds < 100.0)) /* vehicle in front of ego not too far */
                {
                  /*debug*//*
                  if (lane_id == 1)
                  {
                    std::cout<< "I am in front of ego " << std::endl;
                    std::cout<< "current front id is" << front_target_id[lane_id] << std::endl;
                  }*/
                  if (front_target_id[lane_id] == -1) /* hasnt assigned */
                  {
                    /*debug*//*
                    if (lane_id == 1)
                    {
                      std::cout<< "first detected front " << std::endl;
                    }*/
                    front_target_id[lane_id] = target_id;
                    min_front_s = ds;
                    /*debug*//*
                    if (lane_id == 1)
                    {
                      std::cout<< "new front id is " << front_target_id[lane_id] <<std::endl;
                    }*/
                  }
                  else if (ds < min_front_s) /* closer */
                  {
                    front_target_id[lane_id] = target_id;
                    min_front_s = ds;
                  }
                }
                else if (ds > - 50.0) /* vehicle behind ego */
                {
                  /*debug*//*
                  if (lane_id == 1)
                  {
                    std::cout<< "I am behind ego " << std::endl;
                  }*/
                  if (behind_target_id[lane_id] == -1) /* hasnt assigned */
                  {
                    behind_target_id[lane_id] = target_id;
                    max_behind_s = ds;
                  }
                  else if (ds > max_behind_s) /* closer */
                  {
                    behind_target_id[lane_id] = target_id;
                    max_behind_s = ds;
                  }  
                }
              }
            }
          }

          std::cout<<"front target id: "<<front_target_id[0]<<"  "<<front_target_id[1]<<"  "<<front_target_id[2]<<std::endl;
          
          
          /* finite state machine */
          vector<string> possible_states = successor_states(car_lane);
          vector<double> costs;
          
          for (int i = 0; i < possible_states.size(); ++i)
          {
            /* for each possible state calculate cost */
            int target_lane;
            string state = possible_states[i];
            if (state.compare("KL") == 0){
              target_lane = car_lane;
            }
            else if (state.compare("LCL") == 0){
              target_lane = car_lane - 1;
            }
            else { /* LCR */
              target_lane = car_lane + 1;
            }
              
            double speed_cost = calculate_speed_cost(target_lane, car_speed, front_target_id, behind_target_id, sensor_fusion);
            double contact_cost = calculate_contact_cost(target_lane, car_speed, car_s, front_target_id, behind_target_id, sensor_fusion);
            double trouble_cost = calculate_trouble_cost(state, time_since_last_lc);
            
            double total_cost =  speed_cost * 1 + contact_cost * 10 + trouble_cost * 10;
            
            std::cout <<"Cost of "<<state<<" = "<< total_cost << std::endl;
            costs.push_back(total_cost);
          }
          
          vector<double>::iterator best_cost = min_element(begin(costs), end(costs));
          int best_idx = distance(begin(costs), best_cost);
          
          string next_state = possible_states[best_idx];
                    
          //std::cout <<"next state: "<<next_state<< std::endl; 
          
          if (next_state.compare("KL") == 0){
            /* keep car lane value */
            time_since_last_lc += 0.02;
          }
          else if (next_state.compare("LCL") == 0){
            car_lane = car_lane - 1;
            time_since_last_lc = 0;
          }
           else { /* LCR */
            car_lane = car_lane + 1;
            time_since_last_lc = 0;
          }
          
          std::cout <<"I'm in lane: "<<car_lane<< std::endl;
          std::cout <<"time since last lane change: "<<time_since_last_lc<< std::endl;
          
          /* form trajectory */
          
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          int prev_path_size = previous_path_x.size();
          int num_previous_path = std::min(prev_path_size, MAX_PREV_PATH_REUSE);
          
          // std::cout << "previous path x size: " << prev_path_size << "using: " << num_previous_path << std::endl;
          
          vector<double> anchor_x;
          vector<double> anchor_y;
          
          for (int i = 0; i < num_previous_path; ++i) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          double last_x; 
          double last_y;
          double last_angle;
          double last_s;
          double last_d;
          double last_speed;
          
          if (num_previous_path == 0) {
            last_x = car_x;
            last_y = car_y;
            last_angle = deg2rad(car_yaw);
            anchor_x.push_back(last_x);
            anchor_y.push_back(last_y);
            last_speed = car_speed;
          }
          else {
            last_x = previous_path_x[num_previous_path-1];
            last_y = previous_path_y[num_previous_path-1];

            double second_last_x; 
            double second_last_y;
            
            if (num_previous_path > 1) {
              second_last_x = previous_path_x[num_previous_path-2];
              second_last_y = previous_path_y[num_previous_path-2];
            }
            else { /* in case only 1 left over */
              second_last_x = car_x;
              second_last_y = car_y;
            }
                       
            last_angle = atan2(last_y-second_last_y,last_x-second_last_x);
            
            anchor_x.push_back(second_last_x);
            anchor_y.push_back(second_last_y);
            
            anchor_x.push_back(last_x);
            anchor_y.push_back(last_y);
            
            last_speed = distance(last_x,last_y,second_last_x,second_last_y);
            last_speed = PerLoop2PerSec(last_speed);
            
          }
          
          vector<double> last_sd = getFrenet(last_x, last_y, last_angle, map_waypoints_x, map_waypoints_y);
          last_s = last_sd[0];
          last_d = last_sd[1];
          
          /**
           * continue to build future path
           */
          double target_speed;
          double ttc = 99;
          double ds = 100;
          int target_id = front_target_id[car_lane];
          
          std::cout <<"target in my lane id" << target_id << std::endl;
  
          if (target_id == -1)
          {
            /* no front target */
          }
          else
          {
            for (int i = 0; i < sensor_fusion.size(); ++i)
            {
              if (target_id == sensor_fusion[i][0])
              {
                /* loop through to locate our target */
                double target_x = sensor_fusion[i][1];
                double target_y = sensor_fusion[i][2];
                double target_dx = sensor_fusion[i][3];
                double target_dy = sensor_fusion[i][4];
                double target_s = sensor_fusion[i][5];
                target_s = CorrectS(target_s,car_s);
                double target_d = sensor_fusion[i][6];
                
                double target_speed = sqrt(target_dx*target_dx+target_dy*target_dy);
                
                if (  (car_speed > target_speed)
                    &&(target_s > car_s)
                   )
                {
                  /* if target is in front of us and slower */
                  ttc = (target_s - car_s) / (car_speed - target_speed);
                }
                
                if (target_s > car_s)
                {
                  ds = target_s - car_s;
                }
                
                std::cout <<"target speed " << target_speed << std::endl;
                std::cout <<"my speed " << car_speed << std::endl;
                std::cout <<"ds " << ds << std::endl;
              }
            }
          }

          double desired_speed;
          
          if (  (ttc > TTC_TOO_BIG)
              &&(ds > 26)
              )
          {
            desired_speed = last_speed + PerSec2PerLoop(10.0);
            desired_speed = std::min(desired_speed,Mph2Mps(SPEED_LIMIT));
          }
          else if (  (ttc < TTC_WAY_TOO_SMALL)
                   ||(ds < 15)
                   )
          {
            desired_speed = last_speed - PerSec2PerLoop(10.0);
          }
          else if (  (ttc < TTC_TOO_SMALL)
                   ||(ds < 25)
                   )
          {
            desired_speed = last_speed - PerSec2PerLoop(5.0);
          }
          else
          {
            /* maintain speed */
          }
          
          std::cout << "ttc: " << ttc << std::endl;
          
          
          vector<double> next_wp0 = getXY(last_s+30, (2+4*car_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(last_s+60, (2+4*car_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(last_s+90, (2+4*car_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          anchor_x.push_back(next_wp0[0]);
          anchor_x.push_back(next_wp1[0]);
          anchor_x.push_back(next_wp2[0]);
          
          anchor_y.push_back(next_wp0[1]);
          anchor_y.push_back(next_wp1[1]);
          anchor_y.push_back(next_wp2[1]);
          
          for (int i = 0; i < anchor_x.size(); ++i) {
            // shift to car coordinates
            double shift_x = anchor_x[i] - last_x;
            double shift_y = anchor_y[i] - last_y;
            
            anchor_x[i] = shift_x * cos(0-last_angle) - shift_y * sin(0-last_angle);
            anchor_y[i] = shift_x * sin(0-last_angle) + shift_y * cos(0-last_angle);
          }
     
          tk::spline s;
          
          s.set_points(anchor_x,anchor_y);
          
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = distance(target_x,target_y,0,0);
          
          double x_add_on = 0;
          
          double dist_per_loop = PerSec2PerLoop(desired_speed);
                    
          for (int i = 0; i < 50-num_previous_path; ++i) {
            double N = (target_dist/dist_per_loop);
            double x_point = x_add_on + (target_x/N);
            double y_point = s(x_point);
            
            x_add_on = x_point;
            
            double x_ref = x_point;
            double y_ref = y_point;
            
            // shift back to global coordinate
            x_point = x_ref * cos(last_angle) - y_ref * sin(last_angle);
            y_point = x_ref * sin(last_angle) + y_ref * cos(last_angle);
            
            x_point += last_x;
            y_point += last_y;
            
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }
          
/*
          
          for (int i = 0; i < 50-num_previous_path; ++i) {
            double next_s = last_s + dist_per_loop * (i+1);
            double next_d = 6;
            
            vector<double> xy = getXY(next_s, next_d,
                                      map_waypoints_s, map_waypoints_x, map_waypoints_y);
            
            next_x_vals.push_back(xy[0]);
            next_y_vals.push_back(xy[1]);
          }
*/
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

