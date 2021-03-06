#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <limits>
#include <iomanip>
#include <string>
#include <vector>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;

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
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  // Twiddling 
  bool enable_twiddle = false;
  int step_count = 0;
  double best_error = 999;
  std::vector<double> twiddle_dp{0, 0, 0.5};
  bool twiddle_direction = true;
  
  // Create two PID controllers
  PID pid_steer;
  PID pid_throttle;

  // Initialise
  std::vector<double> steer_gains{0.216, 0.002, 1.70};
  std::vector<double> throttle_gains{0.1, 0.00001, 0};
  
  pid_steer.Init(steer_gains[0], steer_gains[1], steer_gains[2]);
  pid_throttle.Init(throttle_gains[0], throttle_gains[1], throttle_gains[2]);

  h.onMessage([&pid_steer, &pid_throttle, &enable_twiddle, &step_count, &best_error, &steer_gains, &throttle_gains, &twiddle_dp, &twiddle_direction](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          //double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          double throttle_value;
          double target_speed = 35;
    
          // Increment run counter.
          step_count++;

          // GAIN TWIDDLING ALGORITHM
          if (enable_twiddle)
          {
            // 2000 steps per run - ignore first 250 as we get up to speed.
            if ((((step_count - 250) % 2000) == 0) && (step_count > 250))
            {

              // Get RMSE
              double RMSE = pid_steer.TotalError();

              // Debug Output
              std::cout << std::setprecision(5) << std::fixed;
              std::cout << "dp: " << twiddle_dp[0] << "\tP_gain: " << steer_gains[0] << "\tError: " << RMSE << "\tBest: " << best_error << "\n";
              std::cout << "dp: " << twiddle_dp[1] << "\tI_gain: " << steer_gains[1] << "\tError: " << RMSE << "\tBest: " << best_error << "\n";
              std::cout << "dp: " << twiddle_dp[2] << "\tD_gain: " << steer_gains[2] << "\tError: " << RMSE << "\tBest: " << best_error << "\n";

              //Update gains....
              if (RMSE < best_error)
              {
                best_error = RMSE;
                //Update gains...
                // std::plus adds together its two arguments:
                for (size_t i = 0; i < steer_gains.size(); i++) steer_gains[i] += twiddle_dp[i];
                
                // Update Twiddle_DP
                for (size_t i = 0; i < twiddle_dp.size(); i++)  twiddle_dp[i] *= 1.1;
              }
              else if (twiddle_direction)
              {
                twiddle_direction = false;
                for (size_t i = 0; i < steer_gains.size(); i++) steer_gains[i] -= 2*twiddle_dp[i];
              } 
              else
              {
                // Reset
                twiddle_direction = true;
                // Reset Twiddle DP and add 0.9 * dp
                for (size_t i = 0; i < steer_gains.size(); i++) steer_gains[i] += 1.9*twiddle_dp[i];
                // Reduce DP and for next round
                for (size_t i = 0; i < twiddle_dp.size(); i++)  twiddle_dp[i] *= 0.9;
              }

              // Reinitialise and continue (this also resets RMSE and integrator)
              pid_steer.Init(steer_gains[0], steer_gains[1], steer_gains[2]);
            }
          }

          // Update PID FF (only for Throttle!)
          // Lazy FF - simple quadratic with drag but then assumes throttle is linear.
          pid_throttle.SetFF(target_speed*target_speed * 1/4000);

          // Update PID errors
          pid_steer.UpdateError(cte);
          pid_throttle.UpdateError(target_speed - speed);
          
          // Get new PID outputs
          steer_value = pid_steer.ControlDemand();
          throttle_value = pid_throttle.ControlDemand();

          /**
           * Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
          
          // Saturate....
          if (steer_value > 1) steer_value = 1;
          if (steer_value < -1) steer_value = -1;
          if (throttle_value > 1) throttle_value = 1;
          if (throttle_value < 0) throttle_value = 0;

          // Fix for now
          //throttle_value = 0.3;
          // DEBUG
          //std::cout << std::setprecision(3) << std::fixed;
          //std::cout << "CTE: " << cte << "\tSteering Value: " << steer_value << "\t"
          //          << "SE:  " << target_speed-speed << "\tthrottle: " << throttle_value 
          //          << std::endl;
                    

          json msgJson;
          msgJson["steering_angle"] = -steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
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