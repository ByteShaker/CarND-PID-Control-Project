#include <uWS/uWS.h>
#include <iostream>
#include <sys/time.h>
#include "json.hpp"
#include "PID.h"
#include <math.h>

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
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

void reset_simulator(uWS::WebSocket<uWS::SERVER>& ws)
{
  // reset
  std::string msg("42[\"reset\", {}]");
  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}


int main()
{
  uWS::Hub h;

  std::vector<double> p(6);
  //p = {0.4979, 0.000730566, 5.63597, 0.101509, 0.00199, 7.07897};
  p = {0.3479, 0.000730566, 6.63597, 0.001509, -0.000199, 16.07897};

  std::vector<double> dp(6);
  dp  = {0.1,0.001,1.0,0.1,0.001,1.0};
  double sum_pd = 6.0;

  PID pid;
  PID pidSpeed;

  pid.Init(p[0], p[1], p[2]);
  pidSpeed.Init(p[3], p[4], p[5]);

  bool twiddle = FALSE;
  bool first_run = TRUE;
  double best_error = 0.0;
  double current_error = 0.0;
  int twiddle_counter = 0;
  int value_iter = 0;
  bool twiddle_higher = FALSE;
  bool twiddle_lower = FALSE;
  double avg_speed = 0.0;


  h.onMessage([&pid, &pidSpeed, &twiddle_counter, &value_iter, &twiddle_higher, &twiddle_lower, &best_error, &current_error, &first_run, &twiddle, &sum_pd, &p, &dp, &avg_speed](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {

      //TWIDDLE ALGORITHM:
      twiddle_counter += 1;
      if (twiddle == TRUE){
        if (twiddle_counter > 6000){
          if (first_run == TRUE){
            first_run = FALSE;
            best_error = pid.total_error/avg_speed;
            pid.Init(p[0], p[1], p[2]);
            pidSpeed.Init(p[3], p[4], p[5]);
            reset_simulator(ws);
            twiddle_counter = 0;
          }else if(sum_pd > 0.0001){
            if(twiddle_higher == FALSE){
              twiddle_higher = TRUE;
              p[value_iter] += dp[value_iter];
              pid.Init(p[0], p[1], p[2]);
              pidSpeed.Init(p[3], p[4], p[5]);
              reset_simulator(ws);
              twiddle_counter = 0;
            }else if(twiddle_lower == FALSE){
              current_error = pid.total_error/avg_speed;
              if(current_error < best_error){
                best_error = current_error;
                dp[value_iter] *= 1.1;
                value_iter = (value_iter + 1) % 6;
                twiddle_higher = FALSE;
              }else{
                twiddle_lower = TRUE;
                p[value_iter] -= 2.0 * dp[value_iter];
                pid.Init(p[0], p[1], p[2]);
                pidSpeed.Init(p[3], p[4], p[5]);
                reset_simulator(ws);
                pid.Init(p[0], p[1], p[2]);
                pidSpeed.Init(p[3], p[4], p[5]);
                reset_simulator(ws);
                twiddle_counter = 0;
              }
            }else{
              current_error = pid.total_error/avg_speed;
              if(current_error < best_error){
                best_error = current_error;
                dp[value_iter] *= 1.1;
                value_iter = (value_iter + 1) % 6;
                twiddle_higher = FALSE;
                twiddle_lower = FALSE;
              }else{
                p[value_iter] += dp[value_iter];
                dp[value_iter] *= 0.9;
                value_iter = (value_iter + 1) % 6;
                twiddle_higher = FALSE;
                twiddle_lower = FALSE;
              }
              sum_pd = 0.0;
              for(int i=0; i<6; i++) {
                sum_pd += dp[i];
              }
            }
          }else{
            pid.Init(p[0], p[1], p[2]);
            pidSpeed.Init(p[3], p[4], p[5]);
            reset_simulator(ws);
            twiddle = FALSE;
          }
          if(twiddle_counter == 0.0){
            avg_speed = 0.0;

            std::cout << "Sum PD " << sum_pd << std::endl;
            std::cout << "Best Error " << best_error << "Current "<< current_error << std::endl;
            std::cout << "Twiddled Values: " << p[0] << "/" << p[1] << "/" << p[2] << "/" << p[3] << "/" << p[4] << "/" << p[5] << std::endl;
          }
        }
      }

      //PID ALGORITHM

      // "42" at the start of the message means there's a websocket message event.
      // The 4 signifies a websocket message
      // The 2 signifies a websocket event
      if (length && length > 2 && data[0] == '4' && data[1] == '2')
      {
        auto s = hasData(std::string(data).substr(0, length));
        if (s != "") {
          auto j = json::parse(s);
          std::string event = j[0].get<std::string>();
          if (event == "telemetry") {
            // j[1] is the data JSON object
            double cte = std::stod(j[1]["cte"].get<std::string>());
            double speed = std::stod(j[1]["speed"].get<std::string>());
            double angle = std::stod(j[1]["steering_angle"].get<std::string>());

            double steer_value;
            double throttle;

            avg_speed += speed;

            pid.UpdateError(cte);
            pid.TotalError();

            steer_value = - (pid.Kp * pid.p_error) - (pid.Ki * pid.i_error) - (pid.Kd * pid.d_error);
            if (steer_value > 1.0){
              steer_value = 1.0;
            }
            if (steer_value < - 1.0){
              steer_value = - 1.0;
            }

            pidSpeed.UpdateError(cte);
            pidSpeed.TotalError();

            //throttle = 1.0 - std::abs(steer_value);
            throttle = - (pid.Kp * pid.p_error) - (pid.Ki * pid.i_error) - (pid.Kd * pid.d_error);
            throttle = (0.9 - std::abs(throttle));
            if (throttle < - 0.5){
              throttle = -0.5;
            }

            // DEBUG
            //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

            json msgJson;
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = throttle;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            //std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

          }
        } else {
          // Manual driving
          std::string msg = "42[\"manual\",{}]";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
