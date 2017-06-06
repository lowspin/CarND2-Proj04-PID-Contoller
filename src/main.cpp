#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

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

int main()
{
  uWS::Hub h;

  PID pid;
  // TODO: Initialize the pid variable.
  pid.Init(0.2, 0.004, 3.0, true);
  //pid.Init(0.0, 0.0, 0.0, true); // to-be adjusted by Twiddle
  pid.Params[0] = pid.Kp; //0.0; // Kp
  pid.Params[1] = pid.Ki; //0.0; // Ki
  pid.Params[2] = pid.Kd; //0.0; // Kd
  pid.dP[0] = 0.1; //0.5;// pid.Kp; //1.0; //1.0;  // dKp
  pid.dP[1] = 0.1; //0.01; // pid.Ki; //1.0; //0.01; // dKi
  pid.dP[2] = 0.1; //pid.Kd; //1.0; //10.0; // dKd
  pid.stage2 = false;
  pid.tuning_param = 2; //0;
  pid.best_avgcte = 99999.99;
  pid.status = 0;

  // first parameter for Twiddle
  pid.Params[pid.tuning_param] += pid.dP[pid.tuning_param];
  std::cout << "---------- Twiddle Starts -------------" << std::endl;

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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

          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

          if(pid.still_twiddling){ // Twiddle
//            std::cout << "\nTwiddle frame#" << pid.framecount << " tuning " << pid.tuning_param << "-" << pid.stage2 << " avg_cte = " << pid.avg_cte << " bestavgcte = " << pid.best_avgcte << std::endl;
            //if((pid.dP[0]<0.002)&&(pid.dP[1]<0.00004)&&(pid.dP[2]<0.03)) { // tolerance met - exit
            if(pid.dP[0]+pid.dP[1]+pid.dP[2]<0.2) {
            //if((pid.dP[0]<0.1)) {
              //pid.Init(0.2, 0.004, 3.0, true);
              //pid.Init(pid.Kp, pid.Ki, pid.Kd, false);
              pid.Init(pid.Params[0], pid.Params[1], pid.Params[2], false);
              std::cout << " ------- Twiddle Complete -------- " << std::endl;
              std::cout << "Final Params: Kp=" << pid.Params[0] << " Ki=" << pid.Params[1] << " Kd=" << pid.Params[2] << std::endl;
              std::cout << " --------------------------------- " << std::endl;

              // reset
              std::cout << "RESET" << std::endl;
              std::string msg = "42[\"reset\",{}]";
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
//            else if (pid.framecount==0)
//            {
//              pid.Params[pid.tuning_param] += pid.dP[pid.tuning_param];
//            }
//            else if ((pid.framecount>500) ||  // Tune Twiddle eval_frames = 200;
//                     (pid.avg_cte>pid.best_avgcte) ||
//                     (cte>3.0) ||
//                     ((speed<0.001)&&(pid.framecount>20)) //stuck
//                     )
            else if (pid.framecount>500)
            {
              if (pid.avg_cte<pid.best_avgcte)
              {
                pid.best_avgcte = pid.avg_cte;
                pid.dP[pid.tuning_param] *= 1.1;
                std::cout << "++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
                std::cout << "Exit 1 Frame " << pid.framecount << " bestcte=" << pid.best_avgcte << " avgcte=" << pid.avg_cte << " P" << pid.tuning_param << ": " << pid.Params[0] << " " << pid.Params[1] << " " << pid.Params[2] << " dP: " << pid.dP[0] << " " << pid.dP[1] << " " << pid.dP[2] << " ";
                pid.Init(pid.Params[0], pid.Params[1], pid.Params[2], true);
                //pid.Init(pid.Kp, pid.Ki, pid.Kd, true);
                pid.stage2 = false;
                pid.TuneNext();
                pid.Params[pid.tuning_param] += pid.dP[pid.tuning_param];

                // reset
                pid.status = 1;
                //std::cout << "RESET" << std::endl;
                std::string msg = "42[\"reset\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
              }
              else
              {
                if (!pid.stage2)
                {
                  pid.Params[pid.tuning_param] -= 2.0*pid.dP[pid.tuning_param];
                  pid.Params[pid.tuning_param] = (pid.Params[pid.tuning_param]<0.0)?0.0:pid.Params[pid.tuning_param];
                  std::cout << "--------------------------------------------" << std::endl;
                  std::cout << "Exit 2 Frame " << pid.framecount << " bestcte=" << pid.best_avgcte << " avgcte=" << pid.avg_cte << " P" << pid.tuning_param << ": " << pid.Params[0] << " " << pid.Params[1] << " " << pid.Params[2] << " dP: " << pid.dP[0] << " " << pid.dP[1] << " " << pid.dP[2] << " ";
                  pid.Init(pid.Params[0], pid.Params[1], pid.Params[2], true);
                  //pid.Init(pid.Kp, pid.Ki, pid.Kd, true);
                  pid.stage2 = true;
                  // same paramter

                  // reset
                  pid.status = 2;
                  //std::cout << "RESET" << std::endl;
                  std::string msg = "42[\"reset\",{}]";
                  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }
                else //stage2 for this parameter
                {
                  if (pid.avg_cte<pid.best_avgcte) // never used: will be caught in exit 1
                  {
                    pid.best_avgcte = pid.avg_cte;
                    pid.dP[pid.tuning_param] *= 1.1;
                  }
                  else
                  {
                    pid.Params[pid.tuning_param] += pid.dP[pid.tuning_param];
                    pid.dP[pid.tuning_param] *= 0.9;
                  }
                  std::cout << "========================" << std::endl;
                  std::cout << "Exit 3 Frame " << pid.framecount<< " bestcte=" << pid.best_avgcte << " avgcte=" << pid.avg_cte  << " P" << pid.tuning_param << ": " << pid.Params[0] << " " << pid.Params[1] << " " << pid.Params[2] << " dP: " << pid.dP[0] << " " << pid.dP[1] << " " << pid.dP[2] << " ";
                  pid.Init(pid.Params[0], pid.Params[1], pid.Params[2], true);
                  //pid.Init(pid.Kp, pid.Ki, pid.Kd, true);
                  pid.stage2 = false;
                  pid.TuneNext();
                  //pid.tuning_param = (pid.tuning_param+1)%3;  // 0, 1, 2, 0, 1, 2, 0 ....

                  // reset
                  pid.status = 3;
                  //std::cout << "RESET" << std::endl;
                  std::string msg = "42[\"reset\",{}]";
                  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }
              }
            }

            else
            {
              // Twiddle evaluate
              if (pid.framecount==0) // just after reset
              {
                //pid.Params[pid.tuning_param] += pid.dP[pid.tuning_param];
              }
              if (pid.framecount<10) // just after reset
              {
                pid.UpdateError(cte); // also update avg_cte
                steer_value = 0.0;
              }
              else
              {
                pid.UpdateError(cte); // also update avg_cte
                steer_value = pid.TotalError();
                steer_value = (steer_value>1.0)?1.0:steer_value;
                steer_value = (steer_value<-1.0)?-1.0:steer_value;
              }

              // DEBUG
//              std::cout << "Twiddle Kp=" << pid.Kp << " Ki=" << pid.Ki << " Kd=" << pid.Kd << std::endl;
//              std::cout << "Twiddle dp=" << pid.dP[0] << " di=" << pid.dP[1] << " dd=" << pid.dP[2] << std::endl;
//              std::cout << "Twiddle " << pid.status << " bestCTE: " << pid.best_avgcte << " avgCTE: " << pid.avg_cte << " currCTE: " << cte << std::endl;
//              std::cout << "speed: " << speed << " Steering Value: " << steer_value << std::endl;

              json msgJson;
              msgJson["steering_angle"] = steer_value;
              msgJson["throttle"] = 0.3; //1.0-abs(steer_value); //0.3;
              auto msg = "42[\"steer\"," + msgJson.dump() + "]";
//              std::cout << msg << std::endl;
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

              pid.framecount++;
            }
          }
          else // actual run
          {
            pid.UpdateError(cte);
            steer_value = pid.TotalError();
            steer_value = (steer_value>1.0)?1.0:steer_value;
            steer_value = (steer_value<-1.0)?-1.0:steer_value;

            // DEBUG
            std::cout << "Kp=" << pid.Kp << " Ki=" << pid.Ki << " Kd=" << pid.Kd << std::endl;
            std::cout << "Ep=" << pid.p_error << " Ei=" << pid.i_error<< " Ed=" << pid.d_error  << std::endl;
            std::cout << "CTE: " << cte << " speed: " << speed << " Steering Value: " << steer_value << std::endl;

            json msgJson;
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = 0.3; // 1.0-abs(steer_value); //0.3;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

            pid.framecount++;
          }
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
