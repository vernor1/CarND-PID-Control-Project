#include <iostream>
#include <math.h>
#include <uWS/uWS.h>
#include "json.hpp"
#include "PidController.h"

// Local Constants
// -----------------------------------------------------------------------------
const auto kKp = 2.093;
const auto kKi = 0.025;
const auto kKd = 11.048;

// for convenience
using json = nlohmann::json;
using namespace std::placeholders;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string HasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
/*
  if (found_null != std::string::npos) {
    return "";
  } else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
*/
  return found_null == std::string::npos
    && b1 != std::string::npos
    && b2 != std::string::npos ? s.substr(b1, b2 - b1 + 1) : std::string();
}

// Local Helper-Functions
// -----------------------------------------------------------------------------

// Checks arguments of the program and exits if the check fails.
// @param[in] argc  Number of arguments
// @param[in] argv  Array of arguments
std::shared_ptr<PidController> CreatePidController(int argc, char* argv[]) {

  std::shared_ptr<PidController> pid_controller;

  std::stringstream oss;
    oss << "Usage instructions: " << argv[0]
        << " [Kp Ki Kd] [dKp dKi dKd trackLength offTrackCte]" << std::endl
        << "  Kp          Proportional coefficient" << std::endl
        << "  Ki          Integral coefficient" << std::endl
        << "  Kd          Differential coefficient" << std::endl
        << "  dKp         Delta of Kp" << std::endl
        << "  dKi         Delta of Ki" << std::endl
        << "  dKd         Delta of Kd" << std::endl
        << "  trackLength Approximate track length in meters" << std::endl
        << "  offTrackCte Approximate CTE when getting off track" << std::endl
        << "If no arguments provided, the default values are used: Kp="
        << kKp << ", Ki=" << kKi << ", Kd=" << kKd << "." << std::endl
        << "If only [Kp Ki Kd] are provided, the PID controller uses"
        << "those values." << std::endl
        << "If [dKp dKi dKd trackLength offTrackCte] are also provided, the PID"
        << " controller finds best coefficients using the Twiddle algorithm,"
        << " and uses them." << std::endl;

  if (argc != 1 && argc != 4 && argc != 9) {
    std::cerr << oss.str();
    exit(EXIT_FAILURE);
  }

  try {
    switch (argc) {
      case 1:
        pid_controller.reset(new PidController(kKp, kKi, kKd));
        break;
      case 4: {
        // TODO: use auto
        double kp = std::stod(argv[1]);
        double ki = std::stod(argv[2]);
        double kd = std::stod(argv[3]);
        pid_controller.reset(new PidController(kp, ki, kd));
        break;
      }
      case 9: {
        double kp = std::stod(argv[1]);
        double ki = std::stod(argv[2]);
        double kd = std::stod(argv[3]);
        double dkp = std::stod(argv[4]);
        double dki = std::stod(argv[5]);
        double dkd = std::stod(argv[6]);
        double track_length = std::stod(argv[7]);
        double off_track_cte = std::stod(argv[8]);
        pid_controller.reset(new PidController(kp, ki, kd, dkp, dki, dkd, track_length, off_track_cte));
        break;
      }
      default:
        std::cerr << "Invalid number of arguments." << std::endl << oss.str();
        exit(EXIT_FAILURE);
    }
  }
  catch (const std::exception& e) {
    std::cerr << "Invalid data format: " << e.what() << std::endl << oss.str();
    exit(EXIT_FAILURE);
  }

  return pid_controller;
}

void ControlSimulator(uWS::WebSocket<uWS::SERVER>& ws,
                      double steering,
                      double throttle) {
//  std::cout << "steering_value " << steering_value << std::endl;
  json json_msg;
  json_msg["steering_angle"] = steering;
  json_msg["throttle"] = throttle;
  auto msg = "42[\"steer\"," + json_msg.dump() + "]";
//  std::cout << msg << std::endl;
  // TODO: consider replacing data() with c_str()
  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}

void ResetSimulator(uWS::WebSocket<uWS::SERVER>& ws) {
  std::string msg("42[\"reset\", {}]");
  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}

int main(int argc, char* argv[])
{
  uWS::Hub h;

  std::shared_ptr<PidController> pid_controller = CreatePidController(argc, argv);

//  PidController pid_controller(kKp, kKi, kKd);
//  PidController pid_controller;

  h.onMessage([pid_controller](uWS::WebSocket<uWS::SERVER> ws,
                               char* data,
                               size_t length,
                               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = HasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
//          std::cout << "cte " << cte << ", speed " << speed << std::endl;

          pid_controller->Update(
            cte,
            speed,
            std::bind(ControlSimulator, ws, _1, _2),
            std::bind(ResetSimulator, ws));
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          * ??? K*(60-speed) - kp*cte - kd*dcte - ki*Icte
          */
          
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
//    std::cout << "Connected!!!" << std::endl;
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
