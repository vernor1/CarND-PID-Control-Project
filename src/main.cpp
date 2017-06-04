#include <cmath>
#include <iostream>
#include <uWS/uWS.h>
#include "json.hpp"
#include "PidController.h"

using namespace std::placeholders;

// Local Constants
// -----------------------------------------------------------------------------

// TCP port accepting incoming connections from simulator
enum { kTcpPort = 4567 };

// Default PID coefficients
const auto kKp = 0.12;
const auto kKi = 1e-5;
const auto kKd = 4.0;

// Local Helper-Functions
// -----------------------------------------------------------------------------

// Checks if the SocketIO event has JSON data.
// @param[in] s  Raw event string
// @return       If there is data the JSON object in string format will be
//               returned, else the empty string will be returned.
std::string HasData(const std::string& s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  return found_null == std::string::npos
    && b1 != std::string::npos
    && b2 != std::string::npos ? s.substr(b1, b2 - b1 + 1) : std::string();
}

// Checks arguments of the program and exits, if the check fails.
// @param[in] argc  Number of arguments
// @param[in] argv  Array of arguments
// @return          A smart pointer to the PID controller object
std::shared_ptr<PidController> CreatePidController(int argc, char* argv[]) {
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
        << " those values." << std::endl
        << "If [dKp dKi dKd trackLength offTrackCte] are also provided, the PID"
        << " controller finds best coefficients using the Twiddle algorithm,"
        << " and uses them." << std::endl;

  if (argc != 1 && argc != 4 && argc != 9) {
    std::cerr << oss.str();
    std::exit(EXIT_FAILURE);
  }

  std::shared_ptr<PidController> pid_controller;
  try {
    switch (argc) {
      case 1:
        pid_controller.reset(new PidController(kKp, kKi, kKd));
        break;
      case 4: {
        auto kp = std::stod(argv[1]);
        auto ki = std::stod(argv[2]);
        auto kd = std::stod(argv[3]);
        pid_controller.reset(new PidController(kp, ki, kd));
        break;
      }
      case 9: {
        auto kp = std::stod(argv[1]);
        auto ki = std::stod(argv[2]);
        auto kd = std::stod(argv[3]);
        auto dkp = std::stod(argv[4]);
        auto dki = std::stod(argv[5]);
        auto dkd = std::stod(argv[6]);
        auto track_length = std::stod(argv[7]);
        auto off_track_cte = std::stod(argv[8]);
        pid_controller.reset(new PidController(kp, ki, kd, dkp, dki, dkd,
                                               track_length, off_track_cte));
        break;
      }
      default:
        std::cerr << "Invalid number of arguments." << std::endl << oss.str();
        std::exit(EXIT_FAILURE);
    }
  }
  catch (const std::exception& e) {
    std::cerr << "Invalid data format: " << e.what() << std::endl << oss.str();
    std::exit(EXIT_FAILURE);
  }

  return pid_controller;
}

// Sends a control message to the simulator.
// @param[in] ws        WebSocket object
// @param[in] steering  Steering value
// @param[in] throttle  Throttle value
void ControlSimulator(uWS::WebSocket<uWS::SERVER>& ws,
                      double steering,
                      double throttle) {
  nlohmann::json json_msg;
  json_msg["steering_angle"] = steering;
  json_msg["throttle"] = throttle;
  auto msg = "42[\"steer\"," + json_msg.dump() + "]";
  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}

// Sends a reset message to the simulator.
// @param[in] ws        WebSocket object
void ResetSimulator(uWS::WebSocket<uWS::SERVER>& ws) {
  std::string msg("42[\"reset\", {}]");
  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}

// main
// -----------------------------------------------------------------------------

int main(int argc, char* argv[])
{
  uWS::Hub hub;
  auto pid_controller = CreatePidController(argc, argv);
  hub.onMessage([pid_controller](uWS::WebSocket<uWS::SERVER> ws,
                                 char* data,
                                 size_t length,
                                 uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = HasData(std::string(data).substr(0, length));
      if (!s.empty()) {
        auto j = nlohmann::json::parse(s);
        auto event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          auto cte = std::stod(j[1]["cte"].get<std::string>());
          auto speed = std::stod(j[1]["speed"].get<std::string>());
          auto angle = std::stod(j[1]["steering_angle"].get<std::string>());
          pid_controller->Update(
            cte,
            speed,
            std::bind(ControlSimulator, ws, _1, _2),
            std::bind(ResetSimulator, ws));
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  hub.onConnection([](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "WebSocket connected" << std::endl;
  });

  hub.onDisconnection([](uWS::WebSocket<uWS::SERVER> ws,
                         int code,
                         char* message,
                         size_t length) {
    ws.close();
    std::cout << "WebSocket disconnected" << std::endl;
  });

  if (hub.listen(kTcpPort)) {
    std::cout << "Listening to port " << kTcpPort << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  hub.run();
}
