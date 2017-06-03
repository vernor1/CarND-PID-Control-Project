#include <cassert>
#include <cmath>
#include <iostream>
#include "PidController.h"

namespace {

// Local Constants
// -----------------------------------------------------------------------------

const auto kMetersInMile = 1609.344;
const auto kFrameRate = 25.;
const auto kSecondsPerFrame = 1. / kFrameRate;
const auto kMphToMps = kMetersInMile / (60. * 60.);
const auto kSpeedToDistanceCoeff = kMphToMps / kFrameRate;
const auto kOffTrackPenalty = 1e+6;
const auto kMaxCteSkipPart = 0.025;
//const auto kKp = 0.1;
//const auto kKi = 1e-4;
//const auto kKd = 4;
// Max CTE is 3.8669 at distance 4000.37! Trying PID parameters 0.071, 0.00031, 4
// Max CTE is 3.2262 at distance 4000.7!  Trying PID parameters 0.0831, 0.000431, 4
// Max CTE is 2.4096 at distance 4000.75! Trying PID parameters 0.127156, 0.000670459, 4
//const Twiddler::ParameterSequence kInitialPidParameters
//  = {{.p=kKp, .dp=0.01}, {.p=kKi, 1e-5}, {.p=kKd, .dp=0.1}};

// Local Helper-Functions
// -----------------------------------------------------------------------------

double NormalizeControl(double value) {
  if (value > 1.) {
    return 1.;
  }
  if (value < -1.) {
    return -1.;
  }
  return value;
}

} // namespace

// Public Members
// -----------------------------------------------------------------------------

PidController::PidController(double kp, double ki, double kd,
                             double dkp, double dki, double dkd,
                             double track_length, double off_track_cte)
  : has_final_coefficients_(false),
    track_length_(track_length),
    off_track_cte_(off_track_cte),
    distance_(0),
    time_(0),
    max_cte_(0),
    pid_(new Pid(kp, ki, kd)),
    twiddler_(
      new Twiddler({{.p=kp, .dp=dkp}, {.p=ki, .dp=dki}, {.p=kd, .dp=dkd}})) {
  std::cout << "Creating PID controller with initial coefficients Kp=" << kp
            << ", Ki=" << ki << ", Kd=" << kd << ", dKp=" << dkp << ", dKi="
            << dki << ", dKd=" << dkd << std::endl;
}

PidController::PidController(double kp, double ki, double kd)
  : has_final_coefficients_(true),
    distance_(0),
    time_(0),
    max_cte_(0),
    pid_(new Pid(kp, ki, kd)) {
  std::cout << "Creating PID controller with final coefficients Kp="
            << kp << ", Ki=" << ki << ", Kd=" << kd << std::endl;
}

void PidController::Update(
  double cte,
  double speed,
  std::function<void(double steering, double throttle)> on_control,
  std::function<void()> on_reset) {

//  std::cout << "has_final_coefficients_ " << has_final_coefficients_ << std::endl;
  if (!has_final_coefficients_) {
    distance_ += kSpeedToDistanceCoeff * speed;
    time_ += kSecondsPerFrame;
    if (cte > max_cte_ && distance_ > track_length_ * kMaxCteSkipPart) {
      std::cout << "New max CTE " << max_cte_ << std::endl;
      max_cte_ = cte;
    }
//    std::cout << "Speed " << speed << ", distance " << distance_ << ", max CTE " << max_cte_ << std::endl;

    if (distance_ > 5 && (std::fabs(cte) > off_track_cte_ || speed < 1)) {
      // Getting off track
      auto error = kOffTrackPenalty / distance_;
      auto parameters = twiddler_->UpdateError(error);
      assert(parameters.size() == 3);
      auto kp = parameters[0].p;
      auto ki = parameters[1].p;
      auto kd = parameters[2].p;
      std::cout << "Getting off track at distance " << distance_ << ", speed "
                << speed << "! (error value " << error
                << ") Trying PID coefficients " << kp << ", " << ki << ", "
                << kd << std::endl;
      pid_.reset(new Pid(kp, ki, kd));
      distance_ = 0;
      time_ = 0;
      max_cte_ = 0;
      on_reset();
      return;
    }

    if (distance_ > track_length_) {
      if (max_cte_ < off_track_cte_ / 2.) {
        auto average_speed = distance_ / (time_ * kMphToMps);
        std::cout << "Max CTE is " << max_cte_ << " at distance " << distance_
                  << "m, time " << time_ << "s, average speed " << average_speed
                  << "mph. Using the final coefficients." << std::endl;
        has_final_coefficients_ = true;
      }
      else {
        auto parameters = twiddler_->UpdateError(max_cte_);
        assert(parameters.size() == 3);
        auto kp = parameters[0].p;
        auto ki = parameters[1].p;
        auto kd = parameters[2].p;
        auto average_speed = distance_ / (time_ * kMphToMps);
        std::cout << "Max CTE is " << max_cte_ << " at distance " << distance_
                  << "m, time " << time_ << "s, average speed " << average_speed
                  << "mph. Trying PID coefficients "
                  << kp << ", " << ki << ", " << kd << std::endl;
        pid_.reset(new Pid(kp, ki, kd));
        distance_ = 0;
        time_ = 0;
        max_cte_ = 0;
        on_reset();
        return;
      }
    }
  }

  auto steering = NormalizeControl(pid_->GetError(cte));
  auto throttle = 1.0;
  if (speed > 60) {
    throttle = NormalizeControl(1.0 - 4.0 * std::fabs(cte) / off_track_cte_);
  }
//  std::cout << "Steering " << steering << ", throttle " << throttle << std::endl;
  on_control(steering, throttle);
}

// Private Members
// -----------------------------------------------------------------------------
