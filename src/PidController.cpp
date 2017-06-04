#include "PidController.h"
#include <cassert>
#include <cmath>
#include <iostream>

namespace {

// Local Constants
// -----------------------------------------------------------------------------

// Target CTE margin w.r.t. the off track CTE
const auto kTargetCteMargin = 0.25;

// Initial part of track where off track detection is not applied
const auto kSkipOffTrackPart = 0.00125;

// Initial part of track where max CTE updates are skipped
const auto kSkipMaxCtePart = 0.025;

// Twiddler error penalty when going off track
const auto kOffTrackPenalty = 1e+6;

// Meters in mile per international agreement of 1959
const auto kMetersInMile = 1609.344;

// Default framerate
const auto kFrameRate = 25.;

// Time passed between frames
const auto kSecondsPerFrame = 1. / kFrameRate;

// Coefficient of conversion miles-per-hour to meters-per-second
const auto kMphToMps = kMetersInMile / (60. * 60.);

// Coefficient for computing distance passed since a previous frame, given the
// instant speed
const auto kSpeedToDistanceCoeff = kMphToMps / kFrameRate;

// Local Helper-Functions
// -----------------------------------------------------------------------------

// Normalizes input within -1..+1.
// @param[in] value  Value to normalize
// @return           Normalized value
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
    distance_(),
    time_(),
    max_cte_(),
    pid_(new Pid(kp, ki, kd)),
    twiddler_(
      new Twiddler({{.p=kp, .dp=dkp}, {.p=ki, .dp=dki}, {.p=kd, .dp=dkd}})) {
  std::cout << "Creating PID controller with initial coefficients Kp=" << kp
            << ", Ki=" << ki << ", Kd=" << kd << ", dKp=" << dkp << ", dKi="
            << dki << ", dKd=" << dkd << std::endl;
}

PidController::PidController(double kp, double ki, double kd)
  : has_final_coefficients_(true),
    track_length_(),
    off_track_cte_(),
    distance_(),
    time_(),
    max_cte_(),
    pid_(new Pid(kp, ki, kd)) {
  std::cout << "Creating PID controller with final coefficients Kp="
            << kp << ", Ki=" << ki << ", Kd=" << kd << std::endl;
}

void PidController::Update(
  double cte,
  double speed,
  std::function<void(double steering, double throttle)> on_control,
  std::function<void()> on_reset) {

  if (!has_final_coefficients_) {
    distance_ += kSpeedToDistanceCoeff * speed;
    time_ += kSecondsPerFrame;
    if (cte > max_cte_ && distance_ > kSkipMaxCtePart * track_length_) {
      std::cout << "New max CTE " << max_cte_ << std::endl;
      max_cte_ = cte;
    }

    // Detect getting off track
    if (distance_ > kSkipOffTrackPart * track_length_
        && (std::fabs(cte) > off_track_cte_ || speed < 1.0)) {
      auto error = kOffTrackPenalty / distance_;
      std::cout << "Getting off track at distance " << distance_ << ", speed "
                << speed << "! (error value " << error << ") ";
      UpdateTwiddlerAndReset(error);
      on_reset();
      return;
    }

    // Detect completing the track
    if (distance_ > track_length_) {
      auto average_speed = distance_ / (time_ * kMphToMps);
      std::cout << "Max CTE is " << max_cte_ << " at distance " << distance_
                << "m, time " << time_ << "s, average speed " << average_speed
                << "mph. ";
      if (max_cte_ < off_track_cte_ / 2.) {
        std::cout << "Using the final coefficients." << std::endl;
        has_final_coefficients_ = true;
      } else {
        UpdateTwiddlerAndReset(max_cte_);
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
  on_control(steering, throttle);
}

// Private Members
// -----------------------------------------------------------------------------
void PidController::UpdateTwiddlerAndReset(double error) {
  auto parameters = twiddler_->UpdateError(error);
  assert(parameters.size() == 3);
  auto kp = parameters[0].p;
  auto ki = parameters[1].p;
  auto kd = parameters[2].p;
  std::cout << "Trying PID coefficients " << kp << ", " << ki << ", " << kd
            << std::endl;
  pid_.reset(new Pid(kp, ki, kd));
  distance_ = 0;
  time_ = 0;
  max_cte_ = 0;
}
