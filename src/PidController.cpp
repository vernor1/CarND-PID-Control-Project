#include "PidController.h"
#include <cassert>
#include <cmath>
#include <iomanip>
#include <iostream>

namespace {

// Local Constants
// -----------------------------------------------------------------------------

// Target CTE margin w.r.t. the off track CTE
const auto kTargetCteMargin = 0.65;

// Max vehicle speed in miles-per-hour
const auto kMaxSpeed = 100.0;

// Safe CTE margin w.r.t. the off track CTE when driving normally
const auto kSafeCteMargin = 0.6;

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
template<typename T>
double Normalize(T value, T lower_boundary, T upper_boundary) {
  if (value > upper_boundary) {
    return upper_boundary;
  }
  if (value < lower_boundary) {
    return lower_boundary;
  }
  return value;
}

} // namespace

// Public Members
// -----------------------------------------------------------------------------

PidController::PidController(double kp, double ki, double kd,
                             double off_track_cte,
                             double dkp, double dki, double dkd,
                             double track_length)
  : has_final_coefficients_(false),
    off_track_cte_(off_track_cte),
    track_length_(track_length),
    distance_(),
    no_max_cte_distance_(kSkipMaxCtePart * track_length),
    no_off_track_distance_(kSkipOffTrackPart * track_length),
    n_frames_(),
    safe_cte_(kSafeCteMargin * off_track_cte),
    max_cte_(),
    sum_cte_(),
    pid_(new Pid(kp, ki, kd)),
    twiddler_(
      new Twiddler({{.p=kp, .dp=dkp}, {.p=ki, .dp=dki}, {.p=kd, .dp=dkd}})) {
  std::cout << "Creating PID controller with initial coefficients Kp=" << kp
            << ", Ki=" << ki << ", Kd=" << kd << ", dKp=" << dkp << ", dKi="
            << dki << ", dKd=" << dkd << ", off-track CTE=" << off_track_cte
            << ", target CTE=" << kTargetCteMargin * off_track_cte << std::endl;
}

PidController::PidController(double kp, double ki, double kd,
                             double off_track_cte)
  : has_final_coefficients_(true),
    off_track_cte_(off_track_cte),
    track_length_(),
    distance_(),
    n_frames_(),
    safe_cte_(kSafeCteMargin * off_track_cte),
    max_cte_(),
    sum_cte_(),
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
    ++n_frames_;
    distance_ += kSpeedToDistanceCoeff * speed;
    sum_cte_ += std::fabs(cte);
    if (cte > max_cte_ && distance_ > no_max_cte_distance_) {
      max_cte_ = cte;
    }

    // Detect getting off track
    if (distance_ > no_off_track_distance_
        && (std::fabs(cte) > off_track_cte_ || speed < 1.0)) {
      auto error = kOffTrackPenalty / distance_;
      std::cout << "Getting off track at distance " << std::fixed
                << std::setprecision(0) << distance_ << "m, speed " << speed
                << "mph! " << std::defaultfloat;
      UpdateTwiddlerAndReset(error);
      on_reset();
      return;
    }

    // Detect completing the track
    if (distance_ > track_length_) {
      auto time = kSecondsPerFrame * n_frames_;
      auto average_speed = distance_ / (kMphToMps * time);
      auto avg_cte = sum_cte_ / n_frames_;
      auto error = max_cte_ * avg_cte;
      std::cout << "Max CTE " << std::fixed << std::setprecision(3) << max_cte_
                << ", average CTE " << avg_cte << " at distance "
                << std::setprecision(0) << distance_ << "m, time " << time
                << "s, average speed " << average_speed << "mph. "
                << std::defaultfloat;
      if (max_cte_ < kTargetCteMargin * off_track_cte_) {
        std::cout << "Using the final coefficients." << std::endl;
        has_final_coefficients_ = true;
      } else {
        UpdateTwiddlerAndReset(error);
        on_reset();
        return;
      }
    }
  }

  auto steering = Normalize(pid_->GetError(cte), -1.0, 1.0);
  // Throttle = 1 - 2 * (Speed / MaxSpeed) * (CTE / SafeCTE)
  auto throttle = Normalize(1.0 - 2.0 * (speed / kMaxSpeed)
                                      * (std::fabs(cte) / safe_cte_),
                            -1.0, 1.0);
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
  std::cout << "Error " << std::fixed << std::setprecision(3) << error
            << std::defaultfloat << ". Trying PID coefficients " << kp << ", "
            << ki << ", " << kd << "." << std::endl;
  pid_.reset(new Pid(kp, ki, kd));
  distance_ = 0;
  n_frames_ = 0;
  max_cte_ = 0;
  sum_cte_ = 0;
}
