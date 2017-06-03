#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <memory>
#include <vector>
#include "Pid.h"
#include "Twiddler.h"

class PidController {
public:
  // Ctors
  PidController(double kp, double ki, double kd,
                double dkp, double dki, double dkd,
                double track_length, double off_track_cte);
  PidController(double kp, double ki, double kd);

  void Update(double cte,
              double speed,
              std::function<void(double steering, double throttle)> on_control,
              std::function<void()> on_reset);

private:
  bool has_final_coefficients_;
  double track_length_;
  double off_track_cte_;
  double distance_;
  double time_;
  double max_cte_;
  std::unique_ptr<Pid> pid_;
  std::unique_ptr<Twiddler> twiddler_;

  void UpdateTwiddlerAndReset(double error);
};

#endif // PID_CONTROLLER_H
