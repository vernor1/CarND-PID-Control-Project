#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <functional>
#include <memory>
#include <vector>
#include "Pid.h"
#include "Twiddler.h"

class PidController {
public:
  // Contructor.
  // @param kp             Initial coefficient Kp of PID
  // @param ki             Initial coefficient Ki of PID
  // @param kd             Initial coefficient Kd of PID
  // @param dkp            Initial delta of Kp
  // @param dki            Initial delta of Ki
  // @param dkd            Initial delta of Kd
  // @param track_length   Track length in meters
  // @param off_track_cte  Off-track CTE
  PidController(double kp, double ki, double kd,
                double dkp, double dki, double dkd,
                double track_length, double off_track_cte);

  // Contructor.
  // @param kp  Final coefficient Kp of PID
  // @param ki  Final coefficient Ki of PID
  // @param kd  Final coefficient Kd of PID
  PidController(double kp, double ki, double kd);

  // Updates this PID controller with the new values of CTE and speed.
  // @param cte         Cross-track error (CTE)
  // @param speed       Speed in miler-per-hour
  // @param on_control  Functional object to control the simulator
  // @param on_reset    Functional object to reset the simulator
  void Update(double cte,
              double speed,
              std::function<void(double steering, double throttle)> on_control,
              std::function<void()> on_reset);

private:
  // Indicates the controller has final PID coefficients
  bool has_final_coefficients_;

  // Track length in meters
  double track_length_;

  // Off-track CTE
  double off_track_cte_;

  // Travel distance in meters
  double distance_;

  // Travel time in seconds
  double time_;

  // Maximum CTE registered so far
  double max_cte_;

  // Implementation of PID
  std::unique_ptr<Pid> pid_;

  // Implementation of Twiddler algorithm
  std::unique_ptr<Twiddler> twiddler_;

  // Updates the Twiddler with the new error value and resets related member
  void UpdateTwiddlerAndReset(double error);
};

#endif // PID_CONTROLLER_H
