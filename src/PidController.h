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
  // @param off_track_cte  CTE when the vehicle is considered off-track
  // @param dkp            Initial delta of Kp
  // @param dki            Initial delta of Ki
  // @param dkd            Initial delta of Kd
  // @param track_length   Track length in meters
  PidController(double kp, double ki, double kd, double off_track_cte,
                double dkp, double dki, double dkd, double track_length);

  // Contructor.
  // @param kp  Final coefficient Kp of PID
  // @param ki  Final coefficient Ki of PID
  // @param kd  Final coefficient Kd of PID
  // @param off_track_cte  CTE when the vehicle is considered off-track
  PidController(double kp, double ki, double kd, double off_track_cte);

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

  // CTE when the vehicle is considered off-track
  double off_track_cte_;

  // Track length in meters
  double track_length_;

  // Travel distance in meters
  double distance_;

  // Initial distance where max CTE tracking is not yet done
  double no_max_cte_distance_;

  // Initial distance where going off-track is not detected
  double no_off_track_distance_;

  // Number of frames observed
  unsigned long int n_frames_;

  // Max safe CTE when driving normally
  double safe_cte_;

  // Maximum CTE registered so far
  double max_cte_;

  // Sum of CTE
  double sum_cte_;

  // Implementation of PID
  std::unique_ptr<Pid> pid_;

  // Implementation of Twiddler algorithm
  std::unique_ptr<Twiddler> twiddler_;

  // Updates the Twiddler with the new error value and resets related member
  void UpdateTwiddlerAndReset(double error);
};

#endif // PID_CONTROLLER_H
