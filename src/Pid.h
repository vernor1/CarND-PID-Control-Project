#ifndef PID_H
#define PID_H

class Pid {
public:
  // Constructor.
  // @param kp  Coefficient Kp of PID
  // @param ki  Coefficient Ki of PID
  // @param kd  Coefficient Kd of PID
  Pid(double kp, double ki, double kd);

  // Updates the PID error given cross-track error (CTE). Calculates the total
  // PID error.
  // @param cte  Cross-track error (CTE)
  double GetError(double cte);

private:
  // PID coefficients Kp, Ki, Kd
  double kp_;
  double ki_;
  double kd_;

  // PID errors
  double p_error_;
  double i_error_;
  double d_error_;

  // Previous CTE and indication whether it's initialized
  double cte_prev_;
  bool is_cte_prev_initialized_;
};

#endif // PID_H
