#ifndef PID_H
#define PID_H

class Pid {
public:
  // Ctor
  Pid(double kp, double ki, double kd);
  // Dtor
  virtual ~Pid() { }

  // Update the Pid error variables given cross track error.
  // Calculates the total Pid error.
  double GetError(double cte);

private:
  // PID Coefficients
  double kp_;
  double ki_;
  double kd_;
  // PID Errors
  double p_error_;
  double i_error_;
  double d_error_;
  // Previous CTE
  double cte_prev_;
  bool is_cte_prev_initialized_;
};

#endif // PID_H
