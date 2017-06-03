#include "Pid.h"

// Public Members
// -----------------------------------------------------------------------------

Pid::Pid(double kp, double ki, double kd)
  : kp_(kp),
    ki_(ki),
    kd_(kd),
    p_error_(),
    i_error_(),
    d_error_(),
    cte_prev_(),
    is_cte_prev_initialized_() {
  // Empty.
}

double Pid::GetError(double cte) {
  // Calculate P-error
  p_error_ = cte;
  // Calculate I-error
  i_error_ += cte;
  // Calculate D-error
  if (!is_cte_prev_initialized_) {
    cte_prev_ = cte;
    is_cte_prev_initialized_ = true;
  }
  d_error_ = cte - cte_prev_;
  cte_prev_ = cte;
  return -kp_ * p_error_ - ki_ * i_error_ - kd_ * d_error_;
}
