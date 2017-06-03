#include "Pid.h"
//#include <iostream>

Pid::Pid(double kp, double ki, double kd)
  : kp_(kp),
    ki_(ki),
    kd_(kd),
    p_error_(0),
    i_error_(0),
    d_error_(0),
    cte_prev_(0),
    is_cte_prev_initialized_(false) {
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
//  std::cout << "p_error_ " << p_error_ << ", i_error_ " << i_error_ << ", d_error_ " << d_error_ << std::endl;
  return -kp_ * p_error_ - ki_ * i_error_ - kd_ * d_error_;
}
