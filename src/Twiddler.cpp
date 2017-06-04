#include "Twiddler.h"

// Public Members
// -----------------------------------------------------------------------------

Twiddler::Twiddler(const ParameterSequence& parameters)
  : parameters_(parameters),
    state_(State::kUninitialized),
    parameter_id_(),
    best_error_() {
  // Empty.
}

Twiddler::ParameterSequence Twiddler::UpdateError(double error) {
  if (parameters_.empty()) {
    return parameters_;
  }
  switch (state_) {
    case State::kUninitialized:
      // Initialize
      best_error_ = error;
      parameters_.at(parameter_id_).p += parameters_.at(parameter_id_).dp;
      state_ = State::kPositiveChange;
      break;
    case State::kPositiveChange:
      // Handle positive change
      if (error < best_error_) {
        best_error_ = error;
        parameters_.at(parameter_id_).dp *= 1.1;
        CompletePositiveChange();
      } else {
        parameters_.at(parameter_id_).p -= 2.
          * parameters_.at(parameter_id_).dp;
        state_ = State::kNegativeChange;
      }
      break;
  case State::kNegativeChange:
    // Handle negative change
    if (error < best_error_) {
      best_error_ = error;
      parameters_.at(parameter_id_).dp *= 1.1;
      CompletePositiveChange();
    } else {
      parameters_.at(parameter_id_).p += parameters_.at(parameter_id_).dp;
      parameters_.at(parameter_id_).dp *= 0.9;
      CompletePositiveChange();
    }
  }
  return parameters_;
}

// Private Members
// -----------------------------------------------------------------------------

void Twiddler::CompletePositiveChange() {
  parameter_id_ = parameter_id_ + 1 < parameters_.size() ?
                  parameter_id_ + 1 : 0;
  parameters_.at(parameter_id_).p += parameters_.at(parameter_id_).dp;
  state_ = State::kPositiveChange;
}
