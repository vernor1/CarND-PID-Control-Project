#ifndef TWIDDLER_H
#define TWIDDLER_H

#include <vector>

class Twiddler {
public:
  // Contains a parameter value and its delta
  struct Parameter {
    double p;
    double dp;
    bool operator==(const Parameter& rhs) const {
      return rhs.p == p && rhs.dp == dp;
    }
  };
  typedef std::vector<Parameter> ParameterSequence;

  // Constructor.
  // @param parameters  Initial sequence of parameters
  Twiddler(const ParameterSequence& parameters);

  // Updates the error, generates a new set of parameters to try.
  // @param[in] error  The error value for the current parameters
  // @return           New parameters to try
  ParameterSequence UpdateError(double error);

private:
  // Defines Twiddler states
  enum class State {
    // Twiddler is not yet initialized with the error value for the initial
    // set of parameters
    kUninitialized,
    // Positive change of a parameter
    kPositiveChange,
    // Negative change of a parameter
    kNegativeChange
  };

  // Parameters
  ParameterSequence parameters_;

  // State
  State state_;

  // Identifier of the parameter being changed
  size_t parameter_id_;

  // Best error value so far
  double best_error_;

  // Completes the positive change of a parameter
  void CompletePositiveChange();
};

#endif // TWIDDLER_H
