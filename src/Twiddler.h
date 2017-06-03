#ifndef TWIDDLER_H
#define TWIDDLER_H

#include <vector>

// TODO: consider templatizing
class Twiddler {
public:
  struct Parameter {
    double p;
    double dp;
    bool operator==(const Parameter& rhs) const {
      return rhs.p == p && rhs.dp == dp;
    }
  };
  typedef std::vector<Parameter> ParameterSequence;

  // Ctor
  Twiddler(const ParameterSequence& parameters);

  ParameterSequence UpdateError(double error);

private:
  enum class State {
    kUninitialized,
    kPositiveChange,
    kNegativeChange
  };

  // Parameters
  ParameterSequence parameters_;
  State state_;
  size_t parameter_id_;
  double best_error_;

  void CompletePositiveChange();
};

#endif // TWIDDLER_H
