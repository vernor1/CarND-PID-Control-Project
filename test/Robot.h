#include <cmath>
#include <random>

class Robot {
public:
  // Creates robot and initializes location/orientation to 0, 0, 0.
  Robot(double length = 20)
    : x_(0),
      y_(0),
      orientation_(0),
      length_(length),
      steering_noise_(0),
      distance_noise_(0),
      steering_drift_(0) { }

  virtual ~Robot() { }

  // Sets robot coordinates.
  void Set(double x, double y, double orientation) {
    x_ = x;
    y_ = y;
    orientation_ = std::remainder(orientation, 2. * M_PI);
  }

  // Gets robot coordinates;
  void Get(double& x, double& y, double& orientation) const {
    x = x_;
    y = y_;
    orientation = orientation_;
  }

  // Sets the noise parameters.
  void SetNoise(double steering_noise, double distance_noise) {
    steering_noise_ = steering_noise;
    distance_noise_ = distance_noise;
  }

  // Sets the systematical steering drift parameter.
  void SetSteeringDrift(double drift) {
    steering_drift_ = drift;
  }

  // Moves the robot.
  // @param steering  Front wheel steering angle, limited by max_steering_angle
  // @param distance  Total distance driven, most be non-negative
  void Move(double steering,
            double distance,
            double tolerance = 0.001,
            double max_steering_angle = M_PI / 4.0) {
    if (steering > max_steering_angle) {
      steering = max_steering_angle;
    }
    if (steering < -max_steering_angle) {
      steering = -max_steering_angle;
    }
    if (distance < 0) {
      distance = 0;
    }

    // Apply noise
    std::random_device random_device;
    std::default_random_engine rng(random_device());
    std::normal_distribution<double> dist_steering(steering, steering_noise_);
    std::normal_distribution<double> dist_distance(distance, distance_noise_);
    double steering2 = dist_steering(rng);
    double distance2 = dist_distance(rng);

    // Apply steering drift
    steering2 += steering_drift_;

    // Execute motion
    double turn = std::tan(steering2) * distance2 / length_;

    if (std::abs(turn) < tolerance) {
      // Approximate by straight line motion
      x_ += distance2 * std::cos(orientation_);
      y_ += distance2 * std::sin(orientation_);
      orientation_ = std::remainder(orientation_ + turn, 2. * M_PI);
    } else {
      // Approximate bicycle model for motion
      double radius = distance2 / turn;
      double cx = x_ - std::sin(orientation_) * radius;
      double cy = y_ + std::cos(orientation_) * radius;
      orientation_ = std::remainder(orientation_ + turn, 2. * M_PI);
      x_ = cx + std::sin(orientation_) * radius;
      y_ = cy - std::cos(orientation_) * radius;
    }
  }

private:
  double x_;
  double y_;
  double orientation_;
  double length_;
  double steering_noise_;
  double distance_noise_;
  double steering_drift_;
};
