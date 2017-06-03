#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "Robot.h"
#include "../src/Twiddler.h"

using ::testing::Pointwise;

template<typename T>
bool IsNear(T v1, T v2, T tolerance) {
  return v1 > v2 - tolerance && v1 < v2 + tolerance;
}

MATCHER_P(NearPointwise, tol, "Not near!") {
  return IsNear(std::get<0>(arg).p, std::get<1>(arg).p, tol)
    && IsNear(std::get<0>(arg).dp, std::get<1>(arg).dp, tol);
}

Robot MakeRobot() {
  Robot robot;
  robot.Set(0, 1, 0);
  robot.SetSteeringDrift(10. / 180. * M_PI);
  return robot;
}

double RunRobot(Robot& robot,
                double tau_p, double tau_i, double tau_d,
                size_t n_iterations) {
  double x = 0;
  double y = 0;
  double orientation = 0;
  double error = 0;
  robot.Get(x, y, orientation);
  double cte0 = y;
  double cte_int = 0;
  for (auto i = 0; i < 2 * n_iterations; ++i) {
    robot.Get(x, y, orientation);
    double cte = y;
    cte_int += cte;
    double steering = -tau_p * cte - tau_d * (cte - cte0) - tau_i * cte_int;
    cte0 = cte;
    robot.Move(steering, 1.0);
    if (i >= n_iterations) {
      error += cte * cte;
    }
  }
  return error / n_iterations;
}

double DeltaParametersSum(const Twiddler::ParameterSequence& p) {
  return p[0].dp + p[1].dp + p[2].dp;
}

TEST(Twiddler, Manual) {
  Twiddler twiddler({{.p=0, .dp=0.5}, {.p=0, .dp=10}, {.p=0, .dp=0.01}});
  Twiddler::ParameterSequence p0
    = {{.p=0.5, .dp=0.5}, {.p=0, 10}, {.p=0, .dp=0.01}};
  EXPECT_THAT(p0, Pointwise(NearPointwise(1e-9), twiddler.UpdateError(1)));
  p0 = {{.p=0.5, .dp=0.55}, {.p=10, 10}, {.p=0, .dp=0.01}};
  EXPECT_THAT(p0, Pointwise(NearPointwise(1e-9), twiddler.UpdateError(0.9)));
  p0 = {{.p=0.5, .dp=0.55}, {.p=10, 11}, {.p=0.01, .dp=0.01}};
  EXPECT_THAT(p0, Pointwise(NearPointwise(1e-9), twiddler.UpdateError(0.5)));
  p0 = {{.p=1.05, .dp=0.55}, {.p=10, 11}, {.p=0.01, .dp=0.011}};
  EXPECT_THAT(p0, Pointwise(NearPointwise(1e-9), twiddler.UpdateError(0.4)));
  p0 = {{.p=1.05, .dp=0.605}, {.p=21, 11}, {.p=0.01, .dp=0.011}};
  EXPECT_THAT(p0, Pointwise(NearPointwise(1e-9), twiddler.UpdateError(0.3)));
  p0 = {{.p=1.05, .dp=0.605}, {.p=-1, 11}, {.p=0.01, .dp=0.011}};
  EXPECT_THAT(p0, Pointwise(NearPointwise(1e-9), twiddler.UpdateError(0.8)));
  p0 = {{.p=1.05, .dp=0.605}, {.p=10, 9.9}, {.p=0.021, .dp=0.011}};
  EXPECT_THAT(p0, Pointwise(NearPointwise(1e-9), twiddler.UpdateError(0.7)));
  p0 = {{.p=1.655, .dp=0.605}, {.p=10, 9.9}, {.p=0.021, .dp=0.0121}};
  EXPECT_THAT(p0, Pointwise(NearPointwise(1e-9), twiddler.UpdateError(0.1)));
  p0 = {{.p=1.655, .dp=0.6655}, {.p=19.9, 9.9}, {.p=0.021, .dp=0.0121}};
  EXPECT_THAT(p0, Pointwise(NearPointwise(1e-9), twiddler.UpdateError(0.05)));
  p0 = {{.p=1.655, .dp=0.6655}, {.p=0.1, 9.9}, {.p=0.021, .dp=0.0121}};
  EXPECT_THAT(p0, Pointwise(NearPointwise(1e-9), twiddler.UpdateError(0.7)));
  p0 = {{.p=1.655, .dp=0.6655}, {.p=10, 8.91}, {.p=0.0331, .dp=0.0121}};
  EXPECT_THAT(p0, Pointwise(NearPointwise(1e-9), twiddler.UpdateError(0.6)));
  p0 = {{.p=1.655, .dp=0.6655}, {.p=10, 8.91}, {.p=0.0089, .dp=0.0121}};
  EXPECT_THAT(p0, Pointwise(NearPointwise(1e-9), twiddler.UpdateError(0.1)));
  p0 = {{.p=2.3205, .dp=0.6655}, {.p=10, 8.91}, {.p=0.021, .dp=0.01089}};
  EXPECT_THAT(p0, Pointwise(NearPointwise(1e-9), twiddler.UpdateError(0.2)));
}

TEST(Twiddler, Robot) {
  Twiddler::ParameterSequence p0
    = {{.p=0, .dp=0.5}, {.p=0, .dp=0.01}, {.p=0, 10}};
  auto p0_final = p0;
  Twiddler twiddler(p0);
  auto robot = MakeRobot();
  double best_error = RunRobot(robot, p0[0].p, p0[1].p, p0[2].p, 100);
  auto p1 = twiddler.UpdateError(best_error);
  auto p1_final = p1;
  auto step = 0;
  while (DeltaParametersSum(p0) > 1e-3) {
    ++step;
    for (auto i = 0; i < p0.size(); ++i) {
      p0[i].p += p0[i].dp;
      robot = MakeRobot();
      auto error = RunRobot(robot, p0[0].p, p0[1].p, p0[2].p, 100);
      p0_final = p0;
      p1_final = p1;
      p1 = twiddler.UpdateError(error);
      if (error < best_error) {
        best_error = error;
        p0[i].dp *= 1.1;
      } else {
        p0[i].p -= 2 * p0[i].dp;
        robot = MakeRobot();
        error = RunRobot(robot, p0[0].p, p0[1].p, p0[2].p, 100);
        p0_final = p0;
        p1_final = p1;
        p1 = twiddler.UpdateError(error);
        if (error < best_error) {
          best_error = error;
          p0[i].dp *= 1.1;
        } else {
          p0[i].p += p0[i].dp;
          p0[i].dp *= 0.9;
        }
      }
    }
  }
  EXPECT_THAT(p0_final, Pointwise(NearPointwise(1e-9), p1_final));
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleMock(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
