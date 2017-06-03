#include "gtest/gtest.h"
#include "Robot.h"
#include "../src/Pid.h"
//#include <iostream>

void RunRobot(Robot& robot,
              double tau_p, double tau_d, double tau_i,
              size_t n_iterations) {
  Pid pid(tau_p, tau_i, tau_d);
  double x = 0;
  double y = 0;
  double orientation = 0;
  robot.Get(x, y, orientation);
  double cte0 = y;
  double cte_int = 0;
  for (auto i = 0; i < n_iterations; ++i) {
    robot.Get(x, y, orientation);
    double cte = y;
    cte_int += cte;
//    std::cout << "cte " << cte << ", cte_int " << cte_int << ", (cte - cte0) " << (cte - cte0) << std::endl;
    double steering = -tau_p * cte - tau_d * (cte - cte0) - tau_i * cte_int;
    cte0 = cte;
    robot.Move(steering, 1.0);
    ASSERT_NEAR(steering, pid.GetError(cte), 1e-9);
  }
}

TEST(Pid, NormalSteering) {
  double tau_p = 0.2;
  double tau_d = 3.0;
  double tau_i = 0.004;
  Robot robot;
  robot.Set(0, 1, 0);
  RunRobot(robot, tau_p, tau_d, tau_i, 100);
}

TEST(Pid, DriftSteering) {
  double tau_p = 2.093;
  double tau_d = 11.048;
  double tau_i = 0.025;
  Robot robot;
  robot.Set(0, 1, 0);
  robot.SetSteeringDrift(10 / 180 * M_PI);
  RunRobot(robot, tau_p, tau_d, tau_i, 100);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
