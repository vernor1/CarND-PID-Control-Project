#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "Robot.h"
#include "../src/PidController.h"
//#include <iostream>

const auto kKp = 0.1;
const auto kKi = 1e-4;
const auto kKd = 4.0;
const auto kdKp = 0.01;
const auto kdKi = 1e-5;
const auto kdKd = 0.1;

using namespace std::placeholders;
using ::testing::_;
using ::testing::SaveArg;

class User {
public:
  MOCK_METHOD2(OnControl, void(double steering, double throttle));
  MOCK_METHOD0(OnReset, void());
};

TEST(PidController, FinalCoefficients) {
  User user;
//  auto kp = 0.1;
//  auto ki = 1e-4;
//  auto kd = 4.0;
  PidController pid_controller(kKp, kKi, kKd);
  double steering;
  double throttle;
//  EXPECT_CALL(user, OnControl(_, _))
//    .Times(1)
//    .WillOnce(DoAll(SaveArg<0>(&steering), SaveArg<1>(&throttle)));
//  pid_controller.Update(0, 0, std::bind(&User::OnControl, &user, _1, _2), std::bind(&User::OnReset, &user)); 
//  EXPECT_NEAR(steering, 0, 1e-9);
//  EXPECT_NEAR(throttle, 1, 1e-9);

  Robot robot;
  robot.Set(0, 1, 0);
  double x = 0;
  double y = 0;
  double orientation = 0;
  robot.Get(x, y, orientation);
  double cte0 = y;
  double cte_int = 0;
  for (auto i = 0; i < 200; ++i) {
    robot.Get(x, y, orientation);
    double cte = y;
    cte_int += cte;
    double angle = -kKp * cte - kKd * (cte - cte0) - kKi * cte_int;
    cte0 = cte;
    robot.Move(angle, 1.0);
    EXPECT_CALL(user, OnControl(_, _))
      .Times(1)
      .WillOnce(DoAll(SaveArg<0>(&steering), SaveArg<1>(&throttle)));
    pid_controller.Update(cte, 60, std::bind(&User::OnControl, &user, _1, _2), std::bind(&User::OnReset, &user)); 
//    std::cout << "cte " << cte << ", steering " << steering << std::endl;
  }
  EXPECT_NEAR(steering, 0, 1e-3);
  EXPECT_NEAR(throttle, 1, 1e-3);
}

TEST(PidController, InitialCoefficientsOnTrack) {
  User user;
/*
  auto kp = 0.1;
  auto ki = 1e-4;
  auto kd = 4.0;
  auto dkp = 0.01;
  auto dki = 1e-5;
  auto dkd = 0.1;
*/
  PidController pid_controller(kKp, kKi, kKd, kdKp, kdKi, kdKd, 10, 5);
  double steering;
  double throttle;
  for (auto i = 0; i < 5; ++i) {
    EXPECT_CALL(user, OnControl(_, _))
      .Times(1)
      .WillOnce(DoAll(SaveArg<0>(&steering), SaveArg<1>(&throttle)));
    pid_controller.Update(4.99, 100, std::bind(&User::OnControl, &user, _1, _2), std::bind(&User::OnReset, &user)); 
  }
  EXPECT_NEAR(steering, -0.5, 0.1);
  EXPECT_NEAR(throttle, -1, 1e-3);
}

TEST(PidController, InitialCoefficientsOffTrack) {
  User user;
/*
  auto kp = 0.1;
  auto ki = 1e-4;
  auto kd = 4.0;
  auto dkp = 0.01;
  auto dki = 1e-5;
  auto dkd = 0.1;
*/
  PidController pid_controller(kKp, kKi, kKd, kdKp, kdKi, kdKd, 10, 5);
  double steering;
  double throttle;
  for (auto i = 0; i < 5; ++i) {
    EXPECT_CALL(user, OnControl(_, _)).Times(1);
    pid_controller.Update(4.99, 100, std::bind(&User::OnControl, &user, _1, _2), std::bind(&User::OnReset, &user)); 
  }
  EXPECT_CALL(user, OnReset()).Times(1);
  pid_controller.Update(5.01, 100, std::bind(&User::OnControl, &user, _1, _2), std::bind(&User::OnReset, &user)); 
}

TEST(PidController, InitialCoefficientsTrackComplete) {
  User user;
/*
  auto kp = 0.1;
  auto ki = 1e-4;
  auto kd = 4.0;
  auto dkp = 0.01;
  auto dki = 1e-5;
  auto dkd = 0.1;
*/
  PidController pid_controller(kKp, kKi, kKd, kdKp, kdKi, kdKd, 10, 5);
  double steering;
  double throttle;
  for (auto i = 0; i < 5; ++i) {
    EXPECT_CALL(user, OnControl(_, _)).Times(1);
    pid_controller.Update(4.99, 100, std::bind(&User::OnControl, &user, _1, _2), std::bind(&User::OnReset, &user)); 
  }
  EXPECT_CALL(user, OnReset()).Times(1);
  pid_controller.Update(4.99, 100, std::bind(&User::OnControl, &user, _1, _2), std::bind(&User::OnReset, &user)); 
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleMock(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
