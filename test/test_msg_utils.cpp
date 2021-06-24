// Copyright Yamaha 2021
// MIT License
// https://github.com/yamaha-bps/cbr_ros/blob/master/LICENSE

#include <gtest/gtest.h>

#include "cbr_ros/msg_utils.hpp"

TEST(MsgUtils, ToFromVector3)
{
  Eigen::Vector3d vec{1, 2, 3};

  auto point_msg = cbr::msgs::to_msg<geometry_msgs::msg::Point>(vec);
  auto vec3_msg = cbr::msgs::to_msg<geometry_msgs::msg::Vector3>(vec);

  ASSERT_DOUBLE_EQ(point_msg.x, 1);
  ASSERT_DOUBLE_EQ(point_msg.y, 2);
  ASSERT_DOUBLE_EQ(point_msg.z, 3);

  ASSERT_DOUBLE_EQ(vec3_msg.x, 1);
  ASSERT_DOUBLE_EQ(vec3_msg.y, 2);
  ASSERT_DOUBLE_EQ(vec3_msg.z, 3);

  auto vec31 = cbr::msgs::from_msg(point_msg);
  auto vec32 = cbr::msgs::from_msg(vec3_msg);

  ASSERT_LE((vec31 - vec).norm(), 1e-10);
  ASSERT_LE((vec32 - vec).norm(), 1e-10);
}

TEST(MsgUtils, Vec6)
{
  Eigen::Matrix<double, 6, 1> vec;
  vec << 1, 2, 3, 4, 5, 6;

  ASSERT_LE((cbr::msgs::from_msg(cbr::msgs::to_msg(vec)) - vec).norm(), 1e-10);
}

TEST(MsgUtils, ToFromPose)
{
  Eigen::Isometry3d p1 = Eigen::Translation3d(5 * Eigen::Vector3d::UnitY()) * Eigen::Quaterniond::UnitRandom();

  auto p2 = cbr::msgs::from_msg(cbr::msgs::to_msg<geometry_msgs::msg::Pose>(p1));

  ASSERT_TRUE(p1.isApprox(p2));
}

TEST(MsgUtils, ToFromTransform)
{
  Eigen::Isometry3d p1 = Eigen::Translation3d(5 * Eigen::Vector3d::UnitY()) * Eigen::Quaterniond::UnitRandom();

  auto p2 = cbr::msgs::from_msg(cbr::msgs::to_msg<geometry_msgs::msg::Transform>(p1));

  ASSERT_TRUE(p1.isApprox(p2));
}

TEST(MsgUtils, RosToChrono)
{
  std::chrono::nanoseconds t_nsec(123978127698966879);
  auto t_nsec_copy = cbr::msgs::from_msg<std::chrono::nanoseconds>(cbr::msgs::to_msg(t_nsec));

  ASSERT_EQ(t_nsec.count(), t_nsec_copy.count());

  builtin_interfaces::msg::Time time_msg;
  time_msg.sec = 1234567;
  time_msg.nanosec = 8901234;

  auto time_msg_copy = cbr::msgs::to_msg(cbr::msgs::from_msg<std::chrono::nanoseconds>(time_msg));
  ASSERT_EQ(time_msg_copy.sec, time_msg.sec);
  ASSERT_EQ(time_msg_copy.nanosec, time_msg.nanosec);

  auto time_msg_double = cbr::msgs::from_msg<std::chrono::duration<double>>(time_msg).count();
  auto time_msg_encode = cbr::msgs::to_msg(std::chrono::duration<double>(time_msg_double));

  ASSERT_EQ(time_msg_encode.sec, time_msg.sec);
  ASSERT_EQ(time_msg_encode.nanosec, time_msg.nanosec);
}
