// Copyright Yamaha 2021
// MIT License
// https://github.com/yamaha-bps/cbr_ros/blob/master/LICENSE

#ifndef CBR_ROS__MSG_UTILS_HPP_
#define CBR_ROS__MSG_UTILS_HPP_

#include <Eigen/Dense>

#include <builtin_interfaces/msg/time.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <chrono>
#include <utility>

namespace cbr::msgs
{

namespace detail
{
// Traits to check if it's a chrono_duration
template<typename T>
struct is_chrono_duration : std::false_type {};
template<typename R, typename P>
struct is_chrono_duration<std::chrono::duration<R, P>>: std::true_type {};
}


/********************************************
    ROS types to Eigen/chrono types
*********************************************/

/**
 * Point/Vector3 message to Eigen Vector3d
 */
template<typename T>
inline std::enable_if_t<
  std::is_same_v<T, geometry_msgs::msg::Point>|| std::is_same_v<T, geometry_msgs::msg::Vector3>,
  Eigen::Vector3d
>
from_msg(const T & msg)
{
  return Eigen::Vector3d(msg.x, msg.y, msg.z);
}

/**
 * Twist message to Eigen Vector6d
 */
inline Eigen::Matrix<double, 6, 1> from_msg(const geometry_msgs::msg::Twist & msg)
{
  return (Eigen::Matrix<double, 6, 1>() << from_msg(msg.linear), from_msg(msg.angular)).finished();
}

/**
 * Quaternion message to Eigen quaternion
 */
inline Eigen::Quaterniond from_msg(const geometry_msgs::msg::Quaternion & msg)
{
  return Eigen::Quaterniond(msg.w, msg.x, msg.y, msg.z);
}

/**
 * Pose message to Eigen transform
 */

inline Eigen::Isometry3d from_msg(const geometry_msgs::msg::Pose & msg)
{
  return Eigen::Isometry3d(Eigen::Translation3d(from_msg(msg.position)) * from_msg(msg.orientation));
}

/**
 * Transform message to Eigen transform
 */
inline Eigen::Isometry3d from_msg(const geometry_msgs::msg::Transform & msg)
{
  return Eigen::Isometry3d(Eigen::Translation3d(from_msg(msg.translation)) * from_msg(msg.rotation));
}

/**
 * ROS duration to std::chrono duration
 */
template<typename T>
std::enable_if_t<detail::is_chrono_duration<T>::value, T>
from_msg(const builtin_interfaces::msg::Time & stamp)
{
  return std::chrono::duration_cast<T>(
    std::chrono::seconds(stamp.sec) + std::chrono::nanoseconds(stamp.nanosec)
  );
}

/********************************************
    Eigen/chrono types to ROS types
*********************************************/

/**
 * Eigen Vector3 to Point/Vector3/Lla message
 */
template<typename T = geometry_msgs::msg::Point, typename Derived>
std::enable_if_t<
  Derived::IsVectorAtCompileTime && Derived::SizeAtCompileTime == 3 &&
  (std::is_same_v<T, geometry_msgs::msg::Point>||
  std::is_same_v<T, geometry_msgs::msg::Vector3>),
  T
>
to_msg(const Eigen::MatrixBase<Derived> & p)
{
  T ret;
  ret.x = p.x();
  ret.y = p.y();
  ret.z = p.z();
  return ret;
}

/**
 * Eigen Vector6 to Twist message
 */
template<typename T = geometry_msgs::msg::Twist, typename Derived>
std::enable_if_t<
  Derived::IsVectorAtCompileTime && Derived::SizeAtCompileTime == 6 &&
  std::is_same_v<T, geometry_msgs::msg::Twist>,
  T
>
to_msg(const Eigen::MatrixBase<Derived> & p)
{
  geometry_msgs::msg::Twist ret;
  ret.linear = to_msg<geometry_msgs::msg::Vector3>(p.template head<3>());
  ret.angular = to_msg<geometry_msgs::msg::Vector3>(p.template tail<3>());
  return ret;
}

/**
 * Eigen Quaternion to Quaternion msg
 */
template<typename T = geometry_msgs::msg::Quaternion, typename Scalar>
std::enable_if_t<std::is_same_v<T, geometry_msgs::msg::Quaternion>, T>
to_msg(const Eigen::Quaternion<Scalar> & q)
{
  T ret;
  ret.w = q.w();
  ret.x = q.x();
  ret.y = q.y();
  ret.z = q.z();
  return ret;
}

/**
 * Eigen transform to Pose/Transform message
 */
template<typename T = geometry_msgs::msg::Pose, typename Scalar>
std::enable_if_t<
  std::is_same_v<T, geometry_msgs::msg::Pose>||
  std::is_same_v<T, geometry_msgs::msg::Transform>,
  T
>
to_msg(const Eigen::Transform<Scalar, 3, Eigen::Isometry> & pose)
{
  T ret;
  if constexpr (std::is_same_v<T, geometry_msgs::msg::Pose>) {
    ret.position = to_msg<geometry_msgs::msg::Point>(pose.translation());
    ret.orientation = to_msg(Eigen::Quaternion<Scalar>(pose.rotation()));
  }
  if constexpr (std::is_same_v<T, geometry_msgs::msg::Transform>)
  {
    ret.translation = to_msg<geometry_msgs::msg::Vector3>(pose.translation());
    ret.rotation = to_msg(Eigen::Quaternion<Scalar>(pose.rotation()));
  }
  return ret;
}

/**
 * std::chrono duration to ROS duration
 */
template<typename T = builtin_interfaces::msg::Time, typename S>
std::enable_if_t<detail::is_chrono_duration<S>::value, T>
to_msg(const S & t)
{
  T ret;
  auto t_sec = std::chrono::duration_cast<std::chrono::seconds>(t);
  ret.sec = t_sec.count();
  ret.nanosec = std::chrono::nanoseconds(
    std::chrono::duration_cast<std::chrono::nanoseconds>(t) - t_sec).count();
  return ret;
}

inline geometry_msgs::msg::Vector3 point_vec3(const geometry_msgs::msg::Point & p)
{
  geometry_msgs::msg::Vector3 out;
  out.x = p.x;
  out.y = p.y;
  out.z = p.z;
  return out;
}

inline geometry_msgs::msg::Point point_vec3(const geometry_msgs::msg::Vector3 & v)
{
  geometry_msgs::msg::Point out;
  out.x = v.x;
  out.y = v.y;
  out.z = v.z;
  return out;
}

}  // namespace cbr::msgs

#endif  // CBR_ROS__MSG_UTILS_HPP_
