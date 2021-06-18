// Copyright 2020 Yamaha Motor Corporation, USA
#ifndef CBR_ROS__CLOCK_TRAITS_ROS_HPP_
#define CBR_ROS__CLOCK_TRAITS_ROS_HPP_

#include <rclcpp/clock.hpp>

namespace cbr::detail
{

template<typename>
struct ClockTraits;

template<>
struct ClockTraits<rclcpp::Clock>
{
  using time_point = rclcpp::Time;
  using duration = rclcpp::Duration;

  template<typename duration_t>
  static duration_t duration_cast(const duration & d)
  {
    return d.to_chrono<duration_t>();
  }
};

}  // namespace cbr::detail

#endif  // CBR_ROS__CLOCK_TRAITS_ROS_HPP_
