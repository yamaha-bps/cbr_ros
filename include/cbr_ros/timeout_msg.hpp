// Copyright Yamaha 2021
// MIT License
// https://github.com/yamaha-bps/cbr_ros/blob/master/LICENSE

#ifndef CBR_ROS__TIMEOUT_MSG_HPP_
#define CBR_ROS__TIMEOUT_MSG_HPP_

#include <rclcpp/rclcpp.hpp>

#include <cbr_utils/type_traits.hpp>

#include <memory>
#include <utility>

namespace cbr
{

struct TimeoutMsgParams
{
  bool active{true};
  double delay{1.0};  // Duration before msg times out (in seconds)

  void check_correctness() const
  {
    if (delay < 0.) {
      throw std::invalid_argument("delay parameter must be >= 0.");
    }
  }
};

template<typename _msg_t>
class TimeoutMsg
{
public:
  using msg_t = _msg_t;

  TimeoutMsg() = default;
  TimeoutMsg(const TimeoutMsg &) = default;
  TimeoutMsg(TimeoutMsg &&) = default;
  TimeoutMsg & operator=(const TimeoutMsg &) = default;
  TimeoutMsg & operator=(TimeoutMsg &&) = default;
  ~TimeoutMsg() = default;

  template<typename T>
  TimeoutMsg(
    T && node,
    const TimeoutMsgParams & params = TimeoutMsgParams{})
  : node_(std::forward<T>(node)),
    prm_(params)
  {}

  template<typename T>
  void operator=(T && msg)
  {
    update(std::forward<T>(msg));
  }

  template<typename T>
  void update(T && msg)
  {
    static_assert(
      is_std_shared_ptr_v<std::decay_t<T>>,
      "Input must be a shared pointer.");
    static_assert(
      std::is_same_v<typename std::decay_t<T>::element_type, msg_t>,
      "Input's element type doesn't match class msg type.");

    msg_ = std::forward<T>(msg);

    if (msg_) {
      deadline_ = rclcpp::Time(msg_->header.stamp) +
        rclcpp::Duration(std::chrono::duration<double>(prm_.delay));
    }
  }

  bool timed_out(const rclcpp::Time & now) const noexcept
  {
    return !msg_ || (prm_.active && now > deadline_);
  }

  bool timed_out() const
  {
    if (!node_) {
      throw std::runtime_error("Invalid node pointer.");
    }
    return timed_out(node_->now());
  }

  std::shared_ptr<const msg_t> get(const rclcpp::Time & now) const noexcept
  {
    if (!timed_out(now)) {
      return msg_;
    }

    return {};
  }

  std::shared_ptr<const msg_t> get() const
  {
    if (!node_) {
      throw std::runtime_error("Invalid node pointer.");
    }
    return get(node_->now());
  }

  std::shared_ptr<const msg_t> get_latest() const noexcept
  {
    return msg_;
  }

  template<typename T>
  void set_node(T && node) noexcept
  {
    node_ = std::forward<T>(node);
  }

  template<typename T>
  void set_params(T && prm) noexcept
  {
    prm_ = std::forward<T>(prm);
  }

  const TimeoutMsgParams & get_params() const noexcept
  {
    return prm_;
  }

  void reset() noexcept
  {
    msg_.reset();
  }

  bool refresh(const rclcpp::Time & now) noexcept
  {
    if (msg_) {
      msg_->header.stamp = now;
      deadline_ = rclcpp::Time(msg_->header.stamp) +
        rclcpp::Duration(std::chrono::duration<double>(prm_.delay));
      return true;
    }
    return false;
  }

  bool refresh()
  {
    if (!node_) {
      throw std::runtime_error("Invalid node pointer.");
    }
    return refresh(node_->now());
  }

protected:
  std::shared_ptr<rclcpp::Node> node_;
  TimeoutMsgParams prm_;
  std::shared_ptr<msg_t> msg_;
  rclcpp::Time deadline_;
};

}  // namespace cbr

#endif  // CBR_ROS__TIMEOUT_MSG_HPP_
