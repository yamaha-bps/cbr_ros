// Copyright Yamaha 2021
// MIT License
// https://github.com/yamaha-bps/cbr_ros/blob/master/LICENSE

#ifndef CBR_ROS__PARAMETERS_HPP_
#define CBR_ROS__PARAMETERS_HPP_

#include <cbr_utils/type_traits.hpp>

#include <boost/hana/at_key.hpp>
#include <boost/hana/for_each.hpp>
#include <boost/hana/keys.hpp>
#include <boost/hana/range.hpp>

#include <rclcpp/rclcpp.hpp>

#include <type_traits>

namespace cbr
{

namespace detail
{

// types that are hana structs
template<typename T>
static constexpr bool is_hana_struct_v = boost::hana::Struct<T>::value;

// types that are directly ROS types
template<typename T>
static constexpr bool is_ros_parameter_v = std::is_constructible_v<rclcpp::ParameterValue, T>;

template<typename T>
struct ros_type
{
  using type = std::conditional_t<is_ros_parameter_v<T>,
      T,
      std::conditional_t<std::is_floating_point_v<T>,
      double,
      std::conditional_t<std::is_integral_v<T>, int64_t, void>>>;
};

template<typename T>
struct is_std_array_or_tuple : std::false_type
{
};

template<typename T, std::size_t N>
struct is_std_array_or_tuple<std::array<T, N>>: std::true_type
{
};

template<typename ... Ts>
struct is_std_array_or_tuple<std::tuple<Ts...>>: std::true_type
{
};

template<typename _ValT, typename F>
void iterator(std::string name, _ValT & val, const F & f)
{
  using ValT = std::decay_t<_ValT>;
  using RosT = typename ros_type<ValT>::type;
  using boost::hana::for_each, boost::hana::int_c;

  if constexpr (!std::is_same_v<RosT, void>) {
    f(name, val);
  } else if constexpr (is_hana_struct_v<ValT>) {
    for_each(
      boost::hana::keys(val), [&](auto key) {
        std::string name_i = boost::hana::to<char const *>(key);
        if (!name.empty()) {name_i = name + "." + name_i;}
        iterator(name_i, boost::hana::at_key(val, key), f);
      });
  } else if constexpr (is_std_array_or_tuple<ValT>::value) {
    for_each(
      boost::hana::make_range(int_c<0>, int_c<std::tuple_size<ValT>::value>), [&](auto i) {
        std::string name_i = name + "_" + std::to_string(i);
        iterator(name_i, std::get<i>(val), f);
      });
  } else if constexpr (is_std_vector<ValT>::value) {
    // TODO(pettni) convert vector of struct into struct of vectors
    // Can we call range iterator here?
    using VecValT = typename ValT::value_type;
    for_each(
      boost::hana::keys(VecValT{}), [&](auto) {

      });
  } else {
    static_assert(
      is_ros_parameter_v<ValT>|| !std::is_same_v<RosT, void>|| is_hana_struct_v<ValT>,
      "Invalid type");
  }
}

/* template<std::ranges::range R, typename F>
void range_iterator(const std::string & name, R & val, const F & f) {
  using ValT = std::decay_t<std::ranges::range_value_t<R>>;

  if constexpr (RosParameter<std::vector<ValT>>) {
    f.template operator()<ValT, R>(name, val);
  } else if constexpr (HanaStruct<ValT>) {
    boost::hana::for_each(boost::hana::keys(ValT{}), [&](auto key) {
      using ItemValT = std::decay_t<decltype(boost::hana::at_key(ValT{}, key))>;

      std::string name_i = boost::hana::to<char const *>(key);
      if (!name.empty()) { name_i = name + "." + name_i; }

      auto R_i = val | std::views::transform([&key](auto & val_i) -> ItemValT &
{ return boost::hana::at_key(val_i, key);
      });

      range_iterator(name_i, R_i, f);
    });
  } else {
    static_assert(RosParameter<std::vector<ValT>> || HanaStruct<ValT>, "Invalid
type");
  }
} */

}  // namespace detail

/**
 * @brief Declare parameters in node
 *
 * @tparam S hana struct
 * @param node where to declare parameters
 * @parrobo::prmam name parameter name
 * @param val parameter object with values
 */
template<typename S>
void declareParams(rclcpp::Node & node, const std::string & name, const S & val)
{
  detail::iterator(
    name, val, [&node](const std::string & pname, const auto & pval) {
      using RosT = typename detail::ros_type<std::decay_t<decltype(pval)>>::type;
      node.declare_parameter<RosT>(pname, static_cast<RosT>(pval));
    });
}

/**
 * @brief Undeclare parameters in node
 *
 * @tparam S hana struct
 * @param node where to undeclare parameters
 * @param name parameter name
 * @param val parameter object (values do not matter)
 */
template<typename S>
void undeclareParams(rclcpp::Node & node, const std::string & name, const S & val)
{
  detail::iterator(
    name, val, [&node](const auto & pname, const auto &) {node.undeclare_parameter(pname);});
}

/**
 * @brief Set parameters in node
 *
 * @tparam S hana struct
 * @param node where to set parameters
 * @param name parameter name
 * @param val parameter object with values
 */
template<typename S>
void setParams(rclcpp::Node & node, const std::string & name, const S & val)
{
  detail::iterator(
    name, val, [&node](const auto & pname, const auto & pval) {
      using RosT = typename detail::ros_type<std::decay_t<decltype(pval)>>::type;
      node.set_parameter(rclcpp::Parameter(pname, static_cast<RosT>(pval)));
    });
}

/**
 * @brief Get parameters from node (inplace version)
 *
 * @tparam S hana struct
 * @param[in] node where to get parameters
 * @param[in] name parameter name
 * @param[out] val parameter object
 */
template<typename S>
void getParams(const rclcpp::Node & node, const std::string & name, S & val)
{
  detail::iterator(
    name, val, [&](const auto & pname, auto & pval) {
      using ValT = std::decay_t<decltype(pval)>;
      using RosT = typename detail::ros_type<ValT>::type;
      pval = static_cast<ValT>(node.get_parameter(
        pname).get_parameter_value().template get<RosT>());
    });
}

/**
 * @brief Get parameters from node (return value version)
 *
 * @tparam S hana struct
 * @param node where to read parameters
 * @param name parameter name
 * @return struct with resulting parameter values
 */
template<typename S>
S getParams(const rclcpp::Node & node, std::string name = "")
{
  S val;
  getParams<S>(node, name, val);
  return val;
}

//////////////////
// EXPERIMENTAL //
//////////////////

// TODO: can these be unified with the above methods to make it seamless?

/**
 * @brief Declare range parameters in node
 *
 * @param node
 * @param name
 * @param val input range
 */
/* template<std::ranges::input_range R>
void declare_range(rclcpp::Node & node, const std::string & name, R & val) {
  detail::range_iterator(
    name, val, [&node]<typename ValT, std::ranges::range PR>(const std::string &
pname, const PR & pval) { node.declare_parameter<std::vector<ValT>>( pname,
std::vector<ValT>(std::ranges::begin(pval), std::ranges::end(pval)));
    });
} */

/**
 * @brief Get range parameters from node
 *
 * @param node
 * @param name
 * @param val output range
 *
 * IMPORTANT the output range must be initialized with the correct size
 */
/* template<std::ranges::range R>
void get_range(const rclcpp::Node & node, const std::string & name, R & val) {
  detail::range_iterator(
    name, val, [&node]<typename ValT, std::ranges::range PR>(const std::string &
pname, const PR & pval) { const auto tmp =
node.get_parameter(pname).get_parameter_value().get<std::vector<ValT>>(); const
std::size_t N = std::min(std::ranges::size(pval), std::ranges::size(tmp));
      std::ranges::copy_n(std::ranges::begin(tmp), N, std::ranges::begin(pval));
    });
} */

}  // namespace cbr

#endif  // CBR_ROS__PARAMETERS_HPP_
