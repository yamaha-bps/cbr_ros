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

using namespace boost::hana;

namespace cbr {

namespace detail {

// types that are hana structs
template<typename T>
static constexpr bool is_hana_struct_v = Struct<T>::value;

// primitive ROS types
template<typename T>
static constexpr bool is_primitive_ros_param_v =
  std::is_same_v<T,
    bool> || std::is_same_v<T, int64_t> || std::is_same_v<T, double> || std::is_same_v<T, std::string>;

template<typename T>
struct is_vector_ros_param : std::false_type
{};

template<typename T>
struct is_vector_ros_param<std::vector<T>>
{
  static constexpr bool value = is_primitive_ros_param_v<T> || std::is_same_v<T, uint8_t>;
};

template<typename T>
static constexpr bool is_vector_ros_param_v = is_vector_ros_param<T>::value;

template<typename T>
static constexpr bool is_ros_param_v = is_primitive_ros_param_v<T> || is_vector_ros_param_v<T>;

template<typename T>
struct ros_type
{
  using type = std::conditional_t<is_ros_param_v<T>,
    T,
    std::conditional_t<std::is_floating_point_v<T>,
      double,
      std::conditional_t<std::is_integral_v<T>, int64_t, void>>>;
};

template<typename T>
struct is_std_array_or_tuple : std::false_type
{};

template<typename T, std::size_t N>
struct is_std_array_or_tuple<std::array<T, N>> : std::true_type
{};

template<typename... Ts>
struct is_std_array_or_tuple<std::tuple<Ts...>> : std::true_type
{};

/**
 * @brief Recursive iteration over a vector of values that yields a sequence of vectors.
 */
template<typename ValT, typename F>
void reference_iterator(
  const std::string & name, std::vector<std::reference_wrapper<ValT>> & val, const F & f)
{
  using RosT = typename ros_type<std::decay_t<ValT>>::type;

  if constexpr (!std::is_same_v<RosT, void> && !is_std_vector_v<RosT>) {
    f(name, val, int_c<1>);
  } else if constexpr (is_hana_struct_v<ValT>) {
    for_each(keys(ValT{}), [&](auto key) {
      using ItemValT  = std::decay_t<decltype(at_key(ValT{}, key))>;
      using CItemValT = std::conditional_t<std::is_const_v<ValT>, const ItemValT, ItemValT>;

      std::string name_i = to<char const *>(key);
      if (!name.empty()) { name_i = name + "." + name_i; }

      std::vector<std::reference_wrapper<CItemValT>> vec_i;

      std::transform(
        val.begin(), val.end(), std::back_inserter(vec_i), [&key](ValT & v) -> CItemValT & {
          return at_key(v, key);
        });

      reference_iterator(name_i, vec_i, f);
    });
  } else {
    static_assert((!std::is_same_v<RosT, void> && !is_std_vector_v<RosT>) || is_hana_struct_v<ValT>,
      "Invalid type");
  }
}

/**
 * @brief Declare parameters in node
 *
 * @tparam S hana struct
 * @param node where to declare parameters
 * @param name parameter name
 * @param val parameter object with values
 */
template<typename S>
void declareOrSetParams(
  rclcpp::Node & node, const std::string & name, const S & val, bool declare = true)
{
  using ValT = std::decay_t<decltype(val)>;
  using RosT = typename detail::ros_type<ValT>::type;

  if constexpr (!std::is_same_v<RosT, void>) {
    if (declare) {
      node.declare_parameter<RosT>(name, static_cast<RosT>(val));
    } else {
      node.set_parameter(rclcpp::Parameter(name, static_cast<RosT>(val)));
    }
  } else if constexpr (detail::is_hana_struct_v<ValT>) {
    for_each(keys(val), [&](auto key) {
      std::string name_i = to<char const *>(key);
      if (!name.empty()) { name_i = name + "." + name_i; }
      declareOrSetParams(node, name_i, at_key(val, key), declare);
    });
  } else if constexpr (detail::is_std_array_or_tuple<ValT>::value) {
    for_each(make_range(int_c<0>, int_c<std::tuple_size<ValT>::value>), [&](auto i) {
      std::string name_i = name + "_" + std::to_string(i);
      declareOrSetParams(node, name_i, std::get<i>(val), declare);
    });
  } else if constexpr (is_std_vector_v<ValT>) {
    using VecValT = typename ValT::value_type;

    std::vector<std::reference_wrapper<const VecValT>> refs;
    std::transform(
      val.begin(), val.end(), std::back_inserter(refs), [](const VecValT & x) -> const VecValT & {
        return x;
      });

    detail::reference_iterator(name, refs, [&](const auto & pname, auto pval, auto) {
      using ValT = typename std::decay_t<decltype(pval)>::value_type::type;
      using RosT = std::decay_t<typename detail::ros_type<ValT>::type>;
      std::vector<RosT> tmp;
      std::transform(pval.begin(), pval.end(), std::back_inserter(tmp), [](auto & x) {
        return static_cast<RosT>(x);
      });

      if (declare) {
        node.declare_parameter<std::vector<RosT>>(pname, tmp);
      } else {
        node.set_parameter(rclcpp::Parameter(pname, tmp));
      }
    });
  }
}

}  // namespace detail

/**
 * @brief Declare parameters in node
 *
 * @tparam S hana struct
 * @param node where to declare parameters
 * @param name parameter name
 * @param val parameter object with values
 */
template<typename S>
void declareParams(rclcpp::Node & node, const std::string & name, const S & val)
{
  detail::declareOrSetParams(node, name, val, true);
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
  detail::declareOrSetParams(node, name, val, false);
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
  using ValT = std::decay_t<decltype(val)>;
  using RosT = typename detail::ros_type<ValT>::type;

  if constexpr (!std::is_same_v<RosT, void>) {
    val = node.get_parameter(name).get_parameter_value().template get<RosT>();
  } else if constexpr (detail::is_hana_struct_v<ValT>) {
    for_each(keys(val), [&](auto key) {
      std::string name_i = to<char const *>(key);
      if (!name.empty()) { name_i = name + "." + name_i; }
      getParams(node, name_i, at_key(val, key));
    });
  } else if constexpr (detail::is_std_array_or_tuple<ValT>::value) {
    for_each(make_range(int_c<0>, int_c<std::tuple_size<ValT>::value>), [&](auto i) {
      std::string name_i = name + "_" + std::to_string(i);
      getParams(node, name_i, std::get<i>(val));
    });
  } else if constexpr (is_std_vector_v<ValT>) {
    using VecValT = typename ValT::value_type;

    std::vector<std::reference_wrapper<VecValT>> refs;

    // figure out size of output
    std::size_t sz = std::numeric_limits<std::size_t>::max();
    detail::reference_iterator(name, refs, [&](const auto & name, auto pval, auto) {
      using ValT = typename std::decay_t<decltype(pval)>::value_type::type;
      using RosT = std::decay_t<typename detail::ros_type<ValT>::type>;
      sz         = std::min(sz,
        node.get_parameter(name).get_parameter_value().template get<std::vector<RosT>>().size());
    });
    val.resize(sz);

    // write into output
    std::transform(
      val.begin(), val.end(), std::back_inserter(refs), [](VecValT & x) -> VecValT & { return x; });
    detail::reference_iterator(name, refs, [&](const auto & name, auto pval, auto) {
      using ValT = typename std::decay_t<decltype(pval)>::value_type::type;
      using RosT = std::decay_t<typename detail::ros_type<ValT>::type>;
      auto tmp   = node.get_parameter(name).get_parameter_value().template get<std::vector<RosT>>();
      for (auto i = 0u; i != sz; ++i) { pval[i].get() = static_cast<ValT>(tmp[i]); }
    });
  }
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

}  // namespace cbr

#endif  // CBR_ROS__PARAMETERS_HPP_
