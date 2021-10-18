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

#include <ranges>
#include <type_traits>

namespace cbr {

namespace detail {

// types that are hana structs
template<typename T>
static constexpr bool is_hana_struct_v = boost::hana::Struct<T>::value;

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

template<typename T1, typename T2>
struct is_std_array_or_tuple<std::pair<T1, T2>> : std::true_type
{};

template<typename... Ts>
struct is_std_array_or_tuple<std::tuple<Ts...>> : std::true_type
{};

/**
 * @brief Recursive iteration over a vector of values that applies f to each unfolded vector.
 */
template<typename Val, typename F>
void reference_iterator(
  const std::string & name, std::vector<std::reference_wrapper<Val>> & val, const F & f)
{
  using RVal = typename ros_type<std::decay_t<Val>>::type;

  if constexpr (!std::is_same_v<RVal, void> && !is_std_vector_v<RVal>) {
    f(name, val);
  } else if constexpr (is_hana_struct_v<Val>) {
    boost::hana::for_each(boost::hana::keys(Val{}), [&](auto key) {
      using ItemValT  = std::decay_t<decltype(boost::hana::at_key(Val{}, key))>;
      using CItemValT = std::conditional_t<std::is_const_v<Val>, const ItemValT, ItemValT>;

      std::string name_i = boost::hana::to<char const *>(key);
      if (!name.empty()) { name_i = name + "." + name_i; }

      std::vector<std::reference_wrapper<CItemValT>> vec_i;

      std::transform(
        val.begin(), val.end(), std::back_inserter(vec_i), [&key](Val & v) -> CItemValT & {
          return boost::hana::at_key(v, key);
        });

      reference_iterator(name_i, vec_i, f);
    });
  } else {
    static_assert((!std::is_same_v<RVal, void> && !is_std_vector_v<RVal>) || is_hana_struct_v<Val>,
      "Invalid type");
  }
}

/**
 * @brief Declare or set parameters in n
 *
 * @tparam S parameter type
 * @param n rclcpp::Node
 * @param name parameter name
 * @param val parameter value
 */
template<typename S>
void declareOrSetParams(
  rclcpp::Node & n, const std::string & name, const S & val, bool declare = true)
{
  using RVal = typename detail::ros_type<S>::type;

  if constexpr (!std::is_same_v<RVal, void>) {
    if (declare) {
      n.declare_parameter<RVal>(name, static_cast<RVal>(val));
    } else {
      n.set_parameter(rclcpp::Parameter(name, static_cast<RVal>(val)));
    }
  } else if constexpr (detail::is_hana_struct_v<S>) {
    boost::hana::for_each(boost::hana::keys(val), [&](auto key) {
      std::string name_i = boost::hana::to<char const *>(key);
      if (!name.empty()) { name_i = name + "." + name_i; }
      declareOrSetParams(n, name_i, boost::hana::at_key(val, key), declare);
    });
  } else if constexpr (detail::is_std_array_or_tuple<S>::value) {
    boost::hana::for_each(
      boost::hana::make_range(boost::hana::int_c<0>, boost::hana::int_c<std::tuple_size<S>::value>),
      [&](auto i) {
        std::string name_i = name + "_" + std::to_string(i);
        declareOrSetParams(n, name_i, std::get<i>(val), declare);
      });
  } else if constexpr (std::ranges::range<S>) {
    using RV  = typename std::ranges::range_value_t<S>;
    using RVRosVal = typename detail::ros_type<RV>::type;

    if constexpr (is_primitive_ros_param_v<RVRosVal>) {
      // range can be declared directly
      const std::vector<RVRosVal> tmp(std::ranges::begin(val), std::ranges::end(val));
      if (declare) {
        n.declare_parameter<std::vector<RVRosVal>>(name, tmp);
      } else {
        n.set_parameter(rclcpp::Parameter(name, tmp));
      }
    } else {
      // got a range of X, must convert to a X of ranges
      if constexpr (detail::is_hana_struct_v<RV>) {
        boost::hana::for_each(boost::hana::keys(RV{}), [&](auto key) {
          using ItemValT     = std::decay_t<decltype(boost::hana::at_key(RV{}, key))>;
          std::string name_i = boost::hana::to<char const *>(key);
          if (!name.empty()) { name_i = name + "." + name_i; }
          declareOrSetParams(n,
            name_i,
            std::ranges::views::transform(val, [&](const auto & x) -> const ItemValT & { return boost::hana::at_key(x, key); }),
            declare);
        });
      } else if constexpr (detail::is_std_array_or_tuple<RV>::value) {
        boost::hana::for_each(boost::hana::make_range(boost::hana::int_c<0>,
                                boost::hana::int_c<std::tuple_size<RV>::value>),
          [&](auto i) {
            using ItemValT     = std::decay_t<std::tuple_element_t<i, RV>>;
            std::string name_i = name + "_" + std::to_string(i);
            declareOrSetParams(n,
              name_i,
              std::ranges::views::transform(val, [&](const auto & x) -> const ItemValT & { return std::get<i>(val); }),
              declare);
          });
      }
    }
  }
}

}  // namespace detail

/**
 * @brief Declare parameters in n
 *
 * @tparam S parameter type
 * @param n rclcpp::Node
 * @param name parameter name
 * @param val parameter value
 */
template<typename S>
void declareParams(rclcpp::Node & n, const std::string & name, const S & val)
{
  detail::declareOrSetParams(n, name, val, true);
}

/**
 * @brief Set parameters in n
 *
 * @tparam S parameter type
 * @param n rclcpp::Node
 * @param name parameter name
 * @param val parameter value
 */
template<typename S>
void setParams(rclcpp::Node & n, const std::string & name, const S & val)
{
  detail::declareOrSetParams(n, name, val, false);
}

/**
 * @brief Get parameters from n (inplace version)
 *
 * @tparam S parameter type
 * @param[in] n rclcpp::Node
 * @param[in] name parameter name
 * @param[out] val parameter value
 */
template<typename S>
void getParams(const rclcpp::Node & n, const std::string & name, S & val)
{
  using Val  = std::decay_t<decltype(val)>;
  using RVal = typename detail::ros_type<Val>::type;

  if constexpr (!std::is_same_v<RVal, void>) {
    val = n.get_parameter(name).get_parameter_value().template get<RVal>();
  } else if constexpr (detail::is_hana_struct_v<Val>) {
    boost::hana::for_each(boost::hana::keys(val), [&](auto key) {
      std::string name_i = boost::hana::to<char const *>(key);
      if (!name.empty()) { name_i = name + "." + name_i; }
      getParams(n, name_i, boost::hana::at_key(val, key));
    });
  } else if constexpr (detail::is_std_array_or_tuple<Val>::value) {
    boost::hana::for_each(boost::hana::make_range(
                            boost::hana::int_c<0>, boost::hana::int_c<std::tuple_size<Val>::value>),
      [&](auto i) {
        std::string name_i = name + "_" + std::to_string(i);
        getParams(n, name_i, std::get<i>(val));
      });
  } else if constexpr (is_std_vector_v<Val>) {
    using VVal = typename Val::value_type;

    std::vector<std::reference_wrapper<VVal>> refs;

    // figure out size of output
    std::size_t sz = std::numeric_limits<std::size_t>::max();
    detail::reference_iterator(name, refs, [&](const auto & subname, auto pval) {
      using Val  = typename std::decay_t<decltype(pval)>::value_type::type;
      using RVal = std::decay_t<typename detail::ros_type<Val>::type>;
      sz         = std::min(sz,
        n.get_parameter(subname).get_parameter_value().template get<std::vector<RVal>>().size());
    });
    val.resize(sz);

    // insert references
    std::transform(
      val.begin(), val.end(), std::back_inserter(refs), [](VVal & x) -> VVal & { return x; });

    // write into output
    detail::reference_iterator(name, refs, [&](const auto & subname, auto pval) {
      using Val  = typename std::decay_t<decltype(pval)>::value_type::type;
      using RVal = std::decay_t<typename detail::ros_type<Val>::type>;
      const auto tmp =
        n.get_parameter(subname).get_parameter_value().template get<std::vector<RVal>>();
      for (auto i = 0u; i != sz; ++i) { pval[i].get() = static_cast<Val>(tmp[i]); }
    });
  }
}

/**
 * @brief Update parameter struct from a single rclcpp::Parameter. Useful in parameter callbacks
 *
 * @param prm parameter information (contains name and value)
 * @param name parameter name
 * @param val parameter struct
 *
 * @return true if parameter was updated in val, false otherwise
 *
 * Example: Declare parameters and register a parameter callback that updates parameters in a
 * struct: Params prm{}; cbr::declareParams(*this, "my_ns", prm);
 *   add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> & params) {
 *     for (const auto & param : params) { cbr::updateParam(param, "my_ns", prm); }
 *     rcl_interfaces::msg::SetParametersResult res;
 *     res.successful = true;
 *     return res;
 *   });
 */
template<typename S>
bool updateParam(const rclcpp::Parameter & prm, const std::string & name, S & val)
{
  using Val  = std::decay_t<decltype(val)>;
  using RVal = typename cbr::detail::ros_type<Val>::type;

  bool success = false;

  if (prm.get_name().substr(0, name.size()) == name) {
    if constexpr (!std::is_same_v<RVal, void>) {
      if (prm.get_name() == name) {
        val     = prm.get_value<RVal>();
        success = true;
      }
    } else if constexpr (cbr::detail::is_hana_struct_v<Val>) {
      boost::hana::for_each(boost::hana::keys(val), [&](auto key) {
        if (!success) {
          std::string name_i = boost::hana::to<char const *>(key);
          if (!name.empty()) { name_i = name + "." + name_i; }
          success |= updateParam(prm, name_i, boost::hana::at_key(val, key));
        }
      });
    } else if constexpr (cbr::detail::is_std_array_or_tuple<Val>::value) {
      boost::hana::for_each(boost::hana::make_range(boost::hana::int_c<0>,
                              boost::hana::int_c<std::tuple_size<Val>::value>),
        [&](auto i) {
          if (!success) {
            std::string name_i = name + "_" + std::to_string(i);
            success |= updateParam(prm, name_i, std::get<i>(val));
          }
        });
    } else if constexpr (cbr::is_std_vector_v<Val>) {
      using VVal = typename Val::value_type;

      // create vector of references to object
      std::vector<std::reference_wrapper<VVal>> refs;

      // resize as appropriate
      std::size_t sz = val.size();
      detail::reference_iterator(name, refs, [&](const auto & subname, auto pval) {
        using Val  = typename std::decay_t<decltype(pval)>::value_type::type;
        using RVal = std::decay_t<typename detail::ros_type<Val>::type>;
        if (prm.get_name() == subname) {
          sz = std::min(sz, prm.template get_value<std::vector<RVal>>().size());
        }
      });
      val.resize(sz);

      // insert references
      std::transform(
        val.begin(), val.end(), std::back_inserter(refs), [](VVal & x) -> VVal & { return x; });

      // write into output
      detail::reference_iterator(name, refs, [&](const auto & subname, auto pval) {
        using Val  = typename std::decay_t<decltype(pval)>::value_type::type;
        using RVal = std::decay_t<typename detail::ros_type<Val>::type>;
        if (prm.get_name() == subname) {
          const auto vec_value = prm.template get_value<std::vector<RVal>>();
          success              = true;
          for (auto i = 0u; i != sz; ++i) { pval[i].get() = static_cast<Val>(vec_value[i]); }
        }
      });
    }
  }

  return success;
}

/**
 * @brief Get parameters from n (return value version)
 *
 * @tparam S hana struct
 * @param n rclcpp::Node
 * @param name parameter name
 *
 * @return parameter value
 */
template<typename S>
S getParams(const rclcpp::Node & n, const std::string & name = "")
{
  S val;
  getParams<S>(n, name, val);
  return val;
}

/**
 * @brief Declare and get parameters in n
 *
 * @tparam S hana struct
 * @param n rclcpp::Node
 * @param[in] name parameter name
 * @param[in, out] parameter default value (in) and current value (out)
 *
 * @return parameter value
 */
template<typename S>
S initParams(rclcpp::Node & n, const std::string & name, S & val)
{
  declareParams<S>(n, name, val);
  getParams<S>(n, name, val);
  return val;
}

}  // namespace cbr

#endif  // CBR_ROS__PARAMETERS_HPP_
