// Copyright Yamaha 2021
// MIT License
// https://github.com/yamaha-bps/cbr_ros/blob/master/LICENSE

#include <gtest/gtest.h>

#include <rclcpp/node.hpp>
#include <rclcpp/parameter_value.hpp>

#include "cbr_ros/parameters.hpp"

#include "test_parameters_common.hpp"

TEST(Prm, Static)
{
  static_assert(std::is_same_v<cbr::detail::ros_type<double>::type, double>);
  static_assert(std::is_same_v<cbr::detail::ros_type<float>::type, double>);
  static_assert(std::is_same_v<cbr::detail::ros_type<bool>::type, bool>);
  static_assert(std::is_same_v<cbr::detail::ros_type<int>::type, int64_t>);
}

TEST(Prm, RosBasic)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("asdf");

  // declare
  cbr::declareParams(*node, "namespace", prm_example);

  ASSERT_TRUE(node->has_parameter("namespace.dvec"));
  ASSERT_TRUE(node->has_parameter("namespace.i"));
  ASSERT_TRUE(node->has_parameter("namespace.d"));
  ASSERT_TRUE(node->has_parameter("namespace.sub.i1"));
  ASSERT_TRUE(node->has_parameter("namespace.sub.i2"));

  // get
  auto prm_copy = cbr::getParams<MyParams>(*node, "namespace");
  ASSERT_EQ(prm_example, prm_copy);

  // set
  prm_copy.i = 3;
  prm_copy.d = 7.5;
  cbr::setParams(*node, "namespace", prm_copy);
  ASSERT_EQ(node->get_parameter("namespace.i").as_int(), 3.);
  ASSERT_EQ(node->get_parameter("namespace.d").as_double(), 7.5);

  rclcpp::shutdown();
}

struct SubValueT
{
  double subd;

  bool operator==(const SubValueT & o) const { return subd == o.subd; }
};

struct ValueT
{
  int64_t i1, i2;
  float f;
  double d;

  // SubValueT sub;

  bool operator==(const ValueT & o) const
  {
    return i1 == o.i1 && i2 == o.i2 && f == o.f && d == o.d; // && sub == o.sub;
  }
};

BOOST_HANA_ADAPT_STRUCT(SubValueT, subd);
BOOST_HANA_ADAPT_STRUCT(ValueT, i1, i2, f, d);

TEST(Prm, RosVectorOfStructs) {
  std::vector<ValueT> v;
  v.push_back(ValueT{1, 2, 0.1, 3.12});
  v.push_back(ValueT{3, 4, 0.2, 6.12});
  v.push_back(ValueT{5, 6, 0.3, 9.12});

  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("node");
  cbr::declare_range(*node, "namespace", v.begin(), v.end());

  auto prms = node->list_parameters(std::vector<std::string>{}, 5);
  for (auto name : prms.names) { std::cout << name << std::endl; }

  auto I1v = node->get_parameter("namespace.i1").as_integer_array();
  for (auto i : I1v) std::cout << i << " ";
  std::cout << std::endl;

  auto I2v = node->get_parameter("namespace.i2").as_integer_array();
  for (auto i : I2v) std::cout << i << " ";
  std::cout << std::endl;

  auto Fv = node->get_parameter("namespace.f").as_double_array();
  for (auto i : Fv) std::cout << i  << " ";
  std::cout << std::endl;

  auto Dv = node->get_parameter("namespace.d").as_double_array();
  for (auto i : Dv) std::cout << i  << " ";
  std::cout << std::endl;

  /* std::vector<ValueT> v_copy;
  v_copy.resize(3);
  cbr::get_range(*node, "namespace", v_copy.begin(), v_copy.end());

  std::cout << v_copy[0].i1;
  std::cout << v_copy[0].i2;
  std::cout << v_copy[0].d;

  std::cout << v_copy[1].i1;
  std::cout << v_copy[1].i2;
  std::cout << v_copy[1].d;

  ASSERT_EQ(v, v_copy); */
  rclcpp::shutdown();
}
