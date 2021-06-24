// Copyright Yamaha 2021
// MIT License
// https://github.com/yamaha-bps/cbr_ros/blob/master/LICENSE


#include <gtest/gtest.h>

#include <rclcpp/node.hpp>

#include "cbr_ros/parameters.hpp"

#include "test_parameters_common.hpp"

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

/* struct SubValueT
{
  double subd;

  auto operator<=>(const SubValueT &) const = default;
};

struct ValueT
{
  int64_t i1, i2;
  double  d;

  SubValueT sub;

  auto operator<=>(const ValueT &) const = default;
};

BOOST_HANA_ADAPT_STRUCT(SubValueT, subd);
BOOST_HANA_ADAPT_STRUCT(ValueT, i1, i2, d, sub); */

/* TEST(Prm, RosVectorOfStructs) {
  std::vector<ValueT> v;
  v.push_back(ValueT{-5, 5, 3.12, SubValueT{1.2}});
  v.push_back(ValueT{-15, 15, 6.12, SubValueT{1.2}});

  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("node");
  cbr::declare_range(*node, "namespace", v);

  auto prms = node->list_parameters(std::vector<std::string>{}, 5);
  for (auto name : prms.names) { std::cout << name << std::endl; }

  auto x = node->get_parameter("namespace.i2").as_integer_array();
  for (auto i : x) std::cout << i << std::endl;

  auto x2 = node->get_parameter("namespace.d").as_double_array();
  for (auto i : x2) std::cout << i << std::endl;

  std::vector<ValueT> v_copy;
  v_copy.resize(2);
  cbr::get_range(*node, "namespace", v_copy);

  ASSERT_EQ(v, v_copy);

  rclcpp::shutdown();
} */
