// Copyright Yamaha 2021
// MIT License
// https://github.com/yamaha-bps/cbr_ros/blob/master/LICENSE

#include <boost/hana/define_struct.hpp>
#include <gtest/gtest.h>
#include <rclcpp/node.hpp>
#include <rclcpp/parameter_value.hpp>

#include <array>
#include <string>
#include <vector>

#include "cbr_ros/parameters.hpp"

struct MySubParams
{
  int i1;
  uint32_t i2;

  bool operator==(const MySubParams & o) const { return i1 == o.i1 && i2 == o.i2; }
};

BOOST_HANA_ADAPT_STRUCT(MySubParams, i1, i2);

struct MyValue
{
  int i1, i2;

  bool operator==(const MyValue & o) const { return i1 == o.i1 && i2 == o.i2; }
};

BOOST_HANA_ADAPT_STRUCT(MyValue, i1, i2);

struct MyParams
{
  int i;
  double d;
  float f;
  bool b1;
  bool b2;
  std::string s;

  std::vector<double> dvec;
  std::vector<std::string> svec;
  std::vector<bool> bvec;
  std::vector<uint8_t> cvec;
  std::vector<int64_t> ivec;
  std::vector<int> i32vec;

  std::array<int, 3> iarr;
  std::array<MyValue, 2> mvarr;
  MySubParams sub;

  bool operator==(const MyParams & o) const
  {
    return i == o.i && d == o.d && f == o.f && b1 == o.b1 && b2 == o.b2 && dvec == o.dvec
        && s == o.s && svec == o.svec && bvec == o.bvec && cvec == o.cvec && ivec == o.ivec
        && i32vec == o.i32vec && iarr == o.iarr && mvarr == o.mvarr && sub == o.sub;
  }
};

BOOST_HANA_ADAPT_STRUCT(
  MyParams, i, d, f, b1, b2, s, dvec, svec, bvec, cvec, ivec, i32vec, iarr, mvarr, sub);

inline MyParams prm_example{
  /* .i = */ 2,
  /* .d = */ 5.55,
  /* .f = */ -3.14,
  /* .b1 = */ false,
  /* .b2 = */ true,
  /* .s = */ "hello",
  /* .dvec = */ {1, 2, 3, 5},
  /* .svec = */ {"hej", "stringvec"},
  /* .bvec = */ {false, true, false, false},
  /* .cvec = */ {uint8_t{0}, uint8_t{26}, uint8_t{101}},
  /* .ivec = */ {1237891, -1241, 123},
  /* .i32vec = */ {1231, 1241, -123},
  /* .iarr = */ {15, 20, -20},
  /* .mvarr = */
  {
    MyValue{/* .i1 = */ 5, /* .i2 = */ 10},
    MyValue{/* .i1 = */ 25, /* .i2 = */ 35},
  },
  /* .sub = */ MySubParams{/* .i1 = */ 1, /* .i2 = */ 2},
};

TEST(Prm, Static)
{
  static_assert(std::is_same_v<cbr::detail::ros_type<double>::type, double>);
  static_assert(std::is_same_v<cbr::detail::ros_type<float>::type, double>);
  static_assert(std::is_same_v<cbr::detail::ros_type<bool>::type, bool>);
  static_assert(std::is_same_v<cbr::detail::ros_type<int>::type, int64_t>);
  static_assert(std::is_same_v<cbr::detail::ros_type<std::string>::type, std::string>);
  static_assert(
    std::is_same_v<cbr::detail::ros_type<std::__cxx11::basic_string<char>>::type, std::string>);
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
  uint32_t i;
  std::string s;

  bool operator==(const SubValueT & o) const { return subd == o.subd && s == o.s; }
};

struct ValueT
{
  int64_t i1, i2;
  float f;
  double d;

  SubValueT sub;

  bool operator==(const ValueT & o) const
  {
    return i1 == o.i1 && i2 == o.i2 && f == o.f && d == o.d && sub == o.sub;
  }
};

struct MasterValueT
{
  std::vector<ValueT> vec_of_tpl;
  int i0;

  bool operator==(const MasterValueT & o) const { return vec_of_tpl == o.vec_of_tpl && i0 == o.i0; }
};

BOOST_HANA_ADAPT_STRUCT(SubValueT, subd, i, s);
BOOST_HANA_ADAPT_STRUCT(ValueT, i1, i2, f, d, sub);
BOOST_HANA_ADAPT_STRUCT(MasterValueT, vec_of_tpl, i0);

TEST(Prm, RosVectorOfStructs)
{
  std::vector<ValueT> v;
  v.push_back(ValueT{1, 2, 0.1, 3.12, SubValueT{-1, 100, "str1"}});
  v.push_back(ValueT{3, 4, 0.2, 6.12, SubValueT{-2, 200, "str2"}});
  v.push_back(ValueT{5, 6, 0.3, 9.12, SubValueT{-3, 300, "str3"}});

  MasterValueT mv{v, 5};

  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("node");
  cbr::declareParams(*node, "namespace", mv);

  auto mv_copy = cbr::getParams<MasterValueT>(*node, "namespace");
  ASSERT_EQ(mv, mv_copy);

  mv.vec_of_tpl[2].i1 = 12345;

  cbr::setParams(*node, "namespace", mv);

  auto mv_copy2 = cbr::getParams<MasterValueT>(*node, "namespace");
  ASSERT_EQ(mv, mv_copy2);

  rclcpp::shutdown();
}

TEST(Prm, RosVectorOfStructsInit)
{
  std::vector<ValueT> v;
  v.push_back(ValueT{1, 2, 0.1, 3.12, SubValueT{-1, 100, "str1"}});
  v.push_back(ValueT{3, 4, 0.2, 6.12, SubValueT{-2, 200, "str2"}});
  v.push_back(ValueT{5, 6, 0.3, 9.12, SubValueT{-3, 300, "str3"}});

  MasterValueT mv{v, 5};

  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("node");
  cbr::initParams(*node, "namespace", mv);

  auto prms = node->list_parameters(std::vector<std::string>{}, 10);

  auto f1 = std::find(prms.names.begin(), prms.names.end(), "namespace.i0") != prms.names.end();
  ASSERT_TRUE(f1);
  auto f2 =
    std::find(prms.names.begin(), prms.names.end(), "namespace.vec_of_tpl.i1") != prms.names.end();
  ASSERT_TRUE(f2);
  auto f3 =
    std::find(prms.names.begin(), prms.names.end(), "namespace.vec_of_tpl.i2") != prms.names.end();
  ASSERT_TRUE(f3);
  auto f4 =
    std::find(prms.names.begin(), prms.names.end(), "namespace.vec_of_tpl.f") != prms.names.end();
  ASSERT_TRUE(f4);
  auto f5 =
    std::find(prms.names.begin(), prms.names.end(), "namespace.vec_of_tpl.d") != prms.names.end();
  ASSERT_TRUE(f5);
  auto f6 = std::find(prms.names.begin(), prms.names.end(), "namespace.vec_of_tpl.sub.i")
         != prms.names.end();
  ASSERT_TRUE(f6);
  auto f7 = std::find(prms.names.begin(), prms.names.end(), "namespace.vec_of_tpl.sub.subd")
         != prms.names.end();
  ASSERT_TRUE(f7);
  auto f8 = std::find(prms.names.begin(), prms.names.end(), "namespace.vec_of_tpl.sub.s")
         != prms.names.end();
  ASSERT_TRUE(f8);

  ASSERT_GE(prms.names.size(), 9u);  // some standard params are always there..

  /* for (auto p : prms.names) {
    std::cout << p << std::endl;
    std::cout << node->get_parameter(p).value_to_string() << std::endl;
  } */

  rclcpp::shutdown();
}
