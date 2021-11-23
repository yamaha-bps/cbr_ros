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

  bool operator==(const MySubParams & o) const {return i1 == o.i1 && i2 == o.i2;}
};

BOOST_HANA_ADAPT_STRUCT(MySubParams, i1, i2);

struct MyValue
{
  int i1, i2;

  bool operator==(const MyValue & o) const {return i1 == o.i1 && i2 == o.i2;}
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
  std::tuple<double, int> tuple;
  MySubParams sub;

  bool operator==(const MyParams & o) const
  {
    return i == o.i && d == o.d && f == o.f && b1 == o.b1 && b2 == o.b2 && dvec == o.dvec &&
           s == o.s && svec == o.svec && bvec == o.bvec && cvec == o.cvec && ivec == o.ivec &&
           i32vec == o.i32vec && iarr == o.iarr && mvarr == o.mvarr && tuple == o.tuple &&
           sub == o.sub;
  }
};

BOOST_HANA_ADAPT_STRUCT(
  MyParams, i, d, f, b1, b2, s, dvec, svec, bvec, cvec, ivec, i32vec, iarr, mvarr, tuple, sub);

inline MyParams prm_example{
  /* .i = */ 2,
  /* .d = */ 5.55,
  /* .f = */ -3.14f,
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
    MyValue{ /* .i1 = */ 5, /* .i2 = */ 10},
    MyValue{ /* .i1 = */ 25, /* .i2 = */ 35},
  },
  /* .tpl = */ {1, 2},
  /* .sub = */ MySubParams{ /* .i1 = */ 1, /* .i2 = */ 2},
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

TEST(Prm, Update)
{
  MyParams prm = prm_example;

  auto r = cbr::updateParam(rclcpp::Parameter("namespace.i", 4), "namespace", prm);
  ASSERT_TRUE(r);
  ASSERT_EQ(prm.i, 4);

  r = cbr::updateParam(rclcpp::Parameter("namespace.does_not_exist", 4), "namespace", prm);
  ASSERT_FALSE(r);

  r = cbr::updateParam(rclcpp::Parameter("namespace.d", 10.55), "namespace", prm);
  ASSERT_TRUE(r);
  ASSERT_EQ(prm.d, 10.55);

  r = cbr::updateParam(
    rclcpp::Parameter("namespace.dvec", std::vector<double>{-10, -20}), "namespace", prm);
  ASSERT_TRUE(r);
  ASSERT_EQ(prm.dvec.size(), 2);
  ASSERT_EQ(prm.dvec[0], -10);
  ASSERT_EQ(prm.dvec[1], -20);

  r = cbr::updateParam(
    rclcpp::Parameter("namespace.svec", std::vector<std::string>{"new", "par", "ams"}),
    "namespace",
    prm);
  ASSERT_TRUE(r);
  ASSERT_EQ(prm.svec.size(), 3);
  ASSERT_EQ(prm.svec[0], "new");
  ASSERT_EQ(prm.svec[1], "par");
  ASSERT_EQ(prm.svec[2], "ams");

  r = cbr::updateParam(
    rclcpp::Parameter("namespace.i32vec", std::vector<int64_t>{199, 299}), "namespace", prm);
  ASSERT_EQ(prm.i32vec.size(), 2);
  ASSERT_TRUE(r);
  ASSERT_EQ(prm.i32vec[0], 199);
  ASSERT_EQ(prm.i32vec[1], 299);

  r = cbr::updateParam(rclcpp::Parameter("namespace.mvarr_0.i1", 55), "namespace", prm);
  ASSERT_TRUE(r);
  r = cbr::updateParam(rclcpp::Parameter("namespace.mvarr_1.i2", -55), "namespace", prm);
  ASSERT_TRUE(r);
  ASSERT_EQ(prm.mvarr[0].i1, 55);
  ASSERT_EQ(prm.mvarr[1].i2, -55);

  r = cbr::updateParam(rclcpp::Parameter("namespace.tuple_0", 11.5), "namespace", prm);
  ASSERT_TRUE(r);
  r = cbr::updateParam(rclcpp::Parameter("namespace.tuple_1", 22), "namespace", prm);
  ASSERT_TRUE(r);
  r = cbr::updateParam(rclcpp::Parameter("namespace.tuple_2", 33.), "namespace", prm);
  ASSERT_FALSE(r);
  ASSERT_EQ(std::get<0>(prm.tuple), 11.5);
  ASSERT_EQ(std::get<1>(prm.tuple), 22);
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

  bool operator==(const SubValueT & o) const {return subd == o.subd && s == o.s;}
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

  bool operator==(const MasterValueT & o) const {return vec_of_tpl == o.vec_of_tpl && i0 == o.i0;}
};

BOOST_HANA_ADAPT_STRUCT(SubValueT, subd, i, s);
BOOST_HANA_ADAPT_STRUCT(ValueT, i1, i2, f, d, sub);
BOOST_HANA_ADAPT_STRUCT(MasterValueT, vec_of_tpl, i0);

TEST(Prm, UpdateVectorOfStructs)
{
  std::vector<ValueT> v;
  v.push_back(ValueT{1, 2, 0.1f, 3.12, SubValueT{-1, 100, "str1"}});
  v.push_back(ValueT{3, 4, 0.2f, 6.12, SubValueT{-2, 200, "str2"}});
  v.push_back(ValueT{5, 6, 0.3f, 9.12, SubValueT{-3, 300, "str3"}});
  MasterValueT mv{v, 5};

  auto r = cbr::updateParam(
    rclcpp::Parameter("namespace.vec_of_tpl.i3", std::vector<int>{10, 20, 30}), "namespace", mv);
  ASSERT_FALSE(r);

  r = cbr::updateParam(
    rclcpp::Parameter("namespace.vec_of_tpl.i1", std::vector<int>{10, 20, 30}), "namespace", mv);
  ASSERT_TRUE(r);
  ASSERT_EQ(mv.vec_of_tpl.size(), 3);
  ASSERT_EQ(mv.vec_of_tpl[0].i1, 10);
  ASSERT_EQ(mv.vec_of_tpl[1].i1, 20);
  ASSERT_EQ(mv.vec_of_tpl[2].i1, 30);
  ASSERT_EQ(mv.vec_of_tpl[0].i2, 2);
  ASSERT_EQ(mv.vec_of_tpl[1].i2, 4);
  ASSERT_EQ(mv.vec_of_tpl[2].i2, 6);

  r = cbr::updateParam(
    rclcpp::Parameter("namespace.vec_of_tpl.i1", std::vector<int>{40, 50}), "namespace", mv);
  ASSERT_TRUE(r);
  ASSERT_EQ(mv.vec_of_tpl.size(), 2);
  ASSERT_EQ(mv.vec_of_tpl[0].i1, 40);
  ASSERT_EQ(mv.vec_of_tpl[1].i1, 50);
  ASSERT_EQ(mv.vec_of_tpl[0].i2, 2);
  ASSERT_EQ(mv.vec_of_tpl[1].i2, 4);

  // reset
  mv = MasterValueT{v, 5};

  r = cbr::updateParam(
    rclcpp::Parameter(
      "namespace.vec_of_tpl.sub.s", std::vector<std::string>{"new", "par",
        "ams"}), "namespace", mv);
  ASSERT_TRUE(r);
  ASSERT_EQ(mv.vec_of_tpl.size(), 3);
  ASSERT_EQ(mv.vec_of_tpl[0].sub.s, "new");
  ASSERT_EQ(mv.vec_of_tpl[1].sub.s, "par");
  ASSERT_EQ(mv.vec_of_tpl[2].sub.s, "ams");
}

TEST(Prm, RosVectorOfStructs)
{
  std::vector<ValueT> v;
  v.push_back(ValueT{1, 2, 0.1f, 3.12, SubValueT{-1, 100, "str1"}});
  v.push_back(ValueT{3, 4, 0.2f, 6.12, SubValueT{-2, 200, "str2"}});
  v.push_back(ValueT{5, 6, 0.3f, 9.12, SubValueT{-3, 300, "str3"}});

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
  v.push_back(ValueT{1, 2, 0.1f, 3.12, SubValueT{-1, 100, "str1"}});
  v.push_back(ValueT{3, 4, 0.2f, 6.12, SubValueT{-2, 200, "str2"}});
  v.push_back(ValueT{5, 6, 0.3f, 9.12, SubValueT{-3, 300, "str3"}});

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
  auto f6 = std::find(prms.names.begin(), prms.names.end(), "namespace.vec_of_tpl.sub.i") !=
    prms.names.end();
  ASSERT_TRUE(f6);
  auto f7 = std::find(prms.names.begin(), prms.names.end(), "namespace.vec_of_tpl.sub.subd") !=
    prms.names.end();
  ASSERT_TRUE(f7);
  auto f8 = std::find(prms.names.begin(), prms.names.end(), "namespace.vec_of_tpl.sub.s") !=
    prms.names.end();
  ASSERT_TRUE(f8);

  ASSERT_GE(prms.names.size(), 9u);  // some standard params are always there..

  /* for (auto p : prms.names) {
    std::cout << p << std::endl;
    std::cout << node->get_parameter(p).value_to_string() << std::endl;
  } */

  rclcpp::shutdown();
}

struct MyParameters
{
  double d{1};
  int i{2};
  std::string s{"3"};
};

TEST(Prm, Snippets1)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("node");

  MyParameters prm;

  node->declare_parameter<double>("myprm.d", prm.d);
  node->declare_parameter<int>("myprm.i", prm.i);
  node->declare_parameter<std::string>("myprm.s", prm.s);

  prm.d = node->get_parameter("myprm.d").as_double();
  prm.i = static_cast<int>(node->get_parameter("myprm.i").as_int());
  prm.s = node->get_parameter("myprm.s").as_string();

  rclcpp::shutdown();
}

BOOST_HANA_ADAPT_STRUCT(MyParameters, d, i, s);  // option: register just a subset of the members

TEST(Prm, Snippets2)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("node");

  MyParameters prm;
  cbr::declareParams(*node, "myprm", prm);
  cbr::getParams(*node, "myprm", prm);

  // cbr::initParams(*node, "myprm", prm);

  rclcpp::shutdown();
}
