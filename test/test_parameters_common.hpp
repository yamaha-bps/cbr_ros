#include <array>
#include <boost/hana/define_struct.hpp>
#include <string>
#include <vector>

struct MySubParams
{
  int i1;
  uint32_t i2;

  bool operator==(const MySubParams & o) const { return i1 == o.i1 && i2 == o.i2; };
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

  std::vector<double> dvec;
  std::vector<std::string> svec;
  std::vector<bool> bvec;
  std::vector<uint8_t> cvec;
  std::vector<int64_t> ivec;

  std::array<int, 3> iarr;
  std::array<MyValue, 2> mvarr;
  MySubParams sub;

  bool operator==(const MyParams & o) const
  {
    return i == o.i && d == o.d && f == o.f && b1 == o.b1 && b2 == o.b2 && dvec == o.dvec
        && svec == o.svec && bvec == o.bvec && cvec == o.cvec && ivec == o.ivec && iarr == o.iarr
        && mvarr == o.mvarr && sub == o.sub;
  }
};

BOOST_HANA_ADAPT_STRUCT(MyParams, i, d, f, b1, b2, dvec, svec, bvec, cvec, ivec, iarr, mvarr, sub);

inline MyParams prm_example{
  /* .i = */ 2,
  /* .d = */ 5.55,
  /* .f = */ -3.14,
  /* .b1 = */ false,
  /* .b2 = */ true,
  /* .dvec = */ {1, 2, 3, 5},
  /* .svec = */ {"hej", "stringvec"},
  /* .bvec = */ {false, true, false, false},
  /* .cvec = */ {uint8_t{0}, uint8_t{26}, uint8_t{101}},
  /* .ivec = */ {1237891, -1241, 123},
  /* .iarr = */ {15, 20, -20},
  /* .mvarr = */
  {
    MyValue{/* .i1 = */ 5, /* .i2 = */ 10},
    MyValue{/* .i1 = */ 25, /* .i2 = */ 35},
  },
  /* .sub = */ MySubParams{/* .i1 = */ 1, /* .i2 = */ 2},
};
