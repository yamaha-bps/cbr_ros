# cbr_ros

[![foxy ci][foxy-ci-shield]][foxy-ci-link]
[![galactic ci][galactic-ci-shield]][galactic-ci-link]
[![coverage][coverage-shield]][coverage-link]

Utility headers to make life easier for ROS2 C++ development.

* ```clock_traits_ros```: use ROS clocks with the ```clock_traits``` header from ```cbr_utils```
* ```msg_utils```: conversions between ROS2 interfaces and ```std::chrono``` and ```Eigen``` types
* ```parameters```: declare/read/write nested data structures as ROS2 parameters via ```boost::hana``` introspection

## Parameters

Including `cbr_ros/parameter.hpp` makes it possible to declare whole data structures as ROS parameters.

### Supported data format

Handle nested data structures of

* Regular ROS parameter types
* `std::array`
* `std::tuple`
* `boost::hana`-registered `struct` 
* `std::vector`, with the exception that multiple levels of `std::vector` is not supported


### Example

For example, consider the following C++ struct
```cpp
struct MyParameters
{
  double d{1};
  int i{2};
  std::string s{"3"};
};
```
Using the [regular API](https://docs.ros2.org/foxy/api/rclcpp/classrclcpp_1_1Node.html) would require three calls each to declare and read those parameters.
```cpp
MyParameters prm{};

node->declare_parameter<double>("myprm.d", prm.d);
node->declare_parameter<int>("myprm.i", prm.i);
node->declare_parameter<std::string>("myprm.s", prm.s);

prm.d = node->get_parameter("myprm.d").as_double();
prm.i = node->get_parameter("myprm.i").as_int();
prm.s = node->get_parameter("myprm.s").as_string();
```
This becomes very verbose for large, nested structures with many parameters.

As an alternative, `declareParams` in `cbr_ros/parameter.hpp` makes it possible to instead deal with the complete structure. The caveat is that it is necessary to register the structure with `boost::hana` to enable data structure introspection.
```cpp
#include <boost/hana/define_struct.hpp>

BOOST_HANA_ADAPT_STRUCT(MyParameters, d, i, s);  // option: register just a subset of the members
```
With introspection enabled parameter handling becomes much more succinct:
```cpp
#include <cbr_ros/parameter.hpp>

MyParameters prm{};

cbr::declareParams(*node, "myprm", prm);
cbr::getParams(*node, "myprm", prm);
```
Even shorter is ```cbr::initParams(*node, "myprm", prm)``` which both declares and reads values.

### Nested structures

See `test/test_parameters.cpp` for more complex examples.

### Shortcomings

There is no support in these methods for declaring parameter descriptors.


<!-- MARKDOWN LINKS AND IMAGES -->

[foxy-ci-shield]: https://img.shields.io/github/workflow/status/yamaha-bps/cbr_ros/foxy/master?label=foxy&style=flat-square
[foxy-ci-link]: https://github.com/yamaha-bps/cbr_ros/actions/workflows/foxy.yml

[galactic-ci-shield]: https://img.shields.io/github/workflow/status/yamaha-bps/cbr_ros/galactic?label=galactic&style=flat-square
[galactic-ci-link]: https://github.com/yamaha-bps/cbr_ros/actions/workflows/galactic.yml

[coverage-shield]: https://img.shields.io/codecov/c/github/yamaha-bps/cbr_ros?style=flat-square&token=ZIVOD9QNRK
[coverage-link]: https://codecov.io/gh/yamaha-bps/cbr_ros

