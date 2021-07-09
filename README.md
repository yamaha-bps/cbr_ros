# cbr_ros

[![foxy ci][foxy-ci-shield]][foxy-ci-link]
[![galactic ci][galactic-ci-shield]][galactic-ci-link]
[![coverage][coverage-shield]][coverage-link]

Utility headers to make life easier for ROS2 C++ development.

* ```clock_traits_ros```: use ROS clocks with the ```clock_traits``` header from ```cbr_utils```
* ```msg_utils```: conversions between ROS2 interfaces and ```std::chrono``` and ```Eigen``` types
* ```parameters```: declare/read/write ROS2 parameters to and from ```struct```s via ```boost::hana``` introspection


[foxy-ci-shield]: https://img.shields.io/github/workflow/status/yamaha-bps/cbr_ros/foxy/master?label=foxy&style=flat-square
[foxy-ci-link]: https://github.com/yamaha-bps/cbr_ros/actions/workflows/foxy.yml

[galactic-ci-shield]: https://img.shields.io/github/workflow/status/yamaha-bps/cbr_ros/galactic?label=galactic&style=flat-square
[galactic-ci-link]: https://github.com/yamaha-bps/cbr_ros/actions/workflows/galactic.yml

[coverage-shield]: https://img.shields.io/codecov/c/github/yamaha-bps/cbr_ros?style=flat-square&token=ZIVOD9QNRK
[coverage-link]: https://codecov.io/gh/yamaha-bps/cbr_ros

