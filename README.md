# cbr_ros

[![foxy](https://github.com/yamaha-bps/cbr_ros/actions/workflows/foxy.yaml/badge.svg)](https://github.com/yamaha-bps/cbr_ros/actions/workflows/foxy.yaml) [![galactic](https://github.com/yamaha-bps/cbr_ros/actions/workflows/galactic.yaml/badge.svg)](https://github.com/yamaha-bps/cbr_ros/actions/workflows/galactic.yaml)

Utility headers to make life easier for ROS2 C++ development.

* ```clock_traits_ros```: use ROS clocks with the ```clock_traits``` header from ```cbr_utils```
* ```msg_utils```: conversions between ROS2 interfaces and ```std::chrono``` and ```Eigen``` types
* ```parameters```: declare/read/write ROS2 parameters to and from ```struct```s via ```boost::hana``` introspection
