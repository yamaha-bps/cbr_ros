cmake_minimum_required(VERSION 3.15)
project(cbr_ros VERSION 0.1.0)

include(CTest)
include(GoogleTest)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# uninstall target has to be before rclcpp otherwise ament exports its own uninstall target
add_custom_target(
  uninstall
  COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/uninstall.cmake
)

find_package(rclcpp REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(cbr_utils REQUIRED)

# ---------------------------------------------------------------------------------------
# TARGETS
# ---------------------------------------------------------------------------------------

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME}
INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}
INTERFACE
  rclcpp::rclcpp
  Eigen3::Eigen
  cbr_utils::cbr_utils
)

# ---------------------------------------------------------------------------------------
# INSTALLATION
# ---------------------------------------------------------------------------------------

include(GNUInstallDirs)

# Header files
install(
  DIRECTORY    ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
  DESTINATION  ${CMAKE_INSTALL_INCLUDEDIR}
)

# Targets
install(
  TARGETS               ${PROJECT_NAME}
  EXPORT                ${PROJECT_NAME}_targets
  LIBRARY DESTINATION   ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION   ${CMAKE_INSTALL_LIBDIR}
)

# ---------------------------------------------------------------------------------------
# CMAKE CONFIGURATION
# ---------------------------------------------------------------------------------------

set(export_dest_dir "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}")

# Generate TARGETS files
install(
  EXPORT       ${PROJECT_NAME}_targets
  FILE         ${PROJECT_NAME}Targets.cmake
  NAMESPACE    ${PROJECT_NAME}::
  DESTINATION  ${export_dest_dir}
)

# Generate CONFIG file
include(CMakePackageConfigHelpers)
configure_package_config_file(
  cmake/ConfigTemplate.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION ${export_dest_dir}
)

# Generate VERSION file
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Version.cmake
  VERSION ${PACKAGE_VERSION}
  COMPATIBILITY SameMajorVersion
)

# Install CONFIG and VERSION files
install(
  FILES        ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
               ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Version.cmake
  DESTINATION  ${export_dest_dir}
)

# Generate uninstall file
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/uninstall.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/uninstall.cmake
  IMMEDIATE @ONLY
)

# ---------------------------------------------------------------------------------------
# TESTING
# ---------------------------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(GTest REQUIRED)

  add_compile_options(-Wall -Wextra -Wpedantic -Werror)

  # Utils
  add_executable(${PROJECT_NAME}_msg_utils test/test_msg_utils.cpp)
  target_link_libraries(${PROJECT_NAME}_msg_utils PRIVATE ${PROJECT_NAME} GTest::Main)
  gtest_discover_tests(${PROJECT_NAME}_msg_utils)

  # Parameters
  add_executable(${PROJECT_NAME}_test_parameters test/test_parameters.cpp)
  target_link_libraries(${PROJECT_NAME}_test_parameters PRIVATE ${PROJECT_NAME} GTest::Main)
  gtest_discover_tests(${PROJECT_NAME}_test_parameters)
endif()

