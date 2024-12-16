cmake_minimum_required(VERSION 3.8)
project(hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(urdf REQUIRED)
find_package(robot_math REQUIRED)
add_executable(test_hardware src/test_hardware.cpp)
target_include_directories(test_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(test_hardware PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  test_hardware
  "pluginlib"
  "urdf"
  "robot_math"
)

install(TARGETS test_hardware
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)


ament_export_include_directories(
  include
)



ament_package()
