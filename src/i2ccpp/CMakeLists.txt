cmake_minimum_required(VERSION 3.8)
project(ros2_i2ccpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(TBB REQUIRED)

add_library(ros2_i2ccpp src/ros2_i2ccpp.cpp src/transaction.cpp src/i2c_handler.cpp src/impl/i2c_handler_impl.cpp)
add_library(ros2_i2ccpp::ros2_i2ccpp ALIAS ros2_i2ccpp)

target_link_libraries(ros2_i2ccpp
  PRIVATE
  i2c
  TBB::tbb
)

target_compile_features(ros2_i2ccpp PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(ros2_i2ccpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(ros2_i2ccpp PRIVATE "ROS2_I2CCPP_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS ros2_i2ccpp
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  "include/${PROJECT_NAME}"
)
ament_export_libraries(
  ros2_i2ccpp
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
