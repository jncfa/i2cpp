cmake_minimum_required(VERSION 3.8)
project(i2cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

add_library(i2cpp
  src/i2cpp.cpp)
add_library(i2cpp::i2cpp ALIAS i2cpp)

target_link_libraries(i2cpp
  PRIVATE
  i2c
)

set_target_properties(i2cpp PROPERTIES
  PUBLIC_HEADER "${CMAKE_CURRENT_BINARY_DIR}/i2cpp_export.h"
)

install(
  FILES "${CMAKE_CURRENT_BINARY_DIR}/i2cpp_export.h"
  DESTINATION include/${PROJECT_NAME}
)

include(GenerateExportHeader)
generate_export_header(i2cpp)

target_compile_features(i2cpp PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(i2cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

install(
  DIRECTORY include
  DESTINATION include/${PROJECT_NAME}
)


install(
  TARGETS i2cpp
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
  i2cpp
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
