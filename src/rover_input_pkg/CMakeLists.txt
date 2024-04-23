cmake_minimum_required(VERSION 3.8)
project(rover_input_pkg)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Curses REQUIRED)

add_executable(input_node_v2 src/input_node_v2.cpp)
ament_target_dependencies(input_node_v2 rclcpp std_msgs geometry_msgs)

target_link_libraries(input_node_v2
  ${CURSES_LIBRARIES}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  input_node_v2
  DESTINATION lib/${PROJECT_NAME})

ament_package()
