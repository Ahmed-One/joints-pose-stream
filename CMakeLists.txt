cmake_minimum_required(VERSION 3.20)
project(joints_stream)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(joints_stream_node
  src/main.cpp
  src/drive.cpp
  src/trajectory.cpp
)
target_compile_features(joints_stream_node PRIVATE cxx_std_17)
target_include_directories(joints_stream_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_include_directories(joints_stream_node SYSTEM PRIVATE
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
)
target_link_libraries(joints_stream_node PRIVATE
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
)

install(TARGETS joints_stream_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(PROGRAMS
  scripts/tester.py
  scripts/tester2.py
  scripts/visualizer.py
  scripts/visualizer2.py
  DESTINATION lib/${PROJECT_NAME}
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

ament_package()
