cmake_minimum_required(VERSION 3.5)

project(ackermann_mux)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)

include_directories(include)

set(
  DEPENDENCIES
  "rclcpp"
  "std_msgs"
  "ackermann_msgs"
  "visualization_msgs"
  "diagnostic_updater"
)

add_executable(ackermann_mux
  src/ackermann_mux.cpp
  src/ackermann_mux_node.cpp
  src/ackermann_mux_diagnostics.cpp
)
ament_target_dependencies(ackermann_mux ${DEPENDENCIES})

add_executable(ackermann_marker
  src/ackermann_marker.cpp
)
ament_target_dependencies(ackermann_marker ${DEPENDENCIES})

install(
  TARGETS ackermann_mux ackermann_marker
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

foreach(dir launch config)
  install(DIRECTORY ${dir}/
    DESTINATION share/${PROJECT_NAME}/${dir})
endforeach()

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake)
endif()

ament_export_include_directories(include)

ament_export_dependencies(${DEPENDENCIES})

ament_package()
