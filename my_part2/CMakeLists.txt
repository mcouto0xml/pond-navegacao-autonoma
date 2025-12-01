cmake_minimum_required(VERSION 3.8)
project(my_part2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cg_interfaces REQUIRED)

add_executable(maze_mapper
  src/main.cpp
  src/sensor_handler.cpp
  src/dfs_mapper.cpp
)

ament_target_dependencies(
  maze_mapper
  rclcpp
  cg_interfaces
)

install(TARGETS
  maze_mapper
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
