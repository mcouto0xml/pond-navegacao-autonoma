cmake_minimum_required(VERSION 3.8)
project(my_part1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---------------------------------------------------------
# Dependências
# ---------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cg_interfaces REQUIRED)

# ---------------------------------------------------------
# Executável do cliente
# ---------------------------------------------------------
add_executable(get_map_client
  src/main.cpp
  src/get_map_client.cpp
  src/algorithm.cpp
)

ament_target_dependencies(
  get_map_client
  rclcpp
  cg_interfaces
)

# ---------------------------------------------------------
# Instalação do executável
# ---------------------------------------------------------
install(
  TARGETS get_map_client
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
