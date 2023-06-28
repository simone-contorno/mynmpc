cmake_minimum_required(VERSION 3.10)
project(mynmpc)

# C++ version
set(CMAKE_CXX_STANDARD 17)

# Build
set(CMAKE_BUILD_TYPE "Release")

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14) # std::clamp does not work with this version and must be substituted 
endif()

# Compilers warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Suppress warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w") 

# Packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(proxsuite REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(urdf REQUIRED)

# Executables
add_executable(controller
  src/controller.cpp 
  src/model.cpp 
  src/mpc.cpp 
  src/proxqp.cpp 
  src/utils.cpp
)

add_executable(obstacle1
  src/obstacles/obstacle1.cpp 
  src/model.cpp 
  src/mpc.cpp 
  src/proxqp.cpp 
  src/utils.cpp
)

add_executable(obstacle2
  src/obstacles/obstacle2.cpp 
  src/model.cpp 
  src/mpc.cpp 
  src/proxqp.cpp 
  src/utils.cpp
)

# Directories
target_include_directories (controller PRIVATE include ${EIGEN3_INCLUDE_DIRS})
target_include_directories (obstacle1 PRIVATE include ${EIGEN3_INCLUDE_DIRS})
target_include_directories (obstacle2 PRIVATE include ${EIGEN3_INCLUDE_DIRS})

# Dependencies

ament_target_dependencies(controller 
  rclcpp 
  std_msgs
  geometry_msgs
  sensor_msgs 
  nav_msgs 
  urdf
)

ament_target_dependencies(obstacle1 
  rclcpp 
  std_msgs
  geometry_msgs
  sensor_msgs 
  nav_msgs 
)

ament_target_dependencies(obstacle2 
  rclcpp 
  std_msgs
  geometry_msgs
  sensor_msgs 
  nav_msgs 
)

# Install 

install(TARGETS 
  controller
  obstacle1
  obstacle2
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()