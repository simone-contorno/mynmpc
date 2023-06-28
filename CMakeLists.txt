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
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w") 

# Packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(proxsuite REQUIRED)

ament_package()