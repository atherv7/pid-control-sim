cmake_minimum_required(VERSION 3.20)
project(pid_drone_sim
  VERSION 0.0.0
  LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

add_library(pid_lib STATIC
  src/pid.cpp
)

target_include_directories(pid_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

option(BUILD_TESTS "Build unit tests" ON)

if(BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
