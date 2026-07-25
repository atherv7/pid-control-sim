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
  src/dynamics.cpp
)

target_include_directories(pid_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

option(BUILD_TESTS "Build unit tests" ON)

if(BUILD_TESTS)
  enable_testing()

  include(FetchContent)
  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.zip
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  )
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)

  add_executable(pid_tests
    tests/pid_test.cpp
    tests/dynamics_test.cpp
  )

  target_link_libraries(pid_tests PRIVATE
    pid_lib
    GTest::gtest_main
  )

  include(GoogleTest)
  gtest_discover_tests(pid_tests)
endif()
