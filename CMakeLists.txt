cmake_minimum_required(VERSION 3.15)

set(CMAKE_PROJECT_NAME graphics_paper)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)

project(${CMAKE_PROJECT_NAME})

option(BUILD_TESTS "build tests" OFF)
if(BUILD_TESTS)
  find_package(GTest REQUIRED)
  enable_testing()
endif(BUILD_TESTS)

add_subdirectory(lib)

add_executable(${CMAKE_PROJECT_NAME} src/main.cpp)
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE bezier)
