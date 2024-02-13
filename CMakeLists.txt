cmake_minimum_required(VERSION 3.15)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_PROJECT_NAME graphics_paper)

project(${CMAKE_PROJECT_NAME})

find_package(GTest REQUIRED)
find_package(Boost REQUIRED)

enable_testing()

add_subdirectory(lib)

add_executable(${CMAKE_PROJECT_NAME} src/main.cpp)
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE bezier)
