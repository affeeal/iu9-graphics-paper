cmake_minimum_required(VERSION 3.15)

set(CMAKE_PROJECT_NAME GraphicsPaper)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)

project(${CMAKE_PROJECT_NAME})

add_subdirectory(lib)

add_executable(${CMAKE_PROJECT_NAME} src/main.cpp)
