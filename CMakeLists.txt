cmake_minimum_required(VERSION 3.8)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(algorithms)

add_executable(test_kdtree kdtree.h test_kdtree.cpp)
