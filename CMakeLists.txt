cmake_minimum_required(VERSION 3.10)
project(MyProject)
set(CMAKE_CXX_STANDARD 17)
include_directories(include/MyProject)
add_executable(my_program src/main.cpp src/lib.cpp)
add_directory(test)
