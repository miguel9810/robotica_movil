cmake_minimum_required(VERSION 2.8)
project(examen_parcial_2)

set(CMAKE_CXX_FLAGS "-std=c++11 -O3 ${CMAKE_CXX_FLAGS}")

add_executable(build/main_exe 
include/OP3_Arm.hpp
include/A01633021.hpp
src/OP3_Arm.cpp
src/A01633021.cpp 
src/main.cpp
)

add_executable(build/test_exe 
include/OP3_Arm.hpp
include/A01633021.hpp
src/OP3_Arm.cpp
src/A01633021.cpp 
tests/test.cpp
)

