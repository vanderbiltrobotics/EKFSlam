cmake_minimum_required(VERSION 3.9)
project(EKFSlam)

set(CMAKE_CXX_STANDARD 11)

add_executable(EKFSlam main.cpp EKFSlammer.h EKFSlammer.cpp Utils.h Robot.cpp)