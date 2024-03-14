cmake_minimum_required(VERSION 3.16)
project(dftpav_rect)
include_directories(
        ${EIGEN3_INCLUDE_DIRS}
        common.h
)
set(CMAKE_CXX_STANDARD 14)

file(GLOB_RECURSE SRCS
        HA/grid_map/*.cpp
        )

add_executable(dftpav_rect main.cpp ${SRCS})
