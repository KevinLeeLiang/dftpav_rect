cmake_minimum_required(VERSION 3.16)
project(dftpav_rect)
include_directories(
        ${EIGEN3_INCLUDE_DIRS}
)
set(CMAKE_CXX_STANDARD 14)
add_executable(dftpav_rect main.cpp common.h)
