cmake_minimum_required(VERSION 2.6)

project(muduo C CXX)


set(CMAKE_CXX_COMPILER "g++")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/)
set(PROJECT_SOURCE_DIR "./")
# 加入一个配置头文件，用于处理 CMake 对源码的设置
configure_file (
  "${PROJECT_SOURCE_DIR}/config.h.cmake"
  "${PROJECT_SOURCE_DIR}/config.h"
  )



