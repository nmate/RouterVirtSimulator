cmake_minimum_required(VERSION 2.8)
project(RouterVirtSimulator)

# C++11
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")

add_subdirectory(src build/src)
if (WITH_TEST)
 add_subdirectory(test build/test)
endif()