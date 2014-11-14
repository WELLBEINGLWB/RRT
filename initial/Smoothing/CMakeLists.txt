cmake_minimum_required(VERSION 2.8)
ADD_DEFINITIONS(
  -Wall
  -g
  -O2
  -std=c++11
)
project( Smoothing )

add_executable( Smoothing main.cpp function.cpp)
target_link_libraries( Smoothing )
