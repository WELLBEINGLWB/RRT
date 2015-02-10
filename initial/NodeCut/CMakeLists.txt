cmake_minimum_required(VERSION 2.8)
ADD_DEFINITIONS(
  -O2
  -Wall
  -g
  -std=c++11
)
project( w )

add_executable( w w.cpp function.cpp )
target_link_libraries( w )
