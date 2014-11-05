cmake_minimum_required(VERSION 2.8)
ADD_DEFINITIONS(
  -O2
  -g
  -std=c++11
)
project( rrt )

add_executable( rrt main.cpp MotionPlan.cpp )
target_link_libraries( rrt )
