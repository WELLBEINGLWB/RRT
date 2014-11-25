cmake_minimum_required(VERSION 2.8)
ADD_DEFINITIONS(
  -O2
  -g
)
project( rrt )

add_executable( rrt main.cpp MotionPlan.cpp Bspline.cpp )
target_link_libraries( rrt )
