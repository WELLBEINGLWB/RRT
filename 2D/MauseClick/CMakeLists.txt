cmake_minimum_required(VERSION 2.8)
ADD_DEFINITIONS(
  -O2
  -g
)
project( test )

find_package( OpenCV REQUIRED )

add_executable( test test.cpp )
target_link_libraries( test ${OpenCV_LIBS} )
