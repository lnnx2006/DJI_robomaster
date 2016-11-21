cmake_minimum_required(VERSION 2.8)
project(Camera)

find_package(catkin REQUIRED)
catkin_package()

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED cv_bridge)
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})

add_executable(Camera src/Camera.cpp src/CompressiveTracker.cpp)
target_link_libraries(Camera ${catkin_LIBRARIES} ${OPENCV_LIBRARIES})
