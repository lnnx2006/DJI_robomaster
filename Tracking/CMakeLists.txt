cmake_minimum_required(VERSION 2.8)
project(Tracking)

find_package(catkin REQUIRED)
catkin_package()

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED cv_bridge)
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})

add_executable(Tracking src/Tracking.cpp src/CompressiveTracker.cpp)
target_link_libraries(Tracking ${catkin_LIBRARIES} ${OPENCV_LIBRARIES})
