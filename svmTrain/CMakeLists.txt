cmake_minimum_required(VERSION 2.8)
project(svmTrain)

find_package(catkin REQUIRED)
catkin_package()

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED cv_bridge)
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})

add_executable(svmTrain src/svmTrain.cpp src/mySVM.cpp)
target_link_libraries(svmTrain ${catkin_LIBRARIES} ${OPENCV_LIBRARIES})
