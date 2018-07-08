cmake_minimum_required(VERSION 2.8)

project(norm_esti)

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV 2.4.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable (${PROJECT_NAME} main.cpp) #注意这里不能替换

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
