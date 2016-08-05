cmake_minimum_required(VERSION 2.6)
project(Topview_proj)


find_package(OpenCV REQUIRED core imgproc highgui)

include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(Topview_proj src/topview.cpp src/IPM.cpp src/IPM.h)
target_link_libraries(Topview_proj ${OpenCV_LIBS})