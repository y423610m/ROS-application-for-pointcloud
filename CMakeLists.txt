cmake_minimum_required(VERSION 3.0.2)
project(pointpkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES remotepkg
  #CATKIN_DEPENDS roscpp rospy std_msgs
  #DEPENDS system_lib
)

include_directories(
  include
  src
  src/not_used
  ${catkin_INCLUDE_DIRS}
)

##################################################################
###############           CoppeliaSim            #################
##################################################################
add_library(
   extApi_ SHARED
   remoteApi/extApi.c
   remoteApi/extApiPlatform.c
 )
add_definitions(-D_CRT_SECURE_NO_WARNINGS)
add_definitions(-DNON_MATLAB_PARSING)
add_definitions(-DMAX_EXT_API_CONNECTIONS=255)
add_definitions(-DDO_NOT_USE_SHARED_MEMORY)

##################################################################
###############              OpenCV              #################
##################################################################
set(OpenCV_DIR "../opencv/build")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

##################################################################
###############               PCL                #################
##################################################################
set(PCL_DIR "C:/Users/y4236/Documents/pcws/src/pointpkg/PCL1.11.1/cmake")
#set(Boost_DEBUG ON) #コンパイルエラー用
find_package(PCL 1.11 REQUIRED) #すべて
#find_package(PCL 1.3 REQUIRED COMPONENTS common io kdtree geometry search octree) #指定
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
message("## PCL_INCLUDE_DIRS: ${PCL_INCLUDE_DIRS}")
##################################################################
####################   excutables and link   #####################
##################################################################
add_executable(pointer
src/main.cpp
src/operator_side_manager.cpp
src/gui.cpp
src/ros_interface.cpp
src/coppeliasim_interface.cpp
src/pcl.cpp
src/labview_interface.cpp
src/params.cpp

#src/not_used/client.cpp
#src/not_used/manager.cpp


)

set(REALSENSE_LIBS "C:/Program Files (x86)/Intel RealSense SDK 2.0/lib/x64/realsense2.lib")

target_link_libraries(pointer
   ${catkin_LIBRARIES}
   extApi_
   ${OpenCV_LIBS}
   ${PCL_LIBRARIES}
   ${REALSENSE_LIBS}
)

add_executable(tdmaker_for_pc
   src/tdmaker_for_pc.cpp
)

target_link_libraries(tdmaker_for_pc
   ${catkin_LIBRARIES}
)

