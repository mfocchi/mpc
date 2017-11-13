cmake_minimum_required(VERSION 2.8.3)
project(crawl_planner)

find_package(catkin REQUIRED
  roscpp
  dls_planner
  dls_controller
  pluginlib
  hyq2max_ik
  hyq_ik
  math_utils
  crawl_controller
  terrain_estimator)

catkin_package(
  INCLUDE_DIRS 
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp dls_planner dwl_msgs
  DEPENDS system_lib
)

#just set robot name here
set(ROBOT_NAME "HyQ") 
#this is to find robot.h for liblocomotionutils
include_directories(/usr/local/include/iit/robots/hyq2max)
include_directories(/usr/local/include/iit/robots/hyq)
string(TOLOWER ${ROBOT_NAME} ROBOT_LOCATION)
set(LOC_LIB_NAME "locomotionutils${ROBOT_LOCATION}")
add_definitions(-DLOC_LIB_NAME=${LOC_LIB_NAME})
add_definitions(-DROBOT_LOCATION=${ROBOT_LOCATION})
add_definitions(-DROBOT=${ROBOT_NAME})
if( ROBOT_NAME STREQUAL "HyQ2Max" )
  add_definitions(-DUSEHYQ2MAX=1)
endif()
if( ROBOT_NAME STREQUAL "HyQ" )
  add_definitions(-DUSEHYQ=1)
endif()

set(INCLUDE_DIRS  ${PROJECT_SOURCE_DIR}/include
                  ${catkin_INCLUDE_DIRS}
                  ${dwl_INCLUDE_DIRS})
               
include_directories(${INCLUDE_DIRS})

## Declare the crawl_planner library
add_library(crawl_planner src/CrawlPlanner.cpp  src/loggingFeatures.cpp)
target_link_libraries(crawl_planner  ${catkin_LIBRARIES})
set_target_properties(crawl_planner PROPERTIES COMPILE_FLAGS "-std=c++0x -o3")     
