cmake_minimum_required(VERSION 2.8.3)
project(crawl_planner)

find_package(catkin REQUIRED
  dls_planner
  dls_controller
  math_utils
  crawl_controller
  terrain_estimator
  dls_map_interface
  locomotion_utils)

catkin_package(
  INCLUDE_DIRS 
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp dls_planner dwl_msgs dls_map_interface locomotion_utils
)

# Setting flags for optimization
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")
set(CMAKE_BUILD_TYPE "Release")

set(INCLUDE_DIRS  ${PROJECT_SOURCE_DIR}/include
                  ${catkin_INCLUDE_DIRS}
                  ${dwl_INCLUDE_DIRS})
               
include_directories(${INCLUDE_DIRS})

## Declare the crawl_planner library
add_library(crawl_planner
            src/CrawlPlanner.cpp
            src/loggingFeatures.cpp
            src/MPCPlanner.cpp)
target_link_libraries(crawl_planner  ${catkin_LIBRARIES} readline)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

add_executable(testPlanner tests/unconstrainedQP/testPlanner.cpp src/MPCPlanner.cpp)
target_link_libraries(testPlanner   ${catkin_LIBRARIES})
set_target_properties(testPlanner PROPERTIES COMPILE_FLAGS "-std=c++0x -o3" RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/tests/unconstrainedQP")

add_executable(testPlannerXY tests/unconstrainedQP_XY/testPlannerXY.cpp src/MPCPlanner.cpp)
target_link_libraries(testPlannerXY   ${catkin_LIBRARIES})
set_target_properties(testPlannerXY PROPERTIES COMPILE_FLAGS "-std=c++0x -o3" RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/tests/unconstrainedQP_XY")

add_executable(testPlannerXYconstraintDecoupled tests/constrainedQP_decoupled/testPlannerXYconstraintDecoupled.cpp src/MPCPlanner.cpp)
target_link_libraries(testPlannerXYconstraintDecoupled   ${catkin_LIBRARIES})
set_target_properties(testPlannerXYconstraintDecoupled PROPERTIES COMPILE_FLAGS "-std=c++0x -o3" RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/tests/constrainedQP_decoupled")

add_executable(testPlannerXYconstraintCoupled tests/constrainedQP_coupled/testPlannerXYconstraintCoupled.cpp src/MPCPlanner.cpp)
target_link_libraries(testPlannerXYconstraintCoupled   ${catkin_LIBRARIES})
set_target_properties(testPlannerXYconstraintCoupled PROPERTIES COMPILE_FLAGS "-std=c++0x -o3" RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/tests/constrainedQP_coupled")


add_executable(testPlannerXYconstraintMPC tests/replanning/testPlannerXYconstraintMPC.cpp src/MPCPlanner.cpp)
target_link_libraries(testPlannerXYconstraintMPC   ${catkin_LIBRARIES})
set_target_properties(testPlannerXYconstraintMPC PROPERTIES COMPILE_FLAGS "-std=c++0x -o3" RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/tests/replanning")
