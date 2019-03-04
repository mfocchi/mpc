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

install(FILES crawl_planner_plugin.xml
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})


if (CATKIN_ENABLE_TESTING)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -o3")

    add_executable(testPlanner tests/unconstrainedQP/testPlanner.cpp src/MPCPlanner.cpp)
    target_link_libraries(testPlanner   ${catkin_LIBRARIES})

    add_executable(testPlannerXY tests/unconstrainedQP_XY/testPlannerXY.cpp src/MPCPlanner.cpp)
    target_link_libraries(testPlannerXY   ${catkin_LIBRARIES})

    add_executable(testPlannerXYconstraintDecoupled tests/constrainedQP_decoupled/testPlannerXYconstraintDecoupled.cpp src/MPCPlanner.cpp)
    target_link_libraries(testPlannerXYconstraintDecoupled   ${catkin_LIBRARIES})

    add_executable(testPlannerXYconstraintCoupled tests/constrainedQP_coupled/testPlannerXYconstraintCoupled.cpp src/MPCPlanner.cpp)
    target_link_libraries(testPlannerXYconstraintCoupled   ${catkin_LIBRARIES})


    add_executable(testPlannerXYconstraintMPC tests/replanning/testPlannerXYconstraintMPC.cpp src/MPCPlanner.cpp)
    target_link_libraries(testPlannerXYconstraintMPC   ${catkin_LIBRARIES})

    add_executable(generateFootSteps tests/niraj/generateFootSteps.cpp src/MPCPlanner.cpp)
    target_link_libraries(generateFootSteps   ${catkin_LIBRARIES})

    install(DIRECTORY tests/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/tests FILES_MATCHING PATTERN "*.m")
    install(TARGETS testPlanner RUNTIME DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/tests/unconstrainedQP/)
    install(TARGETS testPlannerXY RUNTIME DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/tests/unconstrainedQP_XY/)
    install(TARGETS testPlannerXYconstraintDecoupled RUNTIME DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/tests/constrainedQP_decoupled/)
    install(TARGETS testPlannerXYconstraintCoupled RUNTIME DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/tests/constrainedQP_coupled/)
    install(TARGETS testPlannerXYconstraintMPC RUNTIME DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/tests/replanning/)
    install(TARGETS generateFootSteps RUNTIME DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/tests/niraj/)

endif()

