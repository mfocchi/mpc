cmake_minimum_required(VERSION 2.8.3)
project(mpc)

find_package(catkin REQUIRED
  math_utils
)


catkin_package(
  INCLUDE_DIRS
  LIBRARIES ${PROJECT_NAME}

)


find_package(PkgConfig)
pkg_check_modules(EIGEN3 eigen3 REQUIRED)

# Setting flags for optimization
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")
set(CMAKE_BUILD_TYPE "Release")

set(INCLUDE_DIRS  ${PROJECT_SOURCE_DIR}/include
                  ${catkin_INCLUDE_DIRS}
                  ${dwl_INCLUDE_DIRS})

               
include_directories(${INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})

## Declare the crawl_planner library
add_library(mpc
            src/MPCPlanner.cpp
            src/ConsoleUtility.cpp)
target_link_libraries(mpc  ${catkin_LIBRARIES} readline)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${HOME}/mpc
  LIBRARY DESTINATION ${HOME}/mpc
  RUNTIME DESTINATION ${HOME}/mpc
  )


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -o3")

add_executable(testPlanner tests/unconstrainedQP/testPlanner.cpp  src/MPCPlanner.cpp src/ConsoleUtility.cpp)
target_link_libraries(testPlanner   ${catkin_LIBRARIES} mpc)


add_executable(testPlannerXY tests/unconstrainedQP_XY/testPlannerXY.cpp src/MPCPlanner.cpp src/ConsoleUtility.cpp )
target_link_libraries(testPlannerXY   ${catkin_LIBRARIES} mpc)


add_executable(testPlannerXYconstraintDecoupled tests/constrainedQP_decoupled/testPlannerXYconstraintDecoupled.cpp src/MPCPlanner.cpp src/ConsoleUtility.cpp)
target_link_libraries(testPlannerXYconstraintDecoupled   ${catkin_LIBRARIES} mpc)

add_executable(testPlannerXYconstraintCoupled tests/constrainedQP_coupled/testPlannerXYconstraintCoupled.cpp src/MPCPlanner.cpp src/ConsoleUtility.cpp)
target_link_libraries(testPlannerXYconstraintCoupled   ${catkin_LIBRARIES} mpc)


add_executable(testPlannerXYconstraintMPC tests/replanning/testPlannerXYconstraintMPC.cpp  src/MPCPlanner.cpp src/ConsoleUtility.cpp )
target_link_libraries(testPlannerXYconstraintMPC   ${catkin_LIBRARIES} mpc)

add_executable(generateFootSteps tests/generateGait/generateFootSteps.cpp   src/MPCPlanner.cpp  src/ConsoleUtility.cpp)
target_link_libraries(generateFootSteps   ${catkin_LIBRARIES} mpc)

message(${CMAKE_CURRENT_SOURCE_DIR})
install(DIRECTORY tests/ DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/tests FILES_MATCHING PATTERN "*.m")
install(TARGETS testPlanner RUNTIME DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/tests/unconstrainedQP/)
install(TARGETS testPlannerXY RUNTIME DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/tests/unconstrainedQP_XY/)
install(TARGETS testPlannerXYconstraintDecoupled RUNTIME DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/tests/constrainedQP_decoupled/)
install(TARGETS testPlannerXYconstraintCoupled RUNTIME DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/tests/constrainedQP_coupled/)
install(TARGETS testPlannerXYconstraintMPC RUNTIME DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/tests/replanning/)
install(TARGETS generateFootSteps RUNTIME DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/tests/generateGait/)

