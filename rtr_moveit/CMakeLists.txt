cmake_minimum_required(VERSION 2.8.3)
project(rtr_moveit)

# C++ 11
add_compile_options(-std=c++11)

# Warnings
add_definitions(-W -Wall -Wextra
  -Wwrite-strings -Wunreachable-code -Wpointer-arith
  -Winit-self -Wredundant-decls
  -Wno-unused-parameter -Wno-unused-function)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning
  pcl_ros
  pluginlib
  rosconsole
  roscpp
  roslint
  rosparam_shortcuts
  rtr-api
  rtr-core
  rtr-occupancy
  tf
  trajectory_msgs
)

# System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)

###################################
## Catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    moveit_core
    pluginlib
    roscpp
    rtr-api
    rtr-core
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  DEPENDS
    EIGEN3
)

###########
## Build ##
###########

# Specify additional locations of header files
# Your package locations should be listed before other locations
include_directories(
  SYSTEM
    ${EIGEN3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

include_directories(
  include
)

# Declare a C++ library with project namespace to avoid naming collision
add_library(
  ${PROJECT_NAME}
  src/occupancy_handler.cpp
  src/rtr_planner_interface.cpp
  src/rtr_planning_context.cpp
  src/roadmap_visualization.cpp
)

# Specify libraries to link a library or executable target against
target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# Declare a C++ library with project namespace to avoid naming collision
add_library(
  ${PROJECT_NAME}_plugin
  src/rtr_planner_manager.cpp
)

# Specify libraries to link a library or executable target against
target_link_libraries(
  ${PROJECT_NAME}_plugin
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS
  ${PROJECT_NAME} ${PROJECT_NAME}_plugin
  ARCHIVE DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION
    ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#Mark plugin description file for installation
install(
  FILES
    ${PROJECT_NAME}_plugin_description.xml
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(rtr_conversions_test
    test/rtr_conversions.test
    test/rtr_conversions_test.cpp)
  target_link_libraries(rtr_conversions_test
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )

  if(NOT CATKIN_DISABLE_HARDWARE_TEST)
    add_rostest_gtest(rapidplan_test
      test/rapidplan.test
      test/rapidplan_test.cpp)
    target_link_libraries(rapidplan_test
      ${PROJECT_NAME}
      ${catkin_LIBRARIES}
    )
  endif()

endif()

# Test for correct C++ source code
roslint_cpp()
