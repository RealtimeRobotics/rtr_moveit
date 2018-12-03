cmake_minimum_required(VERSION 2.8.3)
project(rtr_moveit)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	moveit_core
	moveit_ros_planning
	pcl_ros
	pluginlib
	rosconsole
	roscpp
	roslint
	rtr_core_stub
	)

#find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES moveit_rtr_planner_plugin
  CATKIN_DEPENDS moveit_core pluginlib roscpp 
)

###########
## Build ##
###########

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_library(rtr_planning_context src/rtr_planning_context.cpp)
set_target_properties(rtr_planning_context PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
target_link_libraries(rtr_planning_context ${catkin_LIBRARIES})

add_library(moveit_rtr_planner_plugin src/rtr_planner_manager.cpp)
set_target_properties(moveit_rtr_planner_plugin PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
target_link_libraries(moveit_rtr_planner_plugin ${catkin_LIBRARIES} rtr_planning_context)

install(TARGETS moveit_rtr_planner_plugin rtr_planning_context
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(FILES rtr_interface_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Test for correct C++ source code
roslint_cpp()
