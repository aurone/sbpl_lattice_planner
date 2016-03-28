cmake_minimum_required(VERSION 2.8.3)
project(sbpl_lattice_planner)

##############################################################################
# Find dependencies
##############################################################################

set(THIS_PACKAGE_ROS_DEPS
  roscpp
  costmap_2d
  nav_core
  sbpl_catkin
  pluginlib
  geometry_msgs
  nav_msgs)
find_package(
  catkin REQUIRED COMPONENTS
    ${THIS_PACKAGE_ROS_DEPS}
    message_generation)

include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

##############################################################################
# Generate messages
##############################################################################

add_message_files(FILES SBPLLatticePlannerStats.msg)
generate_messages(DEPENDENCIES geometry_msgs)

##############################################################################
# Define package
##############################################################################

# Note: the following message will be relevant when moving back to system-
# install sbpl
# Since sbpl is found through pkg-config, we need to manually add its libraries
# and include directories into catkin_package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${THIS_PACKAGE_ROS_DEPS} message_runtime
  DEPENDS
)

##############################################################################
# Build
##############################################################################

add_library(${PROJECT_NAME} src/sbpl_lattice_planner.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)

##############################################################################
# Install
##############################################################################

install(
    FILES bgp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
