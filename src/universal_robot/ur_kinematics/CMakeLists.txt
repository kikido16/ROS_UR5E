cmake_minimum_required(VERSION 3.0.2)
project(ur_kinematics)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs moveit_core moveit_kinematics
  moveit_ros_planning pluginlib tf_conversions)

find_package(Boost REQUIRED COMPONENTS system)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ur3_kin ur5_kin ur10_kin ur3_moveit_plugin ur5_moveit_plugin ur10_moveit_plugin ur3e_kin ur5e_kin ur10e_kin ur16e_kin ur20_kin ur20_moveit_plugin
  CATKIN_DEPENDS roscpp geometry_msgs moveit_core moveit_kinematics moveit_ros_planning
    pluginlib tf_conversions
  DEPENDS Boost
)


###########
## Build ##
###########

include_directories(SYSTEM ${Boost_INCLUDE_DIR})
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(ur5e_kin src/ur_kin.cpp)
set_target_properties(ur5e_kin PROPERTIES COMPILE_DEFINITIONS "UR5e_PARAMS")

add_library(ur5e_moveit_plugin src/ur_moveit_plugin.cpp)
set_target_properties(ur5e_moveit_plugin PROPERTIES COMPILE_DEFINITIONS "UR5e_PARAMS")
target_link_libraries(ur5e_moveit_plugin
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ur5e_kin)


#############
## Install ##
#############

install(TARGETS
    ur5e_kin
    ur5e_moveit_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# install moveit plugin description file
install(FILES ur_moveit_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
