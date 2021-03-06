cmake_minimum_required(VERSION 3.0.2)
project(uav_path_planning)

find_package(catkin REQUIRED COMPONENTS roscpp rospy geometry_msgs mavros_msgs std_msgs message_generation)

catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files(
    FILES
    obstacleListMsg.msg
    obstacleMsg.msg
)

add_service_files(
    FILES
    potential_field_msg.srv
    )

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs  # Or other packages containing msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(offb_node src/offb_node.cpp)

target_link_libraries(offb_node ${catkin_LIBRARIES})

install(
    PROGRAMS
    nodes/obstacle_map_node
    nodes/obstacle_map_node
    nodes/local_path_planner_node
    nodes/global_path_node
    src/py_uav_path_planning/path_planning/calc_apf.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)