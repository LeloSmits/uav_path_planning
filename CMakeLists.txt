cmake_minimum_required(VERSION 3.0.2)
project(uav_path_planning)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs mavros_msgs octomap_msgs std_msgs message_generation)

catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files(
    FILES
    obstacleListMsg.msg
    obstacleMsg.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs  # Or other packages containing msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS})

install(
    PROGRAMS
    nodes/obstacle_map_node nodes/obstacle_map_node nodes/path_planning_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
