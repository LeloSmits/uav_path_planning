# UAV Path Planning
## Table of contents

- [Quick start](#quick-start)
- [What's included](#whats-included)
- [Creators](#creators)


## Quick start
Install PX4, Gazebo and ROS with this tutorial: https://docs.px4.io/master/en/simulation/ros_interface.html

Move entire folder into ~/catkin_ws/src. Run "catkin build" from ~/catkin_ws.

There are already a few example launch files in the folder ./launch. You can launch them with 
```text
roslaunch uav_path_planning path_planner_01.launch
```
or some other launch file of your choice.

## What's included

Refer to the ADP-Documentation for detailed information on the ROS-communication-network and how the nodes work with 
each other. Alternatively, you can use the command rqt_graph to create a graph of the communication of nodes with each other.

```text
uav_path_planning/
├── benchmarks/
│   ├── scenarios/
│   │   ├── 2d  # Contains the 2D-worlds and launch files used for evaluation
│   │   └── 3d  # Contains the 3D-worlds and launch files used for evaluation
│   └── classification_benchmark.csv        # Contains the danger index for different obstacle types; used by the benchmark launch files
├── launch/
│   ├── benchmark_launch/
│   │   └── local_planner_sitl_3cam.launch  # Start the PX4 3DVFH Planner, requires the PX4 avoidance package
│   └── launch_files                        # Start various gazebo maps with our path planner
├── msg/
│   ├── MultiArray.msg          # Message type used by the potential_field node
│   ├── obstacleListMsg.msg     # Message type used by the obstacle_sensor and the obstacle_map nodes
│   └── obstacleMsg.msg         # Message type used by the obstacle_sensor and the obstacle_map nodes
├── nodes/                      # This folder contains python files that can be used as a shortcut to start the nodes from the python package in /src
│   ├── global_path_node           
│   ├── local_path_planner_node
│   ├── obstacle_map_node
│   ├── obstacle_sensor_node
│   ├── path_logger_node
│   └── setup_path_logger_for_px4_node
├── src/                                # Contains the python source code
│   └── py_uav_path_planning/           # The python package
│       ├── extras/
│       │   ├── evaluate_log_file.py                    # Plots the logs file
│       │   ├── path_logger.py                          # Logs the UAV position and velocity and the global waypoints for every time step 
│       │   └── setup_path_logger_for_px4_avoidance.py  # Kills the offb_node and starts the path_logger
│       ├── obstacle_map/
│       │   └── obstacle_map.py                         # Starts the obstacle map
│       ├── obstacle_sensor
│       │   ├── obstacle_sensor.py   # Starts the obstacle sensor
│       │   └── helper
│       │       └── read_gazebo_xml.py   # Reads all obstacles in a gazebo world file
│       └── path_planning/
│           ├── calc_apf.py     # Service node for the potential field and gradient field
│           ├── global_path.py  # Node that forwards waypoints from local planner and publishes global waypoints from a file
│           └── local_path_planning.py  # Local planner that uses calc_apf for finding movement direction
└── srv/
    └── potential_field_msg.srv     # Service type used by calc_apf and local_path_planning
```

## Creators

**Leonard Smits**

https://github.com/LeloSmits

**Daniel Piendl**

https://github.com/dpiendl
