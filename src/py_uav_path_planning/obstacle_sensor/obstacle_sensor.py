#!/usr/bin/env python2

import math
import numpy as np
import rospy
import typing

from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R
from uav_path_planning.msg import obstacleMsg, obstacleListMsg

from helper import Obstacle, read_gazebo_xml


# ToDo: Implement a method that checks if the world from the xml file is the same as currently running in gazebo
# ToDo: Check if Class Obstacle is needed or everything can be done with obstacleMsg
class ObstacleSensor(object):
    def __init__(self):
        self.name = 'obstacle_sensor'
        rospy.init_node(self.name)  # ToDo: In all rospy.loginfo replace obstacle_sensor with self.name

        self.loop_rate = rospy.Rate(1)
        self.all_obstacles = list()  # type: typing.List[Obstacle]
        self.active_obstacles = list()  # type: typing.List[Obstacle]
        self.uav_pose = None
        self.range = 20
        self.angle = 45  # Added angle to only include obstacles within field of view

        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._pose_callback)
        self.pub_obstacle_list = rospy.Publisher("~/active_obstacles", obstacleListMsg, queue_size=100)  # ToDo: find out if queue_size is important

        rospy.loginfo("obstacle_sensor: Node initialized")

    def get_distance_uav_obstacle(self, obstacle):
        # type: (Obstacle) -> float
        """Returns the euclidean distance between the uav and the obstacle.
        :arg obstacle: Obstacle to measure the distance to."""

        assert isinstance(obstacle, Obstacle)
        uav_pos = np.array(
            [self.uav_pose.pose.position.x, self.uav_pose.pose.position.y, self.uav_pose.pose.position.z])
        obs_pos = np.array(obstacle.pose[:3])
        return np.linalg.norm((uav_pos, obs_pos))

    # New function to identify angle between obstacle and drone"
    def get_angle_uav_obstacles(self, obstacle):
        # ToDo @Leo
        # return 0
        assert isinstance(obstacle, Obstacle)
        sp = np.array([1, 0, 0])  # vector of the sensor in the UAV coordinate frame
        uav_quat = np.array([self.uav_pose.pose.orientation.x, self.uav_pose.pose.orientation.y,
                             self.uav_pose.pose.orientation.z, self.uav_pose.pose.orientation.w])
        rot_matrix = R.from_quat(uav_quat)  # calculates UAV rotation matrix
        cu_dv = rot_matrix.dot(sp)  # calculates current direction vector of UAV
        # Build vector between vector and current obstacle
        uav_pos = np.array([self.uav_pose.pose.position.x, self.uav_pose.pose.position.y,
                            self.uav_pose.pose.position.z])
        obs_pos = np.array(obstacle.pose[:3])
        dist_vector = np.subtract(uav_pos, obs_pos)
        # Calculate angle between two vectors
        dist_vector = dist_vector / np.linalg.norm(dist_vector)
        cu_dv = cu_dv / np.linalg.norm(cu_dv)
        angle_uav_obs = np.arccos(np.clip(np.dot(dist_vector, cu_dv), -1.0, 1.0))
        angle_uav_obs = abs(math.degrees(angle_uav_obs))  # turn angle from radians to degrees and give absolute value
        print(angle_uav_obs)
        return angle_uav_obs

    def _get_all_obstacles(self, filepath):
        # type: (str) -> None
        """Reads the gazebo xml-file and saves all obstacles whose name begins with the predefined prefix."""

        self.all_obstacles = read_gazebo_xml(filepath)
        return

    # Added another condition to check if obstacle is within field of view of sensor
    def _get_active_obstacles(self):
        """Loops over all obstacles in self.all_obstacles and checks wether they are within the sensor's scope, defined
        by self.range and self.angle. If true, they are added to self.active_obstacles."""
        self.active_obstacles = list()
        for obstacle_i in self.all_obstacles:
            if self.get_distance_uav_obstacle(obstacle_i) <= self.range and \
                    self.get_angle_uav_obstacles(obstacle_i) <= self.angle:
                self.active_obstacles.append(obstacle_i)
        return

    def _pose_callback(self, data):
        # type: (PoseStamped) -> None
        """Callback function for the subscription to /mavros/local_position/pose."""
        self.uav_pose = data
        return

    def _pub_active_obstacles(self):
        """Publisher Function that publishes self.active_obstacles in the ObstacleListMsg format
        to ~/active_obstacles."""
        msg = obstacleListMsg()
        for obstacle_i in self.active_obstacles:
            msg.obstacleList.append(obstacle_i.to_rosmsg())

        self.pub_obstacle_list.publish(msg)
        rospy.loginfo("obstacle_sensor: Published active obstacles")
        return

    def start(self, filepath):
        # type: (str) -> None
        """main function of ObstaclesSensor. First reads all obstacles from the xml file, then publishes all obstacles
        that are within range of the uav."""

        rospy.loginfo("obstacle_sensor: Node started")

        # Read the XML file
        self._get_all_obstacles(filepath)
        rospy.loginfo("obstacle_sensor: All Obstacles have been read from the XML-File")

        # Wait until the UAV position is published. Otherwise there will be errors
        # because self.uav_pose is by default = None
        rospy.sleep(1)
        while True:
            if self.uav_pose is None:
                rospy.loginfo("obstacle_sensor: Waiting for UAV Pose")
                self.loop_rate.sleep()
            else:
                rospy.loginfo("obstacle_sensor: UAV Pose received")
                break

        # Loop until infinity
        while not rospy.is_shutdown():
            self._get_active_obstacles()
            self._pub_active_obstacles()
            self.loop_rate.sleep()

        return
