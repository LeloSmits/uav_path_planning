#!/usr/bin/env python2

import math
import numpy as np
import rospy
import typing

from geometry_msgs.msg import PoseStamped
from uav_path_planning.msg import obstacleMsg, obstacleListMsg


class ObstacleMap(object):
    def __init__(self):
        self.name = 'obstacle_map'
        rospy.init_node(self.name)

        self.loop_rate = rospy.Rate(1)

        self.uav_pose = None

        self.new_obstacles = list()  # type: typing.List[obstacleMsg]
        self.map = list()  # type: typing.List[obstacleMsg]
        self.radius = 50  # in meters

        rospy.Subscriber('~/active_obstacles', obstacleListMsg, self._obstacle_list_callback)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._pose_callback)
        self.pub_obstacle_map = rospy.Publisher("~/obstacle_map", obstacleListMsg, queue_size=100)  # ToDo: find out if queue_size is important

        rospy.loginfo(self.name + ": Node initialized")

    def _obstacle_list_callback(self, data):
        # type: (obstacleListMsg) -> None
        """Callback function for the subscription to /mavros/local_position/pose."""
        self.new_obstacles = data.obstacleList
        return

    def _pose_callback(self, data):
        # type: (PoseStamped) -> None
        """Callback function for the subscription to /mavros/local_position/pose."""
        self.uav_pose = data
        return

    def _pub_obstacle_map(self):
        """Publisher Function that publishes self.active_obstacles in the ObstacleListMsg format
        to ~/active_obstacles."""
        # ToDo:
        msg = obstacleListMsg()
        for obstacle_i in self.map:
            msg.append(obstacle_i.to_rosmsg())

        self.pub_obstacle_map.publish(msg)
        rospy.loginfo(self.name + ": Published obstacle map")
        return

    def _update_map(self):
        """Compares the incoming new obstacles to the existing obstacles in the map. If they are new, add them. If any
        obstacle is outside of self.radius, remove it."""
        # Remove all new obstacles outside of radius
        new_obstacles_temp = [obstacle_i for obstacle_i in self.new_obstacles if self._check_if_within(obstacle_i)]
        # Remove all old obstacles outside of radius
        self.map[:] = [obstacle_i for obstacle_i in self.map if self._check_if_within(obstacle_i)]
        original_length = len(self.map)

        for new_obstacle_i in new_obstacles_temp:
            already_exists = False
            for j, old_obstacle_j in enumerate(self.map[:original_length]):
                if new_obstacle_i.name == old_obstacle_j.name:
                    already_exists = True
                    break

            if not already_exists:
                self.map.append(new_obstacle_i)
        return

    def _check_if_within(self, obstacle):
        # type: (obstacleMsg) -> bool
        """Checks if obstacle is within a radius (defined with self.radius)
        around the UAV position (defined in self.uav_pose)"""
        uav_pos = np.array(
            [self.uav_pose.pose.position.x, self.uav_pose.pose.position.y, self.uav_pose.pose.position.z])
        obs_pos = np.array(obstacle.pose[:3])
        return np.linalg.norm((self.uav_pose, obs_pos)) <= self.radius

    def start(self):
        """main function of ObstaclesSensor. First reads all obstacles from the xml file, then publishes all obstacles
        that are within range of the uav."""

        rospy.loginfo(self.name + ": Node started")

        rospy.sleep(1)
        while True:
            if self.uav_pose is None:
                rospy.loginfo(self.name + ": Waiting for UAV Pose")
                self.loop_rate.sleep()
            else:
                rospy.loginfo(self.name + ": UAV Pose received")
                break

        while not rospy.is_shutdown():
            self._update_map()
            self._pub_obstacle_map()
            self.loop_rate.sleep()

        return
