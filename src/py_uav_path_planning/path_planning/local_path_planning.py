#!/usr/bin/env python2

import math
import numpy as np
import rospy
import typing
from threading import Thread

from geometry_msgs.msg import PoseStamped
from uav_path_planning.msg import obstacleMsg, obstacleListMsg
from std_msgs.msg import Header

from potentialfield import get_potential_field, get_vector_field


class LocalPathPlanner(object):
    def __init__(self):
        self.name = 'local_path_planner'
        rospy.init_node(self.name)

        self._new_wp_rate = rospy.Rate(2)
        self._pub_wp_rate = rospy.Rate(10)
        self._map = list()  # type: typing.List[obstacleMsg]
        self.uav_pose = None  # type: PoseStamped  # probably
        self.goal_coordinate = None  # type: np.array

        self.step_size = 1.  # type: float
        self.max_iter_per_wp = 100  # type: int
        self.new_wp = None  # type: PoseStamped
        self._wp_tolerance = 0.1  # type: float

        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._pose_callback)
        rospy.Subscriber('~/obstacle_map', obstacleListMsg, self._map_callback)

        self._pub_new_wp = rospy.Publisher("mavros/setpoint_position/local", PoseStamped,
                                                  queue_size=1)  # ToDo: find out if queue_size is important
        self._new_wp_thread = Thread(target=self._pub_waypoints, args=())

        rospy.loginfo(self.name + ": Node initialized")
        return

    def _pose_callback(self, data):
        # type: (PoseStamped) -> None
        """Callback function for the subscription to /mavros/local_position/pose."""
        self.uav_pose = data
        return

    def _map_callback(self, data):
        # type: (obstacleListMsg) -> None
        """Callback function for the subscription to ~/obstacle_map."""
        self._map = data.obstacleList
        return

    def _pub_waypoints(self):
        """Publisher Function that publishes self.new_wp in the PoseStamped format
        to mavros/setpoint_position/local."""

        new_wp_msg = PoseStamped()
        new_wp_msg.header = Header()

        while not rospy.is_shutdown():
            new_wp_msg.header.stamp = rospy.Time.now()

            new_wp_msg.pose.position.x = self.new_wp[0]  # x
            new_wp_msg.pose.position.y = self.new_wp[1]  # y
            new_wp_msg.pose.position.z = self.new_wp[2]  # z

            self._pub_new_wp.publish(new_wp_msg)
            try:  # prevent garbage in console output when thread is killed
                self._pub_wp_rate.sleep()
            except rospy.ROSInterruptException:
                pass
        return

    def get_new_wp(self):
        uav_pos = np.array([self.uav_pose.pose.position.x, self.uav_pose.pose.position.y,
                            self.uav_pose.pose.position.z])

        step_size = self.step_size
        for i in range(self.max_iter_per_wp):
            uav_pos = np.array([self.uav_pose.pose.position.x, self.uav_pose.pose.position.y,
                                self.uav_pose.pose.position.z])

            gradient = get_vector_field(uav_coordinate=uav_pos, goal_coordinate=self.goal_coordinate,
                                        obstacle_map=self._map)
            potential_here = get_potential_field(uav_coordinate=uav_pos, goal_coordinate=self.goal_coordinate,
                                                 obstacle_map=self._map)

            uav_pos_new = uav_pos - step_size * gradient  # Subtraction because we want to descent / move to smaller potential
            potential_new = get_potential_field(uav_coordinate=uav_pos_new, goal_coordinate=self.goal_coordinate,
                                                obstacle_map=self._map)

            if potential_new > potential_here: step_size /= 2
            else: return uav_pos_new
        return uav_pos

    def find_path(self):
        uav_pos = np.array([self.uav_pose.pose.position.x, self.uav_pose.pose.position.y,
                            self.uav_pose.pose.position.z])

        while True:
            while not all(np.isclose(uav_pos, self.goal_coordinate, atol=self._wp_tolerance)):
                self.new_wp = self.get_new_wp()

                if not self._new_wp_thread.isAlive():  # Start publishing waypoints
                    self._new_wp_thread.start()

                uav_pos = np.array([self.uav_pose.pose.position.x, self.uav_pose.pose.position.y,
                                    self.uav_pose.pose.position.z])

                self._new_wp_rate.sleep()

            self.new_wp = self.goal_coordinate

    def start(self, goal_coordinate):
        """main function of LocalPathPlanner"""

        rospy.loginfo(self.name + ": Node started")

        rospy.sleep(1)
        while True:
            if self.uav_pose is None:
                rospy.loginfo(self.name + ": Waiting for UAV Pose")
                self._new_wp_rate.sleep()
            else:
                rospy.loginfo(self.name + ": UAV Pose received")
                break

        self.goal_coordinate = goal_coordinate

        self._new_wp_thread.daemon = True  # keine Ahnung, wofuer das ist. War in den Px4-py tutorials

        while not rospy.is_shutdown():
            self.find_path()
            self._new_wp_rate.sleep()

        return
