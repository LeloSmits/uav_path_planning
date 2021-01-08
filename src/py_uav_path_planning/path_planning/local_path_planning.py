#!/usr/bin/env python2

import numpy as np
import rospy
import typing
from threading import Thread

from geometry_msgs.msg import PoseStamped
from uav_path_planning.msg import obstacleMsg, obstacleListMsg
from std_msgs.msg import Header

from potentialfield import get_potential_field, get_vector_field
from mavros_msgs.msg import PositionTarget


class LocalPathPlanner(object):
    def __init__(self):
        self.name = 'local_path_planner'
        rospy.init_node(self.name)

        self._new_wp_rate = rospy.Rate(10)
        self._pub_wp_rate = rospy.Rate(20)
        self._map = list()  # type: typing.List[obstacleMsg]
        self.uav_pose = None  # type: PoseStamped  # probably
        self.goal_pose_stamped = None  # type: PoseStamped
        self.goal_coordinate = None  # type: np.array

        self.step_size = 1.  # type: float  # ToDo: Make adaptive
        self.max_iter_per_wp = 100  # type: int
        self.wp_local_new = None  # type: PoseStamped
        self.tol_wp_local = rospy.get_param('tol_wp_local', .5)  # Absolute tolerance to set WAYPOINT_ACHIEVED to True when L2-Distance between UAV and local waypoint is less or equal
        self.tol_wp_global = rospy.get_param('tol_wp_global', .1)  # Same as above but for gloabl waypoint

        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._pose_callback)
        rospy.Subscriber('obstacle_map', obstacleListMsg, self._map_callback)
        rospy.Subscriber('wp_global_current', PoseStamped, self._wp_global_callback)

        self._pub_new_wp = rospy.Publisher("wp_local_current", PositionTarget, queue_size=1)
        self._thread_new_wp = Thread(target=self._pub_waypoints, args=())
        self._thread_new_wp.daemon = True  # Deamon-Threads werden gekillt, wenn das Hauptprogramm beendet

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

    def _wp_global_callback(self, data):
        self.goal_pose_stamped = data
        return

    def _pub_waypoints(self):
        """Publisher Function that publishes self.new_wp in the PoseStamped format
        to mavros/setpoint_position/local."""

        new_wp_msg = PositionTarget()
        # new_wp_msg.type_mask = 3064  # 0b101111111000
        new_wp_msg.coordinate_frame = 1

        while not rospy.is_shutdown():
            new_wp_msg.header = Header()
            new_wp_msg.header.stamp = rospy.Time.now()

            new_wp_msg.position.x = self.wp_local_new[0]  # x
            new_wp_msg.position.y = self.wp_local_new[1]  # y
            new_wp_msg.position.z = .5  # self.new_wp[2]  # z  # ToDo: Make Variable

            new_wp_msg.yaw = np.arccos((np.array([self.uav_pose.pose.position.x, self.uav_pose.pose.position.y]).dot(self.wp_local_new[:2])
                                        / (np.linalg.norm(np.array([self.uav_pose.pose.position.x, self.uav_pose.pose.position.y])) * np.linalg.norm(self.wp_local_new[:2]))))
            print(new_wp_msg.yaw)
            self._pub_new_wp.publish(new_wp_msg)
            self._pub_wp_rate.sleep()
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

            uav_pos_new = uav_pos - step_size * gradient / np.linalg.norm(gradient)  # Subtraction because we want to descent / move to smaller potential
            potential_new = get_potential_field(uav_coordinate=uav_pos_new, goal_coordinate=self.goal_coordinate,
                                                obstacle_map=self._map)

            if potential_new > potential_here:
                step_size /= 2
            else:
                return uav_pos_new
        return uav_pos

    # ToDo: find_path tries to reach self.goal_coordinate even when wp_global_current has changed.
    #  Change so that find_path exits when wp_global_current changes
    def find_path(self):
        uav_pos = np.array([self.uav_pose.pose.position.x, self.uav_pose.pose.position.y,
                            self.uav_pose.pose.position.z])

        while not all(np.isclose(uav_pos, self.goal_coordinate, atol=self.tol_wp_global)):  # return after global WP is reached
            self.wp_local_new = self.get_new_wp()

            if not self._thread_new_wp.isAlive():  # Start publishing waypoints
                self._thread_new_wp.start()

            uav_pos = np.array([self.uav_pose.pose.position.x, self.uav_pose.pose.position.y,
                                self.uav_pose.pose.position.z])

            self._new_wp_rate.sleep()

        self.wp_local_new = self.goal_coordinate
        self._new_wp_rate.sleep()
        return

    def start(self):
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

        while self.goal_pose_stamped is None:
            rospy.sleep(1)

        rospy.loginfo(self.name + ': Starting local path planning')
        while not rospy.is_shutdown():
            # self.goal_pose_stamped = rospy.get_param('wp_global_current')
            self.goal_coordinate = np.array([self.goal_pose_stamped.pose.position.x,
                                             self.goal_pose_stamped.pose.position.y,
                                             self.goal_pose_stamped.pose.position.z])
            self.find_path()

        return
