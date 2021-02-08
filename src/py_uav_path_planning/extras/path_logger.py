#!/usr/bin/env python2

import numpy as np
import rospy
from os.path import expanduser, join
from os import mkdir
import errno
from datetime import datetime
import pickle

from geometry_msgs.msg import PoseStamped, Pose, TwistStamped
from visualization_msgs.msg import MarkerArray
from mavros_msgs.msg import PositionTarget, Waypoint, WaypointList


Baumann = False


class PathLogger:
    def __init__(self):
        self.name = 'path_logger'
        rospy.init_node(self.name)

        self.active = False
        self.rate = rospy.Rate(10)

        self.waypoint_global_next_pos = np.zeros(3)
        self.waypoint_global_previous_pos = np.zeros(3)

        self.time_started = datetime.now()

        log_files_folder = "path_planning_logs"
        home_dir = expanduser("~")
        try:
            mkdir(join(home_dir, log_files_folder))
        except OSError as e:
            if e.errno == errno.EEXIST:
                pass
            else:
                raise
        self.file_name = 'path_log_' + self.time_started.strftime("%d_%m_%Y_%H_%M_%S") + '.p'
        self.file_path = join(home_dir, log_files_folder, self.file_name)

        self.update_waypoints = True

        self.uav_pose_stamped = None
        self.uav_velocity_stamped = None
        self.waypoint_global_next_published = None
        self.waypoint_global_previous_published = None
        self.marker_array_goal_position = None

        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self._callback_pose)
        rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped, self._callback_velocity)
        rospy.Subscriber('waypoint_global_next', Waypoint, self._callback_waypoint_global_current)
        rospy.Subscriber('waypoint_global_previous', Waypoint, self._callback_waypoint_global_previous)
        rospy.Subscriber('waypoint_global_previous', Waypoint, self._callback_waypoint_global_previous)
        rospy.Subscriber('/goal_position', MarkerArray, self._callback_goal_position)

    def start(self):
        meta_data = {"Time_started": self.time_started,
                     "Column_names": ("Time", "UAV_Pos_x", "UAV_Pos_y", "UAV_Pos_z",
                                      "WP_Next_Pos_x", "WP_Next_Pos_y", "WP_Next_Pos_z",
                                      "WP_Prev_Pos_x", "WP_Prev_Pos_y", "WP_Prev_Pos_z")}

        log_data = np.zeros((1000, 16))

        rospy.sleep(1)

        while not rospy.has_param("path_logger_active"):
            self.rate.sleep()

        self.active = rospy.get_param("path_logger_active")
        while not self.active and not rospy.is_shutdown():
            self.active = rospy.get_param("path_logger_active")
            self.rate.sleep()

        if self.waypoint_global_next_published is None or self.waypoint_global_previous_published is None:
            self.update_waypoints = False
            self.waypoint_global_next_pos = np.array([self.marker_array_goal_position.markers[0].pose.position.x,
                                                      self.marker_array_goal_position.markers[0].pose.position.y,
                                                      self.marker_array_goal_position.markers[0].pose.position.z])

            self.waypoint_global_previous_pos = np.array([self.uav_pose_stamped.pose.position.x,
                                                          self.uav_pose_stamped.pose.position.y,
                                                          self.uav_pose_stamped.pose.position.z])

        jj = 0

        rospy.loginfo(self.name + ": Started logging")
        while self.active:
            time = float(self.uav_pose_stamped.header.stamp.secs) + \
                   float(self.uav_pose_stamped.header.stamp.nsecs) / 10**9

            if self.update_waypoints:
                self.update_waypoint_pos()

            if jj == len(log_data):
                log_data = np.concatenate((log_data, np.zeros(log_data.shape)), axis=0)

            log_data[jj] = [time,
                            self.uav_pose_stamped.pose.position.x,
                            self.uav_pose_stamped.pose.position.y,
                            self.uav_pose_stamped.pose.position.z,
                            self.waypoint_global_next_pos[0],
                            self.waypoint_global_next_pos[1],
                            self.waypoint_global_next_pos[2],
                            self.waypoint_global_previous_pos[0],
                            self.waypoint_global_previous_pos[1],
                            self.waypoint_global_previous_pos[2],
                            self.uav_velocity_stamped.twist.linear.x,
                            self.uav_velocity_stamped.twist.linear.y,
                            self.uav_velocity_stamped.twist.linear.z,
                            self.uav_velocity_stamped.twist.angular.x,
                            self.uav_velocity_stamped.twist.angular.y,
                            self.uav_velocity_stamped.twist.angular.z]

            if all(np.isclose(log_data[jj, 1:4], log_data[jj, 4:7], atol=0.5)):
                break

            jj += 1
            self.active = rospy.get_param("path_logger_active")

            if rospy.is_shutdown():
                self.file_path = self.file_path + "_ABORTED"
                break

            self.rate.sleep()

        log_data = log_data[:jj+1]
        rospy.loginfo(self.name + ": Ended logging")

        data = {"meta_data": meta_data, "position_data": log_data}
        file_pickle = open(self.file_path, "wb")
        pickle.dump(data, file_pickle)
        file_pickle.close()
        rospy.loginfo(self.name + ": Data saved")

        return

    def update_waypoint_pos(self):
        self.waypoint_global_next_pos = np.array([self.waypoint_global_next_published.x_lat,
                                                  self.waypoint_global_next_published.y_long,
                                                  self.waypoint_global_next_published.z_alt])
        self.waypoint_global_previous_pos = np.array([self.waypoint_global_previous_published.x_lat,
                                                      self.waypoint_global_previous_published.y_long,
                                                      self.waypoint_global_previous_published.z_alt])
        return

    def _callback_pose(self, uav_pose_stamped):
        # type: (PoseStamped) -> None
        """Callback function for the subscription to /mavros/local_position/pose."""
        self.uav_pose_stamped = uav_pose_stamped
        return

    def _callback_velocity(self, uav_velocity_stamped):
        # type: (PoseStamped) -> None
        """Callback function for the subscription to /mavros/local_position/pose."""
        self.uav_velocity_stamped = uav_velocity_stamped
        return

    def _callback_waypoint_global_current(self, waypoint_global):
        # type: (Waypoint) -> None
        self.waypoint_global_next_published = waypoint_global
        return

    def _callback_waypoint_global_previous(self, waypoint_global):
        # type: (Waypoint) -> None
        self.waypoint_global_previous_published = waypoint_global
        return

    def _callback_goal_position(self, goal_position):
        # type: (MarkerArray) -> None
        self.marker_array_goal_position = goal_position
        return
