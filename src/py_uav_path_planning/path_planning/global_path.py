#!/usr/bin/env python2

import numpy as np
import rospy
import typing
import threading
import copy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import SetMode, CommandBool, SetModeResponse


class GlobalPath(object):
    def __init__(self):
        self.name = 'global_path_planner'
        rospy.init_node(self.name)

        self.uav_pose = None  # type: PoseStamped
        self.uav_pose_at_start = None  # type: PoseStamped
        self.mavros_state = None  # type: State

        # ToDo: read from params
        # self.wp_global_all = rospy.get_param('wp_global_all')  # type: typing.List[PoseStamped]
        self.wp_global_all = (PoseStamped(), PoseStamped(), PoseStamped())

        self.wp_global_all[0].pose.position.x = 15  # x
        self.wp_global_all[0].pose.position.y = -7  # y
        self.wp_global_all[0].pose.position.z = .5  # z, default is 10

        self.wp_global_all[1].pose.position.x = 9  # x
        self.wp_global_all[1].pose.position.y = -1  # y
        self.wp_global_all[1].pose.position.z = .5  # z, default is 10

        self.wp_global_all[2].pose.position.x = 25  # x
        self.wp_global_all[2].pose.position.y = 0  # y
        self.wp_global_all[2].pose.position.z = .5  # z, default is 10

        self.wp_local_current = None

        self.tol_wp_local = rospy.get_param('tol_wp_local', .1)  # Absolute tolerance to set WAYPOINT_ACHIEVED to
        # True when L2-Distance between UAV and local waypoint is less or equal
        self.tol_wp_global = rospy.get_param('tol_wp_global', .1)  # Same as above but for gloabl waypoint

        self._rate_publish_wp = rospy.Rate(20)
        self._rate_check_reached_global_wp = rospy.Rate(10)

        # Thread for takeoff waypoint publishing
        self._thread_takeoff_waypoint = None  # type: threading.Thread

        # Thread to keep UAV armed
        self._thread_keep_armed_offboard = threading.Thread(target=self._keep_armed_and_offboard)
        self._thread_keep_armed_offboard.daemon = True

        self._pub_wp_takeoff = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=1)
        self._pub_wp_local = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=1)
        self._pub_wp_global_current = rospy.Publisher("wp_global_current", PoseStamped, queue_size=1)

        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._callback_pose)
        rospy.Subscriber("/mavros/state", State, self._callback_current_state)
        rospy.Subscriber("wp_local_current", PositionTarget, self._callback_waypoint_local_current)

        rospy.loginfo(self.name + ": Node initialized")
        return

    def _callback_pose(self, data):
        # type: (PoseStamped) -> None
        """Callback function for the subscription to /mavros/local_position/pose."""
        self.uav_pose = data
        return

    def _callback_current_state(self, data):
        # type: (State) -> None
        """Callback function for the subscription to /mavros/state."""
        self.mavros_state = data
        return

    def _callback_waypoint_local_current(self, data):
        # type: (PositionTarget) -> None
        """Callback function for the subscription to /mavros/state."""
        self.wp_local_current = data
        return

    def _publish_takeoff_waypoint(self, takeoff_waypoint):
        # type: (PositionTarget) -> None
        t = threading.current_thread()
        while not rospy.is_shutdown() and getattr(t, "do_run", True):
            takeoff_waypoint.header = Header()
            takeoff_waypoint.header.stamp = rospy.Time.now()
            self._pub_wp_takeoff.publish(takeoff_waypoint)
            self._rate_publish_wp.sleep()
        return

    def _publish_waypoint_global_current(self, wp_global):
        self._pub_wp_global_current.publish(wp_global)
        return

    def _publish_waypoint_local_current(self, wp_local):
        if wp_local is None:
            rospy.loginfo(self.name + ': No local waypoint published yet')
            return
        self._pub_wp_local.publish(wp_local)
        return

    def _keep_armed_and_offboard(self):
        """Sets armed to True and changes uav-mode to OFFBOARD, so manual waypoints can be published"""
        arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        offb_set_mode = SetMode()
        offb_set_mode.custom_mode = "OFFBOARD"

        arm_cmd = CommandBool()
        arm_cmd.value = True

        last_request = rospy.Time.now()
        while not rospy.is_shutdown():
            if self.mavros_state.mode != "OFFBOARD" and (rospy.Time.now() - last_request > rospy.Duration(5)):
                # print offb_set_mode._response_class.mode_sent
                res = set_mode_client(0, offb_set_mode.custom_mode)
                if res and res.mode_sent:
                    rospy.loginfo(self.name + ": Offboard enabled")
                last_request = rospy.Time.now()
            else:
                if not self.mavros_state.armed and (rospy.Time.now() - last_request > rospy.Duration(5)):
                    res = arming_client(True)
                    if res and res.success:
                        rospy.loginfo(self.name + ": Vehicle armed")
                    last_request = rospy.Time.now()
        return

    def _takeoff_procedure(self):
        # Get Goal-Waypoint for Takeoff procedure
        takeoff_altitude = rospy.get_param('takeoff_altitude', 10)
        takeoff_waypoint = PositionTarget()
        takeoff_waypoint.type_mask = 0b111111111000  # 0b000111111111
        takeoff_waypoint.position.x = self.uav_pose_at_start.pose.position.x  # x
        takeoff_waypoint.position.y = self.uav_pose_at_start.pose.position.y  # y
        takeoff_waypoint.position.z = takeoff_altitude  # z, default is 10
        takeoff_waypoint.coordinate_frame = 1

        for i in range(100):  # Push some waypoints into the queue before ARMING and switching to OFFBOARD
            takeoff_waypoint.header = Header()
            takeoff_waypoint.header.stamp = rospy.Time.now()
            self._pub_wp_takeoff.publish(takeoff_waypoint)

        # Start a seperate Thread to publish the WPs for takeoff
        self._thread_takeoff_waypoint = threading.Thread(target=self._publish_takeoff_waypoint,
                                                         args=(takeoff_waypoint,))  # The comma in args is intentional
        self._thread_takeoff_waypoint.daemon = True
        self._thread_takeoff_waypoint.start()

        # Start a seperate Thread to keep the UAV armed and in Offboard mode
        self._thread_keep_armed_offboard.start()

        while not self._check_if_pose_isclose(self.uav_pose, takeoff_waypoint, atol=self.tol_wp_local):
            self._rate_check_reached_global_wp.sleep()
        return

    @staticmethod
    def _check_if_pose_isclose(poseStamped_1, pose_2, atol):
        # type: (PoseStamped, PoseStamped, float) -> bool
        """Checks if the xyz position of the first two arguments is close in regard to the given absolute tolerance
        :arg poseStamped_1: first pose
        :arg pose_2: second pose
        :arg atol: absolute tolerance
        :returns bool"""
        if isinstance(pose_2, PoseStamped):
            pos_1 = np.array([poseStamped_1.pose.position.x, poseStamped_1.pose.position.y,
                              poseStamped_1.pose.position.z])
            pos_2 = np.array([pose_2.pose.position.x, pose_2.pose.position.y,
                              pose_2.pose.position.z])
            return all(np.isclose(pos_1, pos_2, atol=atol))
        elif isinstance(pose_2, PositionTarget):
            pos_1 = np.array([poseStamped_1.pose.position.x, poseStamped_1.pose.position.y,
                              poseStamped_1.pose.position.z])
            pos_2 = np.array([pose_2.position.x, pose_2.position.y,
                              pose_2.position.z])
            return all(np.isclose(pos_1, pos_2, atol=atol))

    def start(self):
        """main function of GlobalPathPlanner"""

        rospy.loginfo(self.name + ": Node started")

        rospy.sleep(1)
        while True:
            if self.uav_pose is None:
                rospy.loginfo(self.name + ": Waiting for UAV Pose")
                rospy.sleep(1)
            else:
                self.uav_pose_at_start = copy.copy(
                    self.uav_pose)  # Vllt wird copy nicht gebraucht, bin mir aber unsicher
                rospy.loginfo(self.name + ": UAV Pose received")
                break

        # Set mode to Offboard, Arm the UAV and takeoff to set altitude
        takeoff_success = False
        self._takeoff_procedure()
        rospy.sleep(1)  # To prevent that takeoff goes directly into path following
        rospy.loginfo('Takeoff procedure finished')

        for wp_global in self.wp_global_all:  # Iterates over all global waypoints
            rospy.loginfo(self.name + ': Published new global waypoint')
            self._publish_waypoint_global_current(wp_global)

            # This loops keeps running the takeoff thread until local planner has published the first waypoint
            while not takeoff_success:
                if self.wp_local_current is None:
                    self._rate_publish_wp.sleep()
                else:
                    takeoff_success = True
                    self._thread_takeoff_waypoint.do_run = False  # Stops sending the takeoff waypoint. Between this and
                    # sending the next waypoint from the local planner can be a maximum of .5 seconds, since waypoints have
                    # to be published with >2Hz (PX4/MAVROS restriction)
                    break

            while not self._check_if_pose_isclose(self.uav_pose, wp_global, atol=self.tol_wp_global):
                self._publish_waypoint_local_current(self.wp_local_current)
                self._rate_publish_wp.sleep()
            rospy.loginfo(self.name + ': Reached previous global waypoint')

        rospy.loginfo('Reached final global waypoint')
        rospy.spin()
        return
