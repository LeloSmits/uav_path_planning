#!/usr/bin/env python2

import numpy as np
import rospy
import typing
import threading
import copy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from mavros_msgs.msg import State, PositionTarget, Waypoint, WaypointList
from mavros_msgs.srv import SetMode, CommandBool, SetModeResponse


class GlobalPath(object):
    def __init__(self):
        self.name = 'global_path'
        rospy.init_node(self.name)

        self.uav_pose = None  # type: PoseStamped  # ToDo Change to Pose not PoseStamped
        self.uav_pose_at_start = None  # type: PoseStamped
        self.mavros_state = None  # type: State

        # ToDo: read from params
        # self.wp_global_all = rospy.get_param('wp_global_all')  # type: typing.List[PoseStamped]
        self.waypoint_global_all = WaypointList()
        wp_1 = Waypoint()
        wp_1.x_lat = 25
        wp_1.y_long = 0
        wp_1.z_alt = .5
        self.waypoint_global_all.waypoints.append(wp_1)

        self.waypoint_global_current = None  # type: Waypoint
        self.waypoint_global_previous = None  # type: Waypoint

        self.local_setpoint = None  # type: PositionTarget

        self.finished = False  # Indicate if final point is reached

        # ToDo: Remove tol_wp, use message from local path planner instead
        self.tol_wp_reached = rospy.get_param('tol_wp_reached', .1)
        self.tol_takeoff = rospy.get_param('tol_wp_takeoff', .1)
        # self.tol_wp_local = rospy.get_param('tol_wp_local', .1)  # Absolute tolerance to set WAYPOINT_ACHIEVED to
        # self.tol_wp_global = rospy.get_param('tol_wp_global', .1)  # Same as above but for gloabl waypoint

        # --- RATES --- #
        self._rate_publish = rospy.Rate(20)
        self._rate_reached_waypoint = rospy.Rate(20)

        # --- THREADS --- #
        # Thread for global waypoint publishing
        self._thread_waypoint_global = threading.Thread(target=self._publish_global_waypoints)
        self._thread_waypoint_global.daemon = False

        # Thread for takeoff waypoint publishing
        self._thread_takeoff_setpoint = None  # type: threading.Thread

        # Thread to keep UAV armed
        self._thread_arm_and_offboard = threading.Thread(target=self._arm_and_offboard)
        self._thread_arm_and_offboard.daemon = True

        # Thread to forward the local setpoints from Local Planner to Mavros
        self._thread_forward_local_setpoints = threading.Thread(target=self._forward_local_setpoints)
        self._thread_forward_local_setpoints.daemon = True

        # Thread to log waypoints. Maybe put into a seperate node?
        self._thread_position_logger = threading.Thread(target=self.log_path)
        self._thread_position_logger.daemon = False

        # --- PUBLISHERS --- #
        # Publishes the next global waypoint
        self._pub_waypoint_global_current = rospy.Publisher("waypoint_global_next", Waypoint, queue_size=1)
        # Publishes the previous global waypoint. Needed to build the "trench" artificial potential field
        self._pub_waypoint_global_previous = rospy.Publisher("waypoint_global_previous", Waypoint, queue_size=1)
        # Publishes the setpoint directly to MAVROS for starting and landing
        self._pub_setpoint = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=1)

        # --- SUBSCRIBERS --- #
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self._callback_pose)
        rospy.Subscriber("/mavros/state", State, self._callback_current_state)
        rospy.Subscriber("local_setpoint", PositionTarget, self._callback_local_setpoint)

        rospy.loginfo(self.name + ": Node initialized")
        return

    def _callback_pose(self, uav_pose):
        # type: (PoseStamped) -> None
        """Callback function for the subscription to /mavros/local_position/pose."""
        self.uav_pose = uav_pose
        return

    def _callback_current_state(self, mavros_state):
        # type: (State) -> None
        """Callback function for the subscription to /mavros/state."""
        self.mavros_state = mavros_state
        return

    def _callback_local_setpoint(self, local_setpoint):
        # type: (PositionTarget) -> None
        """Callback function for the subscription to local_setpoint"""
        self.local_setpoint = local_setpoint
        return

    def _publish_setpoint(self, setpoint):
        # type: (PositionTarget) -> None
        """Publishes the setpoint to mavros"""
        t = threading.current_thread()
        while not rospy.is_shutdown() and getattr(t, "do_run", True):
            setpoint.header = Header()
            setpoint.header.stamp = rospy.Time.now()
            self._pub_setpoint.publish(setpoint)
            self._rate_publish.sleep()
        return

    def _publish_global_waypoints(self):
        """Publishes the setpoint to mavros"""
        t = threading.current_thread()

        # Wait until the global waypoints are set
        while (self.waypoint_global_current is None or self.waypoint_global_previous is None) \
                and getattr(t, "do_run", True):
            self._rate_publish.sleep()

        while not rospy.is_shutdown() and getattr(t, "do_run", True):
            self._pub_waypoint_global_current.publish(self.waypoint_global_current)
            self._pub_waypoint_global_previous.publish(self.waypoint_global_previous)
            self._rate_publish.sleep()
        return

    def _arm_and_offboard(self):
        """Sets armed to True and changes uav-mode to OFFBOARD, so manual waypoints can be published"""
        arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        offb_set_mode = SetMode()
        offb_set_mode.custom_mode = "OFFBOARD"

        arm_cmd = CommandBool()
        arm_cmd.value = True

        wait_time = rospy.Duration(1)

        last_request = rospy.Time.now()
        while not rospy.is_shutdown() and (self.mavros_state.mode != "OFFBOARD" or not self.mavros_state.armed):
            if self.mavros_state.mode != "OFFBOARD" and (rospy.Time.now() - last_request > wait_time):
                res = set_mode_client(0, offb_set_mode.custom_mode)
                if res and res.mode_sent:
                    rospy.loginfo(self.name + ": Offboard enabled")
                last_request = rospy.Time.now()
            else:
                if not self.mavros_state.armed and (rospy.Time.now() - last_request > wait_time):
                    res = arming_client(True)
                    if res and res.success:
                        rospy.loginfo(self.name + ": Vehicle armed")
                    last_request = rospy.Time.now()
        return

    def _takeoff_procedure(self, uav_pose_start):
        # type: (PoseStamped) -> None
        # Get Goal-Waypoint for Takeoff procedure
        rospy.loginfo(self.name + ": Takeoff procedure started")

        takeoff_altitude = rospy.get_param('takeoff_altitude', 10)

        takeoff_waypoint = PositionTarget()
        takeoff_waypoint.type_mask = 0b111111111000
        takeoff_waypoint.position.x = uav_pose_start.pose.position.x  # x
        takeoff_waypoint.position.y = uav_pose_start.pose.position.y  # y
        takeoff_waypoint.position.z = takeoff_altitude  # z, default is 10
        takeoff_waypoint.coordinate_frame = 1

        for i in range(10):  # Push some waypoints into the queue before ARMING and switching to OFFBOARD
            takeoff_waypoint.header = Header()
            takeoff_waypoint.header.stamp = rospy.Time.now()
            self._pub_setpoint.publish(takeoff_waypoint)

        # Start a seperate Thread to publish the WPs for takeoff
        self._thread_takeoff_setpoint = threading.Thread(target=self._publish_setpoint,
                                                         args=(takeoff_waypoint,))  # The comma in args is intentional
        self._thread_takeoff_setpoint.daemon = True
        self._thread_takeoff_setpoint.start()

        rospy.loginfo(self.name + ": Sending Takeoff points")

        # Start a seperate Thread to keep the UAV armed and in Offboard mode
        self._thread_arm_and_offboard.start()  # This thread is kept alive even after _takeoff_procedure is finished
        rospy.loginfo(self.name + ": Started arming and offboarding")

        while not self._is_at_position(self.uav_pose, takeoff_waypoint, atol=self.tol_takeoff):
            self._rate_reached_waypoint.sleep()
        return

    def _forward_local_setpoints(self):
        """Forwards the commands from the local planner to mavros"""
        # ToDo: Move this into the subscriber callback
        # ToDo: If no new setpoints are incoming, set to loiter
        t = threading.current_thread()
        # Wait until the local setpoint are set
        while self.local_setpoint is None and getattr(t, "do_run", True):
            self._rate_publish.sleep()

        while not rospy.is_shutdown() and getattr(t, "do_run", True):
            self._pub_setpoint.publish(self.local_setpoint)
            self._rate_publish.sleep()
        return

    @staticmethod
    def _is_at_position(pose_1, pose_2, atol):
        # type: (typing.Union[PoseStamped, PositionTarget, Waypoint], typing.Union[PoseStamped, PositionTarget, Waypoint], float) -> bool
        """Checks if the xyz position of the first two arguments is close in regard to the given absolute tolerance
        :arg pose_1: first pose
        :arg pose_2: second pose
        :arg atol: absolute tolerance
        :returns bool"""

        pos = [np.zeros(3), np.zeros(3)]
        for i, pose in enumerate((pose_1, pose_2)):
            if isinstance(pose, PoseStamped):
                pos[i][:] = np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])
            elif isinstance(pose, PositionTarget):
                pos[i][:] = np.array([pose.position.x, pose.position.y, pose.position.z])
            elif isinstance(pose, Waypoint):
                pos[i][:] = np.array([pose.x_lat, pose.y_long, pose.z_alt])
            else:
                raise Warning("Wrong type")

        return all(np.isclose(pos[0], pos[1], atol=atol))

    def log_path(self):
        from datetime import datetime
        import pickle

        uav_path = np.zeros((1000, 3))
        now = datetime.now()
        file_path = '/home/daniel/catkin_ws/src/path_planning_private/src/py_uav_path_planning/path_planning/path_logs/' \
                    + 'path_log_' + now.strftime("%d_%m_%Y_%H_%M_%S")
        file_log = open(file_path, 'wb')
        i = 0

        rospy.loginfo(self.name + ': Position log started')
        while not self.finished:
            uav_path[i] = (self.uav_pose.pose.position.x, self.uav_pose.pose.position.y, self.uav_pose.pose.position.z)
            if i == uav_path.shape[0] - 1:
                uav_path = np.concatenate((uav_path, np.zeros(uav_path.shape)), axis=0)
            i += 1
            self._rate_publish.sleep()

        uav_path = uav_path[:i]
        pickle.dump(uav_path, file_log, protocol=pickle.HIGHEST_PROTOCOL)
        rospy.loginfo(self.name + ': Position log written')
        return

    def start(self):
        """main function of GlobalPathPlanner"""

        rospy.loginfo(self.name + ": Node started")

        rospy.sleep(1)
        while True:
            if self.uav_pose is None:
                rospy.loginfo(self.name + ": Waiting for UAV Pose")
                self._rate_reached_waypoint.sleep()
            else:
                uav_pose_start = copy.copy(self.uav_pose)  # copy is needed here, because uav_pose is mutable!

                # Define the first previous waypoint as the uav start position
                wp_global_previous_temp = Waypoint()
                wp_global_previous_temp.x_lat = uav_pose_start.pose.position.x
                wp_global_previous_temp.y_long = uav_pose_start.pose.position.y
                wp_global_previous_temp.z_alt = uav_pose_start.pose.position.z
                wp_global_previous_temp = copy.copy(wp_global_previous_temp)

                rospy.loginfo(self.name + ": UAV Pose received")
                break

        # Set mode to Offboard, Arm the UAV and takeoff to set altitude
        self._takeoff_procedure(uav_pose_start)
        rospy.sleep(1)  # To prevent that takeoff goes directly into path following
        rospy.loginfo(self.name + ': Takeoff procedure finished')

        # self._thread_position_logger.start()

        self._thread_waypoint_global.start()  # Start publishing global waypoints

        self._thread_forward_local_setpoints.start()  # Starts forwarding the setpoints from the local planner

        self._thread_takeoff_setpoint.do_run = False  # Stops sending the takeoff waypoint. Between this and
        # sending the next waypoint from the local planner can be a maximum of .5 seconds, since waypoints have
        # to be published with >2Hz (PX4/MAVROS restriction)

        for wp_global_current in self.waypoint_global_all.waypoints:  # Iterates over all global waypoints
            self.waypoint_global_current = wp_global_current
            self.waypoint_global_previous = wp_global_previous_temp
            rospy.loginfo(self.name + ': Published new global waypoint')

            while not self._is_at_position(self.uav_pose, wp_global_current, atol=self.tol_wp_reached) \
                    and not rospy.is_shutdown():
                self._rate_reached_waypoint.sleep()

            rospy.loginfo(self.uav_pose)
            rospy.loginfo(wp_global_current)
            rospy.loginfo(self.name + ': Reached previous global waypoint')
            wp_global_previous_temp = copy.copy(wp_global_current)

        self.finished = True
        self._thread_forward_local_setpoints.do_run = False
        rospy.loginfo(self.name + ': Reached final global waypoint')
        rospy.sleep(10)
        return
