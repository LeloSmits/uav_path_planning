#!/usr/bin/env python2
import copy

import numpy as np
import rospy
import typing
from threading import Thread
import threading

from tf.transformations import euler_from_quaternion

from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Header
from mavros_msgs.msg import PositionTarget, Waypoint, WaypointList

from uav_path_planning.msg import obstacleMsg, obstacleListMsg
from uav_path_planning.srv import potential_field_msg, potential_field_msgRequest


# from potentialfield import get_potential_field, get_vector_field


class LocalPathPlanner(object):
    def __init__(self):
        self.name = 'local_path_planner'
        rospy.init_node(self.name)

        self._rate_publish = rospy.Rate(20)
        self._map = list()  # type: typing.List[obstacleMsg]
        self.uav_pose = None  # type: Pose

        self.waypoint_global_next = None  # type: Waypoint
        self.waypoint_global_previous = None  # type: Waypoint
        self.setpoint_local = None  # type: PositionTarget

        # Normal APF Path Planning
        self.speed_max_xy = .5  # type: float  # Maximum speed in xy plane
        self.speed_max_z = .5  # type: float  # Maximum speed in z plane
        self.speed_multiplier = 1  # type: float  # ToDo: Make adaptive
        self.step_size = 1.  # ToDo: Make PARAM
        self.max_iter_per_wp = 100  # type: int

        # Local Minima Bug-APF
        # ToDo: Make PARAM
        self.bug_mode_allowed = rospy.get_param('bug_mode_allowed', 1)
        self.bug_step_size = rospy.get_param('bug_step_size', .2)
        self.bug_timeout = rospy.get_param('bug_timeout', .5)  # seconds
        self.bug_deviation_from_path_max = rospy.get_param('bug_deviation_from_path_max', 20)
        self.bug_gd_step_size = self.step_size  # rospy.get_param('bug_gd_step_size', .5)
        self.bug_potential_diff_limit = rospy.get_param('bug_potential_diff_limit', 1)
        self.bug_gd_max_iter = 1000  # type: int

        self.mode_0_1_angle_threshold = 5.  # degrees
        self.mode_0_1_limit = 1.
        self.mode_1_2_time_limit = 5  # secs

        self.uav_is_stalled = False

        self.tol_wp_global_reached = rospy.get_param('tol_wp_global_reached', .1)  # Tolerance for global waypoint
        self.tol_wp_local_reached = rospy.get_param('tol_wp_reached_local',
                                                    .1)  # Tolerance for local waypoint when using APF*
        self.tol_wp_local_simulated = rospy.get_param('tol_wp_reached_simulated',
                                                      .05)  # Tolerance for local waypoint when creating new nodes for APF* with Gradient Descent
        self.tol_dev_global_path = rospy.get_param('tol_dev_global_path', .5)
        self.threshold_wp_position_control = rospy.get_param('threshold_wp_position_control', 1)

        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._callback_pose)
        rospy.Subscriber('obstacle_map', obstacleListMsg, self._callback_map)
        rospy.Subscriber('waypoint_global_next', Waypoint, self._callback_waypoint_global_current)
        rospy.Subscriber('waypoint_global_previous', Waypoint, self._callback_waypoint_global_previous)

        self.client_apf_potential = rospy.ServiceProxy('get_apf_potential', potential_field_msg)
        self.client_apf_gradient = rospy.ServiceProxy('get_apf_gradient', potential_field_msg)

        self._pub_setpoint = rospy.Publisher("local_setpoint", PositionTarget, queue_size=1)
        self._thread_publish_setpoint = Thread(target=self._publish_setpoint, args=())
        self._thread_publish_setpoint.daemon = True  # Daemon-Threads werden gekillt, wenn das Hauptprogramm beendet

        rospy.loginfo(self.name + ": Node initialized")
        return

    def _callback_pose(self, uav_pose_stamped):
        # type: (PoseStamped) -> None
        """Callback function for the subscription to /mavros/local_position/pose."""
        self.uav_pose = uav_pose_stamped.pose
        return

    def _callback_map(self, obstacleList):
        # type: (obstacleListMsg) -> None
        """Callback function for the subscription to ~/obstacle_map."""
        self._map = obstacleList.obstacleList
        return

    def _callback_waypoint_global_current(self, waypoint_global):
        # type: (Waypoint) -> None
        self.waypoint_global_next = waypoint_global
        return

    def _callback_waypoint_global_previous(self, waypoint_global):
        # type: (Waypoint) -> None
        self.waypoint_global_previous = waypoint_global
        return

    def _publish_setpoint(self):
        """Publisher Function that publishes self.new_wp in the PoseStamped format
        to mavros/setpoint_position/local."""

        while self.setpoint_local is None:
            self._rate_publish.sleep()

        while not rospy.is_shutdown():
            self._pub_setpoint.publish(self.setpoint_local)
            self._rate_publish.sleep()

        return

    def _get_uav_position(self, pose=None):
        # type: (Pose) -> np.ndarray
        if pose is None:
            uav_pose = copy.copy(self.uav_pose)
            return np.array([uav_pose.position.x, uav_pose.position.y, uav_pose.position.z])
        else:
            if not isinstance(pose, Pose):
                raise Warning("Argument pose must be of type Pose. Was of type: " + type(pose))
            return np.array([pose.position.x, pose.position.y, pose.position.z])

    def _get_uav_yaw(self, pose=None):
        # type: (Pose) -> float
        if pose is None:
            uav_pose = copy.copy(self.uav_pose)
            quaternion = (
                uav_pose.orientation.x, uav_pose.orientation.y, uav_pose.orientation.z, uav_pose.orientation.w)
            yaw = euler_from_quaternion(quaternion)[2]
            return yaw
        else:
            if not isinstance(pose, Pose):
                raise Warning("Argument pose must be of type Pose. Was of type: " + type(pose))
            quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
            yaw = euler_from_quaternion(quaternion)[2]
            return yaw

    @staticmethod
    def calc_angle(vector, ref_axis=np.array([1, 0, 0]), plane_normal=np.array([0, 0, 1])):
        """Angle between reference axis and the vector"""
        vector = np.asarray(vector)
        ref_axis = np.asarray(ref_axis)
        plane_normal = np.asarray(plane_normal)

        vector_len = np.linalg.norm(vector)
        ref_axis_len = np.linalg.norm(ref_axis)

        cross = np.cross(ref_axis, vector)
        angle = np.arccos(vector.dot(ref_axis) / (vector_len * ref_axis_len)) * np.sign(plane_normal.dot(cross))

        return angle

    @staticmethod
    def calc_tangential(vector_normal, vector_direction):
        """https://www.maplesoft.com/support/help/Maple/view.aspx?path=MathApps%2FProjectionOfVectorOntoPlane"""
        vector_normal_normalized = copy.copy(vector_normal)/np.linalg.norm(vector_normal)
        tang_in_direction_of_path = vector_direction - (vector_direction.dot(vector_normal_normalized)) \
                                    * vector_normal_normalized

        return tang_in_direction_of_path

    @staticmethod
    def calc_point_projected_on_line(vector_x, vector_a, vector_b):
        """Returns the closest point on the vector through a and b when projecting x on that line.
        https://stackoverflow.com/questions/5227373/minimal-perpendicular-vector-between-a-point-and-a-line"""
        direction = (vector_b - vector_a) / np.linalg.norm((vector_b - vector_a))
        vector_proj = vector_a + ((vector_x - vector_a).dot(direction)) * direction

        return vector_proj

    def _get_waypoint_global_position(self):
        # type: (LocalPathPlanner) -> (np.ndarray, np.ndarray)
        """Returns the x,y,z coordinates of the current and previous global waypoint in a tuple
        (np.array(x_cur,y_cur,z_cur), np.array(x_cur,y_cur,z_cur))"""

        waypoint_global_next_position = np.array([self.waypoint_global_next.x_lat,
                                                  self.waypoint_global_next.y_long,
                                                  self.waypoint_global_next.z_alt])

        waypoint_global_previous_position = np.array([self.waypoint_global_previous.x_lat,
                                                      self.waypoint_global_previous.y_long,
                                                      self.waypoint_global_previous.z_alt])

        return waypoint_global_next_position, waypoint_global_previous_position

    def apf_new_setpoint(self, goal_waypoint=None, ctrl='velocity'):
        # type: (Waypoint, str) -> PositionTarget
        if goal_waypoint is None:
            goal_pos = self._get_waypoint_global_position()[0]
        else:
            goal_pos = np.array([goal_waypoint.x_lat,
                                 goal_waypoint.y_long,
                                 goal_waypoint.z_alt])

        uav_pos = self._get_uav_position()

        setpoint = PositionTarget()
        setpoint.type_mask = 0b111111111000
        setpoint.coordinate_frame = 1
        setpoint.header = Header()
        setpoint.header.stamp = rospy.Time.now()
        setpoint.position.x = uav_pos[0]
        setpoint.position.y = uav_pos[1]
        setpoint.position.z = self.waypoint_global_next.z_alt  # ToDo: make 3D

        step_size = self.step_size
        for i in range(self.max_iter_per_wp):
            gradient_srv_req = potential_field_msgRequest()
            gradient_srv_req.req.data = self._get_uav_position()
            gradient_srv_req.mode = 0
            gradient_srv_resp = self.client_apf_gradient(gradient_srv_req)

            gradient = deseralize_coordinates(gradient_srv_resp.resp.data, 3)

            direction = -gradient[0]  # Subtraction because we want to descent / move to smaller potential
            direction[2] = 0  # ToDo: make 3D

            # ToDo: handle direction_length close to 0
            direction_length = np.linalg.norm(direction)  # ToDo: adapt for 3D

            if ctrl == 'velocity':
                setpoint = PositionTarget()

                # Deal with small step size
                if np.isclose(direction_length, 0):
                    setpoint.type_mask = 0b111111000011
                    setpoint.type_mask = 0b111111000011
                else:
                    setpoint.type_mask = 0b101111000011
                    setpoint.yaw = self.calc_angle(
                        vector=direction)  # np.arccos((direction[:2].dot(np.array([1, 0])) / direction_length))

                setpoint.coordinate_frame = 1
                setpoint.header = Header()
                setpoint.header.stamp = rospy.Time.now()

                # ToDo: handle direction_length close to 0
                velocity = direction * self.speed_multiplier  # / direction_length

                if np.linalg.norm(velocity[:2]) > self.speed_max_xy:
                    velocity[:2] = velocity[:2] / np.linalg.norm(velocity[:2]) * self.speed_max_xy

                if velocity[2] > self.speed_max_z:
                    velocity[2] = velocity[2] / np.abs(velocity[2]) * self.speed_max_z  # preserves sign

                setpoint.velocity.x = velocity[0]
                setpoint.velocity.y = velocity[1]
                setpoint.velocity.z = 0
                setpoint.position.z = self.waypoint_global_next.z_alt  # ToDo: make 3D
                break

            elif ctrl == 'position':
                potential_srv_req = potential_field_msgRequest()
                potential_srv_req.mode = 0
                potential_srv_req.req.data = uav_pos  # ToDo if error: Maybe needs to be transformed to list

                potential_srv_resp = self.client_apf_potential(potential_srv_req)
                potential_here = potential_srv_resp.resp.data

                # ToDo: handle direction_length close to 0
                # if np.linalg.norm(direction) > 1:
                #     direction = direction / np.linalg.norm(direction)
                uav_pos_new = uav_pos + step_size * direction  # / direction_length

                potential_srv_req = potential_field_msgRequest()
                potential_srv_req.mode = 0
                potential_srv_req.req.data = uav_pos_new  # ToDo if error: Maybe needs to be transformed to list
                potential_srv_resp = self.client_apf_potential(potential_srv_req)
                potential_new = potential_srv_resp.resp.data

                # ToDo if error: Maybe needs to access the first element explicitly: potential_new[0]
                if potential_new[0] > potential_here[0]:
                    step_size /= 2
                else:
                    setpoint = PositionTarget()

                    # Deal with small step size
                    if np.isclose(direction_length, 0):
                        setpoint.type_mask = 0b111111111000
                    else:
                        setpoint.type_mask = 0b101111111000
                        setpoint.yaw = self.calc_angle(
                            vector=direction)  # np.arccos((direction[:2].dot(np.array([1, 0])) / direction_length))

                    setpoint.coordinate_frame = 1
                    setpoint.header = Header()
                    setpoint.header.stamp = rospy.Time.now()
                    setpoint.position.x = uav_pos_new[0]
                    setpoint.position.y = uav_pos_new[1]
                    setpoint.position.z = self.waypoint_global_next.z_alt  # ToDo: make 3D
                    break

            else:
                rospy.error("Wrong setpoint kind")

        return setpoint

    def apf_bug_setpoint(self, uav_pos, uav_potential_target_obs_only, direction_bug, vector_waypoints):

        potential_srv_req = potential_field_msgRequest()
        potential_srv_req.req.data = uav_pos
        potential_srv_req.mode = 1
        gradient_srv_resp = self.client_apf_potential(potential_srv_req)
        potential_obs_only = gradient_srv_resp.resp.data[0]
        gradient_srv_resp = self.client_apf_gradient(potential_srv_req)
        gradient_obs_only = deseralize_coordinates(gradient_srv_resp.resp.data, 3)[0]
        # gradient_obs_only[2] = 0

        tang = self.calc_tangential(vector_normal=gradient_obs_only, vector_direction=direction_bug)

        uav_pos_new = uav_pos + tang / np.linalg.norm(tang) * self.bug_step_size

        potential_srv_req = potential_field_msgRequest()
        potential_srv_req.req.data = uav_pos_new
        potential_srv_req.mode = 1
        potential_srv_resp = self.client_apf_potential(potential_srv_req)
        gradient_srv_resp = self.client_apf_gradient(potential_srv_req)
        potential_new_obs_only = potential_srv_resp.resp.data[0]
        gradient_new_obs_only = deseralize_coordinates(gradient_srv_resp.resp.data, 3)[0]
        gradient_new_obs_only[2] = 0

        potential_delta = uav_potential_target_obs_only - potential_new_obs_only

        # If the correction step is larger than some fixed step length, scale it to the step length. This prevents
        # overshooting in areas where the second derivative is larger and the first order Taylor approximation applied
        # here is not precise
        correction_step = potential_delta * gradient_new_obs_only / np.linalg.norm(gradient_new_obs_only) ** 2
        factor = 1
        if np.linalg.norm(correction_step) > factor * self.bug_step_size:
            correction_step = correction_step * factor * self.bug_step_size / np.linalg.norm(correction_step)

        uav_pos_new = uav_pos_new + correction_step

        is_orthogonal = False
        vector_waypoints[2] = 0
        gradient_obs_only_2d = copy.copy(gradient_obs_only)
        vector_waypoints_2d = copy.copy(vector_waypoints)
        gradient_obs_only_2d[2] = 0
        vector_waypoints_2d[2] = 0
        if np.isclose(np.abs(self.calc_angle(gradient_obs_only_2d, ref_axis=vector_waypoints_2d,
                                             plane_normal=np.array((0, 0, 1)))),
                      np.pi, atol=.1):
            is_orthogonal = True
            rospy.loginfo("Gradient orthogonal to original path")

        return uav_pos_new, is_orthogonal

    def determine_progress(self):
        """Sets self.uav_is_stalled to True, if there was in no progress in direction of the goal point in the last
        self.mode_1_2_time_limit seconds"""
        t = threading.current_thread()

        function_name = "UAV Progress Determination"
        # ToDo: if goal_point changes, reset
        rospy.loginfo(function_name + ": Thread started")
        time_last_progress = rospy.get_rostime().secs
        goal_pos = self._get_waypoint_global_position()[0]
        best_uav_pos = self._get_uav_position()

        while not rospy.is_shutdown() and getattr(t, "do_run", True):
            uav_pos = self._get_uav_position()
            if np.linalg.norm(uav_pos - goal_pos) < np.linalg.norm(best_uav_pos - goal_pos):
                best_uav_pos = uav_pos
                time_last_progress = rospy.get_rostime().secs

            if rospy.get_rostime().secs - time_last_progress < self.mode_1_2_time_limit:
                self.uav_is_stalled = False
            else:
                if not self.uav_is_stalled:
                    rospy.loginfo(function_name + ": UAV has stalled")
                self.uav_is_stalled = True

            self._rate_publish.sleep()

        rospy.loginfo(function_name + ": Thread stopped")
        return

    def apf_local_minima_destroyer(self):
        uav_pos = self._get_uav_position()
        waypoint_global_next_pos, waypoint_global_previous_pos = self._get_waypoint_global_position()

        potential_srv_req = potential_field_msgRequest()
        potential_srv_req.req.data = uav_pos
        potential_srv_req.mode = 1  # Only get potential caused by obstacles
        potential_srv_resp = self.client_apf_potential(potential_srv_req)
        uav_pos_potential_obs_only = potential_srv_resp.resp.data[0]

        potential_srv_req = potential_field_msgRequest()
        potential_srv_req.req.data = uav_pos
        potential_srv_req.mode = 0
        potential_srv_resp = self.client_apf_potential(potential_srv_req)
        uav_pos_potential = potential_srv_resp.resp.data[0]

        initial_vector_to_path = self.calc_point_projected_on_line(uav_pos, waypoint_global_next_pos,
                                                                   waypoint_global_previous_pos) - uav_pos

        # initial_vector_to_path[2] = 0
        # If the stalled position is on the line between the waypoints, the initial_vector_to_path can be not orthogonal
        # to that line. In this case a new orthogonal vector is calculated, that randomly (through numerical errors)
        # points either to the right or left. z is defaultly set to 0.

        vector_inter_waypoints_global = waypoint_global_next_pos - waypoint_global_previous_pos
        if any(np.isclose(initial_vector_to_path, 0)):
            initial_vector_to_path = np.zeros(3)
            if np.isclose(vector_inter_waypoints_global[0], 0):
                initial_vector_to_path[1] = 1
            elif np.isclose(vector_inter_waypoints_global[1], 0):
                initial_vector_to_path[0] = 1
            else:
                initial_vector_to_path[0] = 1
                initial_vector_to_path[1] = -vector_inter_waypoints_global[0] / vector_inter_waypoints_global[1]

        # Indicates wether the UAV has already turned around
        dir_reversed = False

        escape_trigger = False
        escape_trigger_pos = None
        extra_distance = .1

        while not rospy.is_shutdown():  # Exits when timeout or distance threshold reached or an escape point is found
            uav_pos = self._get_uav_position()
            is_escaped, _ = self.gradient_descent(start_pos=uav_pos, initial_potential=uav_pos_potential,
                                                  abort_early=True)

            if is_escaped and not escape_trigger:
                rospy.loginfo("Found escape. Moving a bit further.")
                escape_trigger = True
                escape_trigger_pos = uav_pos

            uav_pos_setpoint, is_orthogonal = self.apf_bug_setpoint(uav_pos=uav_pos,
                                                                    uav_potential_target_obs_only=uav_pos_potential_obs_only,
                                                                    direction_bug=initial_vector_to_path,
                                                                    vector_waypoints=vector_inter_waypoints_global)

            distance_to_path = np.linalg.norm(self.calc_point_projected_on_line(uav_pos_setpoint,
                                                                                waypoint_global_next_pos,
                                                                                waypoint_global_previous_pos)
                                              - uav_pos_setpoint)

            if distance_to_path >= 1.1 * self.bug_deviation_from_path_max:
                rospy.loginfo("Too far away from path. Emergency Landing")  # Send trigger to global path for landing
                return False

            elif escape_trigger:
                if np.linalg.norm((escape_trigger_pos - uav_pos)) >= extra_distance:
                    return True

            elif not dir_reversed and distance_to_path >= .9 * self.bug_deviation_from_path_max:  # and time below timeout; make max deviation a ROS Param, defaults to inter waypoint distance
                rospy.loginfo("Reversing direction")
                dir_reversed = True
                initial_vector_to_path = -initial_vector_to_path

            elif dir_reversed and distance_to_path >= self.bug_deviation_from_path_max:  # and time below timeout; make waypoint global distance a ROS Param, defaults to inter waypoint distance
                rospy.loginfo(
                    "No escape found, target unrechable. Trying next global waypoint")  # Send trigger to global path to change global waypoints
                return False

            setpoint = PositionTarget()
            setpoint.type_mask = 0b101111111000
            setpoint.coordinate_frame = 1
            setpoint.header = Header()
            setpoint.header.stamp = rospy.Time.now()
            setpoint.position.x = uav_pos_setpoint[0]
            setpoint.position.y = uav_pos_setpoint[1]
            setpoint.position.z = self.waypoint_global_next.z_alt  # ToDo: make 3D
            setpoint.yaw = self.calc_angle(
                vector=uav_pos_setpoint - uav_pos)
            self.setpoint_local = setpoint

            self._rate_publish.sleep()

    def gradient_descent(self, start_pos, initial_potential, abort_early=False):

        pos = start_pos
        for ii in range(self.bug_gd_max_iter):
            step_size = self.bug_gd_step_size
            potential_srv_req = potential_field_msgRequest()
            potential_srv_req.req.data = pos
            potential_srv_req.mode = 0
            potential_srv_resp = self.client_apf_potential(potential_srv_req)
            potential = potential_srv_resp.resp.data[0]

            gradient_srv_resp = self.client_apf_gradient(potential_srv_req)
            descent_dir = -deseralize_coordinates(gradient_srv_resp.resp.data, 3)[0]
            descent_dir[2] = 0  # Only move in xy-plane

            valid = False
            while step_size * (2 ** 11) > self.bug_gd_step_size:
                pos_new = pos + step_size * descent_dir

                potential_srv_req = potential_field_msgRequest()
                potential_srv_req.req.data = pos_new
                potential_srv_req.mode = 0
                potential_srv_resp = self.client_apf_potential(potential_srv_req)
                potential_new = potential_srv_resp.resp.data[0]

                if potential_new < potential:
                    valid = True
                    pos = pos_new
                    potential = potential_new
                    break
                else:
                    step_size /= 2

            if abort_early and potential < .8 * initial_potential:
                return True, pos

            elif not valid:
                return False, pos

        return False, pos

    def master(self):
        mode = 1  # 0: direct, 1: APF, 2: APF_Star

        # start progression watcher
        thread_uav_progression = Thread(target=self.determine_progress, args=())
        thread_uav_progression.daemon = True
        thread_uav_progression.start()

        self._thread_publish_setpoint.start()

        while not rospy.is_shutdown():
            if not thread_uav_progression.is_alive():
                thread_uav_progression = Thread(target=self.determine_progress, args=())
                thread_uav_progression.daemon = True
                thread_uav_progression.start()

            uav_pos = self._get_uav_position()
            uav_yaw = self._get_uav_yaw()
            goal_coordinate = self._get_waypoint_global_position()[0]

            # ToDo: Find heuristic to determine if to use position or velocity control
            setpoint_apf = self.apf_new_setpoint(ctrl='position')
            # setpoint_direct = self.direct_new_setpoint()

            if self.bug_mode_allowed and self.uav_is_stalled:
                rospy.loginfo(self.name + ": UAV is in a local minima. Starting APF_Star.")
                thread_uav_progression.do_run = False
                self.uav_is_stalled = False

                # Turn UAV in a circle to get the entire environment
                setpoint = PositionTarget()
                setpoint.type_mask = 0b101111111000
                setpoint.coordinate_frame = 1
                setpoint.header = Header()
                setpoint.header.stamp = rospy.Time.now()
                setpoint.position.x = uav_pos[0]
                setpoint.position.y = uav_pos[1]
                setpoint.position.z = self.waypoint_global_next.z_alt  # ToDo: make 3D
                rospy.loginfo(self.name + ": Scanning 360 environment")
                for angle_i in np.linspace(uav_yaw, uav_yaw + 2 * np.pi, 30):
                    setpoint.yaw = angle_i
                    self.setpoint_local = setpoint
                    rospy.sleep(.1)

                rospy.sleep(.5)

                rospy.loginfo(self.name + ": Starting Bug Mode")
                success = self.apf_local_minima_destroyer()

                if success:
                    rospy.loginfo(self.name + ": Escaped local minima")
                else:
                    rospy.loginfo(self.name + ": Escaping local minima failed")
                    while not rospy.is_shutdown():
                        uav_pos = self._get_uav_position()
                        setpoint = PositionTarget()
                        setpoint.type_mask = 0b101111111000
                        setpoint.coordinate_frame = 1
                        setpoint.header = Header()
                        setpoint.header.stamp = rospy.Time.now()
                        setpoint.position.x = uav_pos[0]
                        setpoint.position.y = uav_pos[1]
                        setpoint.position.z = self.waypoint_global_next.z_alt  # ToDo: make 3D
                        self.setpoint_local = setpoint

                rospy.loginfo(self.name + ": Returning to normal control")

            # If UAV is close to global waypooint, do position setpoint control
            if all(np.isclose(uav_pos[:2], goal_coordinate[:2], atol=self.threshold_wp_position_control)):
                setpoint = PositionTarget()
                setpoint.type_mask = 0b111111111000
                setpoint.coordinate_frame = 1
                setpoint.header = Header()
                setpoint.header.stamp = rospy.Time.now()
                setpoint.position.x = self.waypoint_global_next.x_lat
                setpoint.position.y = self.waypoint_global_next.y_long
                setpoint.position.z = self.waypoint_global_next.z_alt
                self.setpoint_local = setpoint

            elif mode == 1:
                self.setpoint_local = setpoint_apf

            self._rate_publish.sleep()

        return

    def start(self):
        """main function of LocalPathPlanner"""

        rospy.loginfo(self.name + ": Node started")

        rospy.sleep(1)
        while not rospy.is_shutdown():
            if self.uav_pose is None:
                rospy.loginfo(self.name + ": Waiting for UAV Pose")
                self._rate_publish.sleep()
            else:
                rospy.loginfo(self.name + ": UAV Pose received")
                break

        rospy.loginfo(self.name + ": Waiting for global waypoints")
        while (self.waypoint_global_next is None or self.waypoint_global_previous is None) \
                and not rospy.is_shutdown():
            self._rate_publish.sleep()
        rospy.loginfo(self.name + ": Global waypoints received")

        # Wait for services
        rospy.wait_for_service('get_apf_potential')
        rospy.wait_for_service('get_apf_gradient')
        rospy.loginfo(self.name + ': Services online')

        rospy.loginfo(self.name + ': Starting local path planning')

        if self.bug_mode_allowed:
            rospy.loginfo(self.name + ": Local Minima Mode activated")
        else:
            rospy.loginfo(self.name + ": Local Minima Mode NOT activated")

        self.master()

        return

    #######################
    # NOT IMPLEMENTED YET #
    #######################

    def direct_new_setpoint(self):
        goal_coordinate = self._get_waypoint_global_position()[0]
        uav_coordinate = self._get_uav_position()

        direction = goal_coordinate - uav_coordinate
        direction_length = np.linalg.norm(direction)

        setpoint = PositionTarget()
        setpoint.type_mask = 0b101111000011
        setpoint.coordinate_frame = 1
        setpoint.header = Header()
        setpoint.header.stamp = rospy.Time.now()

        velocity = direction / direction_length ** 2 * self.speed_multiplier
        setpoint.velocity.x = velocity[0]
        setpoint.velocity.y = velocity[1]
        setpoint.position.z = velocity[2]

        setpoint.yaw = np.arccos((direction[:2].dot(np.array([1, 0])) / np.linalg.norm(direction[:2])))

        # def get_intersection_point(X1, X2, Y):
        #     """X1 is start of line, X2 end of line, Y is point"""
        #     vec_X2_X1_norm = (X2 - X1) / np.linalg.norm(X2 - X1) # vector from X1 to X2 normalized
        #     vec_Y_X1 = Y - X1  # vector from X1 to Y
        #     t = vec_Y_X1.dot(vec_X2_X1_norm)
        #     vec_intersection = X1 + t * vec_X2_X1_norm
        #     return vec_intersection
        #
        # wp_global_current, wp_global_prev = self.__get_waypoint_global_position()
        # uav_pos = self.__get_uav_position()
        #
        # # Returns the distance of the uav position to the line between the current and previous global waypoint
        # dist_from_global_path = np.abs(np.cross(wp_global_current-wp_global_prev, wp_global_prev-uav_pos))
        # /np.linalg.norm(wp_global_current-wp_global_prev)
        #
        # if dist_from_global_path > self.tol_dev_global_path:
        #     pass

        return setpoint

    def apf_star_new_setpoint(self):
        #     class APFPosition:
        #         def __init__(self, parent, position, depth):
        #             self.parent = parent  # type: APFPosition
        #             self.children = list()  # type: typing.List[APFPosition]
        #             self.position = position
        #             self.depth = depth
        #             self.potential = get_potential_field(position, goal_coordinate=waypoint_global_next_position,
        #                                                  obstacle_map=obs_map)
        #             self.gradient = get_vector_field(position, goal_coordinate=waypoint_global_next_position,
        #                                              obstacle_map=obs_map)
        #             self.reached_potential = np.inf
        #
        #         def gradient_descent(self, save_reached_potential=True, goal_coordinate=None, obstacle_map=None,
        #                              max_iter=1000, tolerance=0.1):
        #             if goal_coordinate is None:
        #                 goal_coordinate = waypoint_global_next_position
        #
        #             if obstacle_map is None:
        #                 obstacle_map = obs_map
        #
        #             pts_hist = np.zeros((max_iter, 3))
        #             pot_hist = np.zeros(max_iter)
        #
        #             pts_hist[0] = self.position
        #             pot_hist[0] = get_potential_field(pts_hist[0], goal_coordinate=goal_coordinate,
        #                                               obstacle_map=obstacle_map)
        #
        #             k = 0
        #             for ii in range(max_iter - 1):
        #                 step_size = descent_step
        #
        #                 neg_grad = -get_vector_field(pts_hist[k], goal_coordinate=goal_coordinate,
        #                                              obstacle_map=obstacle_map)
        #
        #                 # ToDo: New
        #                 while step_size * (2 ** 11) > descent_step:
        #                     new_pt = pts_hist[k] + step_size * neg_grad
        #                     new_potential = get_potential_field(new_pt, goal_coordinate=goal_coordinate,
        #                                                         obstacle_map=obstacle_map)
        #                     if new_potential < pot_hist[k]:
        #                         pts_hist[k + 1] = new_pt
        #                         pot_hist[k + 1] = new_potential
        #                         k += 1
        #                         break
        #
        #                     else:
        #                         step_size /= 2
        #
        #                 if all(np.isclose(pts_hist[k], goal_coordinate, atol=tolerance)):
        #                     break
        #
        #             if save_reached_potential:
        #                 self.reached_potential = pot_hist[k]  # save reached potential
        #
        #             pts_hist = pts_hist[:k + 1]
        #             pot_hist = pot_hist[:k + 1]
        #
        #             return pts_hist, pot_hist
        #
        #         def expand(self):
        #             if self.parent is None:
        #                 vector_root_node = self.position - uav_pos
        #             else:
        #                 vector_root_node = self.position - self.parent.position
        #             vector_root_node = vector_root_node / np.linalg.norm(vector_root_node) * radius_steps
        #             position1 = self.position + vector_root_node
        #             position2 = self.position + R(alpha).dot(vector_root_node)
        #             position3 = self.position + R(-alpha).dot(vector_root_node)
        #
        #             newNode1 = APFPosition(self, position1, self.depth + 1)
        #             newNode2 = APFPosition(self, position2, self.depth + 1)
        #             newNode3 = APFPosition(self, position3, self.depth + 1)
        #
        #             # Check if new nodes reachable with apf
        #             for node_i in (newNode1, newNode2, newNode3):
        #                 pts, pot = self.gradient_descent(save_reached_potential=False, goal_coordinate=node_i.position, max_iter=max_iter_wp_wp, tolerance=tol_wp_local_simulated)
        #                 if all(np.isclose(pts[-1], node_i.position, atol=tol_wp_local_simulated)):
        #                     self.children.append(node_i)
        #             return
        #
        #     def bfs():
        #         import Queue
        #         # initilalize
        #         active_nodes = Queue.Queue()
        #         for angle in np.linspace(uav_yaw, uav_yaw + (2 - 2 / n_pts) * np.pi, n_pts):
        #             position = uav_pos + R(angle).dot(np.array([radius_steps, 0, 0]))
        #             node = APFPosition(None, position, 0)
        #             active_nodes.put(node)
        #
        #         while not active_nodes.empty():
        #             node_i = active_nodes.get()
        #             # Only visit a node if its potential and depth is below the upper bound
        #             if node_i.potential < potential_ub \
        #                     and node_i.depth <= depth_ub \
        #                     and np.linalg.norm(node_i.position - uav_pos) <= max_radius:
        #                 node_i.gradient_descent(save_reached_potential=True, goal_coordinate=waypoint_global_next_position,
        #                                         obstacle_map=obs_map, max_iter=max_iter_wp_goal, tolerance=tol_wp_global_reached)
        #                 # If the node reaches a potential below the threshold return that node
        #                 if node_i.reached_potential < ((1 - potential_percentage_increase) * uav_potential
        #                                                + potential_percentage_increase * goal_potential):
        #                     return node_i
        #                 # If it is above the threshold, expand
        #                 else:
        #                     node_i.expand()
        #                     for child in node_i.children:
        #                         active_nodes.put(child)
        #
        #         return None
        #
        #     function_name = "Local Minima Evasion"
        #
        #     uav_pos = self.__get_uav_position()
        #     uav_yaw = self.__get_uav_yaw()
        #
        #     obs_map = self._map
        #     waypoint_global_next_position, waypoint_global_previous_position = self.__get_waypoint_global_position()
        #     tol_wp_global_reached = self.tol_wp_global_reached
        #     tol_wp_local_simulated = self.tol_wp_local_simulated
        #
        #     max_radius = 2 * np.linalg.norm(waypoint_global_next_position - waypoint_global_previous_position)
        #
        #     max_iter_wp_wp = 100
        #     max_iter_wp_goal = 1000
        #     depth_ub = 6
        #
        #     # Set the maximum allowed potential for a node to expand equal to twice the potential of the previous global
        #     # waypoint. This should theoretically allow the graph to expand to a radius around the current global waypoint
        #     # equal to twice the length between the previous and current global waypoint.
        #     potential_ub = 4 * get_potential_field(uav_pos, goal_coordinate=self.__get_waypoint_global_position()[1],
        #                                            obstacle_map=self._map)
        #
        #     descent_step = self.step_size  # step size for gradient descent of nodes
        #     # Percentage by which the reached potential of the new node has to be better
        #     potential_percentage_increase = 0.2
        #     radius_steps = 1
        #     n_pts = 9  # number of points for the first layer
        #     n_exp = 3  # Not used. Nodes to be expanded from each node. So far only hardcoded
        #     alpha = 45. / 360. * 2 * np.pi  # angle between the vector between child node to parent node
        #     # and parent node - parent's parent node
        #     R = lambda x: np.array([[np.cos(x), np.sin(x), 0], [-np.sin(x), np.cos(x), 0], [0, 0, 1]])  # Rotation matrix
        #
        #     uav_potential = get_potential_field(uav_pos, goal_coordinate=waypoint_global_next_position,
        #                                         obstacle_map=self._map)
        #     goal_potential = get_potential_field(waypoint_global_next_position,
        #                                          goal_coordinate=waypoint_global_next_position,
        #                                          obstacle_map=self._map)
        #
        #     result = None
        #     escape_node = bfs()
        #     if escape_node is None:
        #         rospy.loginfo(function_name + ": UAV potential {0}".format(uav_potential))
        #         rospy.loginfo(function_name + ": Goal potential {0}".format(goal_potential))
        #         rospy.loginfo(function_name + ": Target unreachable")
        #         # ToDo: call "not reachable"
        #         pass
        #     else:
        #         result = WaypointList()
        #         # initialize list where nodes are put
        #         nodes = [None] * (escape_node.depth + 1)  # type: typing.List[APFPosition]
        #         current_node = escape_node
        #         # find the root node
        #         i = escape_node.depth
        #         while current_node is not None:
        #             nodes[i] = current_node
        #             current_node = current_node.parent
        #             i -= 1
        #         if i != -1: rospy.loginfo(function_name + ": Something went wrong")
        #
        #         # append all nodes from the root node to the node returned by bfs
        #         for node_j in nodes:
        #             wpt = Waypoint()
        #             wpt.x_lat = node_j.position[0]
        #             wpt.y_long = node_j.position[1]
        #             wpt.z_alt = node_j.position[2]
        #             result.waypoints.append(wpt)
        #
        #         rospy.loginfo(result)
        #         rospy.loginfo(function_name + ": UAV potential {0}".format(uav_potential))
        #         rospy.loginfo(function_name + ": Goal potential {0}".format(goal_potential))
        #         rospy.loginfo(function_name + ": Last setpoint potential {0}".format(nodes[-1].reached_potential))
        #
        #     # ToDo: APF mit neuer wpts-liste starten
        #     return result
        pass

    @staticmethod
    def _speed_towards_obstacle_center(uav_pos, setpoint, obstacle):
        # type: (np.ndarray, PositionTarget, obstacleMsg) -> float
        """Returns the speed with which the uav moves towards the center of obstacle"""
        uav_velocity = np.array([setpoint.velocity.x, setpoint.velocity.y, setpoint.velocity.z])
        obstacle_position = np.array(obstacle.pose)

        vector_uav_obs = obstacle_position - uav_pos
        vector_uav_obs_len = np.linalg.norm(vector_uav_obs)

        uav_velocity_projected = (uav_velocity.dot(vector_uav_obs) / vector_uav_obs_len) \
                                 * vector_uav_obs / vector_uav_obs_len
        uav_velocity_projected_len = np.linalg.norm(uav_velocity_projected)
        return uav_velocity_projected_len

    # ToDo: finish method
    @staticmethod
    def setpoint_failsafe(setpoint):
        # type: (PositionTarget) -> bool
        """Check if setpoint is safe to send"""
        # ToDo: Implement
        if setpoint.type_mask == 0b111111111000:  # Position Control
            pass
        elif setpoint.type_mask == 0b101111000011:  # Velocity and Yaw Control
            pass

        return False

    def determine_reachability(self):
        """Determine if the current global waypoint is reachable"""
        # ToDo: Implement
        pass


def serialize_coordinates(array_2d):
    # type: (np.ndarray) -> np.ndarray
    array_2d = np.asarray(array_2d)
    array_1d = np.array(array_2d).ravel()
    return array_1d


def deseralize_coordinates(array_1d, n_pts):
    # type: (np.ndarray, int) -> np.ndarray
    array_1d = np.asarray(array_1d)
    array_size = len(array_1d)
    if not array_size % n_pts == 0:
        raise Warning("Array size must be a clean multiple of n_pts. Array length: {0}, n_pts: {1}".format(array_size,
                                                                                                           n_pts))
    array_2d = array_1d.reshape((array_size / n_pts, n_pts))
    return array_2d
