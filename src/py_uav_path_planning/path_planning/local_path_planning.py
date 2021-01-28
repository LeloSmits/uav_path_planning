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
        self.allow_3D = rospy.get_param('apf_3D', False)
        self.control_type = rospy.get_param('apf_ctrl_type', 'position')
        self.speed_max_xy = .5  # type: float  # Maximum speed in xy plane for velocity ctrl
        self.speed_max_z = .5  # type: float  # Maximum speed in z plane for velocity ctrl
        self.speed_multiplier = 1.  # type: float  # ToDo: Make adaptive
        self.step_size_max = rospy.get_param('step_size_max', 1.)
        self.max_iter_per_wp = rospy.get_param('apf_max_iter', 100)

        # Local Minima Bug-APF
        # ToDo: Make PARAM
        self.bug_mode_allowed = rospy.get_param('bug_mode_allowed', True)
        self.bug_step_size = rospy.get_param('bug_step_size', 1.)
        self.bug_timeout = rospy.get_param('bug_timeout', .5)  # seconds
        self.bug_deviation_from_path_max = rospy.get_param('bug_deviation_from_path_max', 20.)
        self.bug_gd_step_size = rospy.get_param('bug_gd_step_size', self.step_size_max)
        self.bug_gd_max_iter = rospy.get_param('bug_gd_max_iter', 100)
        self.bug_correction_factor = rospy.get_param('bug_correction_factor', 1)

        self.uav_is_stalled_time_limit = rospy.get_param('uav_is_stalled_time_limit', 5)  # secs
        self.uav_is_stalled = False

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
        rospy.loginfo(self.name + ': Potential field services online')

        rospy.loginfo(self.name + ": Local Minima Mode activated:\t{0}".format(self.bug_mode_allowed))
        rospy.loginfo(self.name + ": 3D Mode activated:\t{0}".format(self.allow_3D))
        rospy.loginfo(self.name + ": Control type:\t{0}".format(self.control_type))

        rospy.loginfo(self.name + ': Starting local path planning')
        self.master()

        return

    def master(self):
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
            waypoint_global_next_pos = self._get_waypoint_global_position()[0]

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
                setpoint.position.z = uav_pos[2]  # ToDo: make 3D - CHECK
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
                    rospy.loginfo(self.name + ": Escaping local minima failed")  # ToDo: Land
                    while not rospy.is_shutdown():
                        uav_pos = self._get_uav_position()
                        setpoint = PositionTarget()
                        setpoint.type_mask = 0b111111111000
                        setpoint.coordinate_frame = 1
                        setpoint.header = Header()
                        setpoint.header.stamp = rospy.Time.now()
                        setpoint.position.x = uav_pos[0]
                        setpoint.position.y = uav_pos[1]
                        setpoint.position.z = uav_pos[2]  # ToDo: make 3D - CHECK
                        self.setpoint_local = setpoint

                rospy.loginfo(self.name + ": Returning to normal control")

            # If UAV is close to global waypooint, do position setpoint control
            if all(np.isclose(uav_pos[:2], waypoint_global_next_pos[:2], atol=self.threshold_wp_position_control)):
                setpoint = PositionTarget()
                setpoint.type_mask = 0b111111111000
                setpoint.coordinate_frame = 1
                setpoint.header = Header()
                setpoint.header.stamp = rospy.Time.now()
                setpoint.position.x = self.waypoint_global_next.x_lat
                setpoint.position.y = self.waypoint_global_next.y_long
                setpoint.position.z = self.waypoint_global_next.z_alt
                self.setpoint_local = setpoint
            # Otherwise use APF-Setpoint
            else:
                self.setpoint_local = self.apf_new_setpoint()

            self._rate_publish.sleep()
        return

    def apf_new_setpoint(self):
        # type: (LocalPathPlanner) -> PositionTarget

        uav_pos = self._get_uav_position()

        # Default Setpoint if the gradient descent below does not find a new setpoint within the maximum iterations
        setpoint = PositionTarget()
        setpoint.type_mask = 0b111111111000
        setpoint.coordinate_frame = 1
        setpoint.header = Header()
        setpoint.header.stamp = rospy.Time.now()
        setpoint.position.x = uav_pos[0]
        setpoint.position.y = uav_pos[1]
        setpoint.position.z = uav_pos[2]  # ToDo: make 3D - CHECK

        step_size = self.step_size_max

        gradient_srv_req = potential_field_msgRequest()
        gradient_srv_req.req.data = uav_pos
        gradient_srv_req.mode = 0
        gradient_srv_resp = self.client_apf_gradient(gradient_srv_req)
        gradient = self.deseralize_coordinates(gradient_srv_resp.resp.data, 3)

        direction = -gradient[0]  # Subtraction because we want to descent / move to smaller potential
        if not self.allow_3D:
            direction[2] = 0  # ToDo: make 3D - CHECK
        direction_length = np.linalg.norm(direction)  # ToDo: adapt for 3D - CHECK?

        for i in range(self.max_iter_per_wp):
            if self.control_type == 'velocity':
                setpoint = PositionTarget()

                if self.allow_3D:
                    # Deal with small step size
                    if np.isclose(direction_length, 0):
                        setpoint.type_mask = 0b111111000111
                        setpoint.type_mask = 0b111111000111
                    else:
                        setpoint.type_mask = 0b101111000111
                        setpoint.yaw = self.calc_angle(vector=direction)
                else:
                    # The IGNORE_VEL_Z-Bit has to be set to 0, even though we don't need the z-velocity, only the
                    # z-position. If PX4 gets a setpoint in OFFBOARD mode, where either not all VEL-Bits or POS-Bits are
                    # set to 0, it rejects the setpoint.

                    # Deal with small step size
                    if np.isclose(direction_length, 0):
                        setpoint.type_mask = 0b111111000011
                        setpoint.type_mask = 0b111111000011
                    else:
                        setpoint.type_mask = 0b101111000011
                        setpoint.yaw = self.calc_angle(vector=direction)

                setpoint.coordinate_frame = 1
                setpoint.header = Header()
                setpoint.header.stamp = rospy.Time.now()

                # ToDo: handle direction_length close to 0 and adapt velocity in areas where there a few obstacles
                velocity = direction * self.speed_multiplier  # / direction_length

                # Scale speed so that upper bounds on xy-speed and z-speed are not violated
                if np.linalg.norm(velocity[:2]) > self.speed_max_xy:
                    velocity[:] = velocity / np.linalg.norm(velocity[:2]) * self.speed_max_xy
                if np.abs(velocity[2]) > self.speed_max_z:
                    velocity[:] = velocity / np.abs(velocity[2]) * self.speed_max_z  # preserves sign

                # Insert new
                if self.allow_3D:
                    setpoint.velocity.x = velocity[0]
                    setpoint.velocity.y = velocity[1]
                    setpoint.velocity.z = velocity[2]
                else:
                    setpoint.velocity.x = velocity[0]
                    setpoint.velocity.y = velocity[1]
                    setpoint.velocity.z = 0  # ToDo: Check if z-pos is changed even when velocity is set to 0

                    # Linearilly interpolate desired z-position by using the current distance from the next waypoint and
                    # the distance between the previous and next waypoint
                    waypoint_global_next_pos, waypoint_global_previous_pos = self._get_waypoint_global_position()
                    dist_to_waypoint = np.linalg.norm(uav_pos - waypoint_global_next_pos)
                    dist_btw_waypoints = np.linalg.norm(waypoint_global_next_pos - waypoint_global_previous_pos)
                    setpoint.position.z = dist_to_waypoint / dist_btw_waypoints * waypoint_global_previous_pos[2] \
                                          + (dist_btw_waypoints - dist_to_waypoint) / dist_btw_waypoints \
                                          * waypoint_global_next_pos[2]
                break
            elif self.control_type == 'position':
                potential_srv_req = potential_field_msgRequest()
                potential_srv_req.mode = 0
                potential_srv_req.req.data = uav_pos

                potential_srv_resp = self.client_apf_potential(potential_srv_req)
                potential_here = potential_srv_resp.resp.data

                # ToDo: handle direction_length close to 0 and adapt step width in areas where there a few obstacles
                if direction_length <= 1:
                    uav_pos_new = uav_pos + step_size * direction
                else:
                    uav_pos_new = uav_pos + step_size * direction / direction_length

                potential_srv_req = potential_field_msgRequest()
                potential_srv_req.mode = 0
                potential_srv_req.req.data = uav_pos_new
                potential_srv_resp = self.client_apf_potential(potential_srv_req)
                potential_new = potential_srv_resp.resp.data

                if potential_new[0] > potential_here[0]:
                    step_size /= 2
                else:
                    setpoint = PositionTarget()

                    # Deal with small step size
                    if np.isclose(direction_length, 0):
                        setpoint.type_mask = 0b111111111000
                    else:
                        setpoint.type_mask = 0b101111111000
                        setpoint.yaw = self.calc_angle(vector=direction)

                    setpoint.coordinate_frame = 1
                    setpoint.header = Header()
                    setpoint.header.stamp = rospy.Time.now()
                    setpoint.position.x = uav_pos_new[0]
                    setpoint.position.y = uav_pos_new[1]
                    if self.allow_3D:
                        setpoint.position.z = uav_pos_new[2]  # ToDo: make 3D - CHECK
                    else:
                        # Linearilly interpolate desired z-position by using the current distance from the next waypoint
                        # and the distance between the previous and next waypoint
                        waypoint_global_next_pos, waypoint_global_previous_pos = self._get_waypoint_global_position()
                        dist_to_waypoint = np.linalg.norm(uav_pos - waypoint_global_next_pos)
                        dist_btw_waypoints = np.linalg.norm(waypoint_global_next_pos - waypoint_global_previous_pos)
                        setpoint.position.z = dist_to_waypoint / dist_btw_waypoints * waypoint_global_previous_pos[2] \
                                              + (dist_btw_waypoints - dist_to_waypoint) / dist_btw_waypoints \
                                              * waypoint_global_next_pos[2]
                    break
            else:
                rospy.logerr("Wrong setpoint kind")

        return setpoint

    def apf_bug_setpoint(self, uav_pos, uav_potential_target_obs_only, direction_bug, vector_waypoints):

        potential_srv_req = potential_field_msgRequest()
        potential_srv_req.req.data = uav_pos
        potential_srv_req.mode = 1
        gradient_srv_resp = self.client_apf_potential(potential_srv_req)
        potential_obs_only = gradient_srv_resp.resp.data[0]
        gradient_srv_resp = self.client_apf_gradient(potential_srv_req)
        gradient_obs_only = self.deseralize_coordinates(gradient_srv_resp.resp.data, 3)[0]
        # gradient_obs_only[2] = 0

        tang = self.calc_tangential(vector_normal=gradient_obs_only, vector_direction=direction_bug)

        uav_pos_new = uav_pos + tang / np.linalg.norm(tang) * self.bug_step_size

        potential_srv_req = potential_field_msgRequest()
        potential_srv_req.req.data = uav_pos_new
        potential_srv_req.mode = 1
        potential_srv_resp = self.client_apf_potential(potential_srv_req)
        gradient_srv_resp = self.client_apf_gradient(potential_srv_req)
        potential_new_obs_only = potential_srv_resp.resp.data[0]
        gradient_new_obs_only = self.deseralize_coordinates(gradient_srv_resp.resp.data, 3)[0]
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

            if rospy.get_rostime().secs - time_last_progress < self.uav_is_stalled_time_limit:
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
        uav_pos_z_start = uav_pos[2]
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
        initial_vector_to_path[2] = 0  # Bug should be only 2D

        # If the stalled position is on the line between the waypoints, the initial_vector_to_path can be not orthogonal
        # to that line. In this case a new orthogonal vector is calculated, that randomly (through numerical errors)
        # points either to the right or left. z is defaultly set to 0.
        vector_inter_waypoints_global = waypoint_global_next_pos - waypoint_global_previous_pos
        # Check if current UAV position is directly on the path between the waypoints
        if all(np.isclose(initial_vector_to_path[:2], 0)):
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
        reverse_time = rospy.Time.from_sec(5)  # time to give the uav to reverse its direction before checking the distance violations again

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
                rospy.loginfo(
                    "Too far away from path. Emergency Landing")  # ToDo: Send trigger to global path for landing
                return False

            elif escape_trigger:
                if np.linalg.norm((escape_trigger_pos - uav_pos)) >= extra_distance:
                    return True

            elif not dir_reversed and (distance_to_path >= self.bug_deviation_from_path_max or is_orthogonal):
                rospy.loginfo("Reversing direction")
                dir_reversed = True
                initial_vector_to_path = -initial_vector_to_path  # reverse the movement direction
                time_reversed = rospy.get_time()

            elif dir_reversed and (distance_to_path >= self.bug_deviation_from_path_max or is_orthogonal):
                # This if-statement gives the uav time to reverse its direction, before saying that target is
                # unreachable
                if rospy.get_time() - time_reversed > reverse_time:
                    # ToDo: Send trigger to global path to change global waypoints
                    rospy.loginfo("No escape found, target unrechable. Trying next global waypoint")
                    return False

            setpoint = PositionTarget()
            setpoint.type_mask = 0b101111111000
            setpoint.coordinate_frame = 1
            setpoint.header = Header()
            setpoint.header.stamp = rospy.Time.now()
            setpoint.position.x = uav_pos_setpoint[0]
            setpoint.position.y = uav_pos_setpoint[1]
            setpoint.position.z = uav_pos_z_start  # Keep at a constant altitude
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
            descent_dir = -self.deseralize_coordinates(gradient_srv_resp.resp.data, 3)[0]
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
        vector_normal_normalized = copy.copy(vector_normal) / np.linalg.norm(vector_normal)
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

    @staticmethod
    def serialize_coordinates(array_2d):
        # type: (np.ndarray) -> np.ndarray
        """The input for the services get_apf_potential and get_apf_gradient has the format
        (x0, y0, z0, x1, y1, z1, ..., xn, yn, zn). This functions transforms an 2d numpy array of the format
        ((x0, y0, z0), (x1,y1,z1), ..., (xn, yn, zn)) into the needed input format."""
        array_2d = np.asarray(array_2d)
        array_1d = np.array(array_2d).ravel()
        return array_1d

    @staticmethod
    def deseralize_coordinates(array_1d, n_pts):
        # type: (np.ndarray, int) -> np.ndarray
        """The output from the service get_apf_gradient has the format (x0, x1, x2, ..., xn, y0, y1, ..., yn, z0, ..., zn).
        This function transforms it into a numpy array of the format ((x0, y0, z0), (x1,y1,z1), ..., (xn, yn, zn))."""
        array_1d = np.asarray(array_1d)
        array_size = len(array_1d)
        if not array_size % n_pts == 0:
            raise Warning(
                "Array size must be a clean multiple of n_pts. Array length: {0}, n_pts: {1}".format(array_size,
                                                                                                     n_pts))
        array_2d = np.zeros((array_size / n_pts, n_pts))
        for i in range(n_pts):
            array_2d[:, i] = array_1d[array_size / n_pts * i:array_size / n_pts * (i + 1)]
        return array_2d