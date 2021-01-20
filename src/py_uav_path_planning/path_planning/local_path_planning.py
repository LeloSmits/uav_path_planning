#!/usr/bin/env python2
import copy

import numpy as np
import rospy
import typing
from threading import Thread
import threading

from geometry_msgs.msg import PoseStamped, Pose
from uav_path_planning.msg import obstacleMsg, obstacleListMsg
from std_msgs.msg import Header
from mavros_msgs.msg import PositionTarget, Waypoint, WaypointList

from tf.transformations import euler_from_quaternion

from potentialfield import get_potential_field, get_vector_field


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

        self.speed_max_xy = 3.  # type: float  # Maximum speed in xy plane
        self.speed_max_z = 1.  # type: float  # Maximum speed in z plane
        self.speed_multiplier = 3.  # type: float  # ToDo: Make adaptive
        self.step_size = 1.
        self.max_iter_per_wp = 100  # type: int

        self.mode_0_1_angle_threshold = 5.  # degrees
        self.mode_0_1_limit = 1.
        self.mode_1_2_time_limit = 5  # secs

        self.uav_is_stalled = False

        self.tol_wp_global_reached = rospy.get_param('tol_wp_global_reached', .1)  # Tolerance for global waypoint
        self.tol_wp_local_reached = rospy.get_param('tol_wp_reached_local', .1)  # Tolerance for local waypoint when using APF*
        self.tol_wp_local_simulated = rospy.get_param('tol_wp_reached_simulated', .05)  # Tolerance for local waypoint when creating new nodes for APF* with Gradient Descent
        self.tol_dev_global_path = rospy.get_param('tol_dev_global_path', .5)
        self.threshold_wp_position_control = rospy.get_param('threshold_wp_position_control', 1)

        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._callback_pose)
        rospy.Subscriber('obstacle_map', obstacleListMsg, self._callback_map)
        rospy.Subscriber('waypoint_global_next', Waypoint, self._callback_waypoint_global_current)
        rospy.Subscriber('waypoint_global_previous', Waypoint, self._callback_waypoint_global_previous)

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

    def __get_uav_position(self, pose=None):
        # type: (Pose) -> np.ndarray
        if pose is None:
            uav_pose = copy.copy(self.uav_pose)
            return np.array([uav_pose.position.x, uav_pose.position.y, uav_pose.position.z])
        else:
            if not isinstance(pose, Pose):
                raise Warning("Argument pose must be of type Pose. Was of type: " + type(pose))
            return np.array([pose.position.x, pose.position.y, pose.position.z])

    def __get_uav_yaw(self, pose=None):
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

    def __get_waypoint_global_position(self):
        # type: (LocalPathPlanner) -> (np.ndarray, np.ndarray)
        """Returns the x,y,z coordinates of the current and previous global waypoint in a tuple
        (np.array(x_cur,y_cur,z_cur), np.array(x_cur,y_cur,z_cur))"""

        waypoint_global_current_position = np.array([self.waypoint_global_next.x_lat,
                                                     self.waypoint_global_next.y_long,
                                                     self.waypoint_global_next.z_alt])

        waypoint_global_previous_position = np.array([self.waypoint_global_previous.x_lat,
                                                      self.waypoint_global_previous.y_long,
                                                      self.waypoint_global_previous.z_alt])

        return waypoint_global_current_position, waypoint_global_previous_position

    def direct_new_setpoint(self):
        goal_coordinate = self.__get_waypoint_global_position()[0]
        uav_coordinate = self.__get_uav_position()

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

    def apf_new_setpoint(self, goal_waypoint=None, ctrl='velocity'):
        # type: (Waypoint, str) -> PositionTarget
        if goal_waypoint is None:
            goal_pos = self.__get_waypoint_global_position()[0]
        else:
            goal_pos = np.array([goal_waypoint.x_lat,
                                 goal_waypoint.y_long,
                                 goal_waypoint.z_alt])

        uav_pos = self.__get_uav_position()

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
            gradient = get_vector_field(uav_coordinate=self.__get_uav_position(), goal_coordinate=goal_pos,
                                        obstacle_map=self._map)
            direction = -gradient  # Subtraction because we want to descent / move to smaller potential
            direction[2] = 0

            # ToDo: handle direction_length close to 0
            direction_length = np.linalg.norm(direction)  # ToDo: adapt for 3D

            if ctrl == 'velocity':
                setpoint = PositionTarget()

                # Deal with small step size
                if np.isclose(direction_length, 0):
                    setpoint.type_mask = 0b111111111000
                else:
                    setpoint.type_mask = 0b101111111000
                    setpoint.yaw = np.arccos((direction[:2].dot(np.array([1, 0])) / direction_length))

                setpoint.coordinate_frame = 1
                setpoint.header = Header()
                setpoint.header.stamp = rospy.Time.now()

                velocity = direction ** 2 * self.speed_multiplier  # / direction_length  # ToDo: direction_length, see above
                setpoint.velocity.x = velocity[0]
                setpoint.velocity.y = velocity[1]
                setpoint.position.z = self.waypoint_global_next.z_alt  # ToDo: make 3D
                break

            elif ctrl == 'position':
                potential_here = get_potential_field(uav_coordinate=uav_pos, goal_coordinate=goal_pos,
                                                     obstacle_map=self._map)

                uav_pos_new = uav_pos + step_size * direction  # / direction_length  # ToDo: direction_length, see above
                potential_new = get_potential_field(uav_coordinate=uav_pos_new, goal_coordinate=goal_pos,
                                                    obstacle_map=self._map)
                if potential_new > potential_here:
                    step_size /= 2
                else:
                    setpoint = PositionTarget()

                    # Deal with small step size
                    if np.isclose(direction_length, 0):
                        setpoint.type_mask = 0b111111111000
                    else:
                        setpoint.type_mask = 0b101111111000
                        setpoint.yaw = np.arccos((direction[:2].dot(np.array([1, 0])) / direction_length))

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

    def apf_star_new_setpoint(self):
        class APFPosition:
            def __init__(self, parent, position, depth):
                self.parent = parent  # type: APFPosition
                self.children = list()  # type: typing.List[APFPosition]
                self.position = position
                self.depth = depth
                self.potential = get_potential_field(position, goal_coordinate=waypoint_global_current_position,
                                                     obstacle_map=obs_map)
                self.gradient = get_vector_field(position, goal_coordinate=waypoint_global_current_position,
                                                 obstacle_map=obs_map)
                self.reached_potential = np.inf

            def gradient_descent(self, save_reached_potential=True, goal_coordinate=None, obstacle_map=None):
                if goal_coordinate is None:
                    goal_coordinate = waypoint_global_current_position

                if obstacle_map is None:
                    obstacle_map = obs_map

                pts_hist = np.zeros((max_iter, 3))
                pot_hist = np.zeros(max_iter)

                pts_hist[0] = self.position
                pot_hist[0] = get_potential_field(pts_hist[0], goal_coordinate=goal_coordinate,
                                                  obstacle_map=obstacle_map)

                k = 0
                for ii in range(max_iter - 1):
                    step_size = descent_step

                    neg_grad = -get_vector_field(pts_hist[k], goal_coordinate=goal_coordinate,
                                                 obstacle_map=obstacle_map)

                    # ToDo: New
                    for j in range(max_iter - 1):
                        new_pt = pts_hist[k] + step_size * neg_grad
                        new_potential = get_potential_field(new_pt, goal_coordinate=goal_coordinate,
                                                            obstacle_map=obstacle_map)
                        if new_potential < pot_hist[k]:
                            pts_hist[k + 1] = new_pt
                            pot_hist[k + 1] = new_potential
                            k += 1

                            if all(np.isclose(pts_hist[k], goal_coordinate, atol=tol_wp_global_reached)):
                                break

                            break

                        else:
                            step_size /= 2

                    if step_size * 2 ** 20 < descent_step:  # step size too small
                        break

                if save_reached_potential:
                    self.reached_potential = pot_hist[k]  # save reached potential

                pts_hist = pts_hist[:k + 1]
                pot_hist = pot_hist[:k + 1]

                return pts_hist, pot_hist

            def expand(self):
                if self.parent is None:
                    vector_root_node = self.position - uav_pos
                else:
                    vector_root_node = self.position - self.parent.position
                vector_root_node = vector_root_node / np.linalg.norm(vector_root_node) * radius_steps
                position1 = self.position + vector_root_node
                position2 = self.position + R(alpha).dot(vector_root_node)
                position3 = self.position + R(-alpha).dot(vector_root_node)

                newNode1 = APFPosition(self, position1, self.depth + 1)
                newNode2 = APFPosition(self, position2, self.depth + 1)
                newNode3 = APFPosition(self, position3, self.depth + 1)

                # Check if new nodes reachable with apf
                for node_i in (newNode1, newNode2, newNode3):
                    pts, pot = self.gradient_descent(save_reached_potential=False, goal_coordinate=node_i.position)
                    if all(np.isclose(pts[-1], node_i.position, atol=tol_wp_local_simulated)):
                        self.children.append(node_i)
                return

        def bfs():
            import Queue
            # initilalize
            active_nodes = Queue.Queue()
            for angle in np.linspace(uav_yaw, uav_yaw + (2 - 2 / n_pts) * np.pi, n_pts):
                position = uav_pos + R(angle).dot(np.array([radius_steps, 0, 0]))
                node = APFPosition(None, position, 0)
                active_nodes.put(node)

            while not active_nodes.empty():
                node_i = active_nodes.get()
                # Only visit a node if its potential and depth is below the upper bound
                if node_i.potential < potential_ub \
                        and node_i.depth <= depth_ub \
                        and np.linalg.norm(node_i.position - uav_pos) <= max_radius:
                    node_i.gradient_descent()
                    # If the node reaches a potential below the threshold return that node
                    if node_i.reached_potential < ((1 - potential_percentage_increase) * uav_potential
                                                   + potential_percentage_increase * goal_potential):
                        return node_i
                    # If it is above the threshold, expand
                    else:
                        node_i.expand()
                        for child in node_i.children:
                            active_nodes.put(child)

            return None

        function_name = "Local Minima Evasion"

        uav_pos = self.__get_uav_position()
        uav_yaw = self.__get_uav_yaw()

        obs_map = self._map
        waypoint_global_current_position, waypoint_global_previous_position = self.__get_waypoint_global_position()
        tol_wp_global_reached = self.tol_wp_global_reached
        tol_wp_local_simulated = self.tol_wp_local_simulated

        max_radius = 2 * np.linalg.norm(waypoint_global_current_position - waypoint_global_previous_position)

        max_iter = 100
        depth_ub = 20

        # Set the maximum allowed potential for a node to expand equal to twice the potential of the previous global
        # waypoint. This should theoretically allow the graph to expand to a radius around the current global waypoint
        # equal to twice the length between the previous and current global waypoint.
        potential_ub = 2 * get_potential_field(uav_pos, goal_coordinate=self.__get_waypoint_global_position()[1],
                                               obstacle_map=self._map)

        descent_step = self.step_size  # step size for gradient descent of nodes
        # Percentage by which the reached potential of the new node has to be better
        potential_percentage_increase = 0.2
        radius_steps = 1
        n_pts = 9  # number of points for the first layer
        n_exp = 3  # Not used. Nodes to be expanded from each node. So far only hardcoded
        alpha = 45. / 360. * 2 * np.pi  # angle between the vector between child node to parent node
        # and parent node - parent's parent node
        R = lambda x: np.array([[np.cos(x), np.sin(x), 0], [-np.sin(x), np.cos(x), 0], [0, 0, 1]])  # Rotation matrix

        uav_potential = get_potential_field(uav_pos, goal_coordinate=waypoint_global_current_position,
                                            obstacle_map=self._map)
        goal_potential = get_potential_field(waypoint_global_current_position,
                                             goal_coordinate=waypoint_global_current_position,
                                             obstacle_map=self._map)

        result = None
        escape_node = bfs()
        if escape_node is None:
            rospy.loginfo(function_name + ": UAV potential {0}".format(uav_potential))
            rospy.loginfo(function_name + ": Goal potential {0}".format(goal_potential))
            rospy.loginfo(function_name + ": Target unreachable")
            # ToDo: call "not reachable"
            pass
        else:
            result = WaypointList()
            # initialize list where nodes are put
            nodes = [None] * (escape_node.depth + 1)  # type: typing.List[APFPosition]
            current_node = escape_node
            # find the root node
            i = escape_node.depth
            while current_node is not None:
                nodes[i] = current_node
                current_node = current_node.parent
                i -= 1
            if i != -1: rospy.loginfo(function_name + ": Something went wrong")

            # append all nodes from the root node to the node returned by bfs
            for node_j in nodes:
                wpt = Waypoint()
                wpt.x_lat = node_j.position[0]
                wpt.y_long = node_j.position[1]
                wpt.z_alt = node_j.position[2]
                result.waypoints.append(wpt)

            rospy.loginfo(result)
            rospy.loginfo(function_name + ": UAV potential {0}".format(uav_potential))
            rospy.loginfo(function_name + ": Goal potential {0}".format(goal_potential))
            rospy.loginfo(function_name + ": Last setpoint potential {0}".format(nodes[-1].reached_potential))

        # ToDo: APF mit neuer wpts-liste starten
        return result

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

    def determine_progress(self):
        """Sets self.uav_is_stalled to True, if there was in no progress in direction of the goal point in the last
        self.mode_1_2_time_limit seconds"""
        t = threading.current_thread()

        function_name = "UAV Progress Determination"
        # ToDo: if goal_point changes, reset
        rospy.loginfo(function_name + ": Thread started")
        time_last_progress = rospy.get_rostime().secs
        goal_pos = self.__get_waypoint_global_position()[0]
        best_uav_pos = self.__get_uav_position()

        while not rospy.is_shutdown() and getattr(t, "do_run", True):
            uav_pos = self.__get_uav_position()
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

            uav_pos = self.__get_uav_position()
            uav_yaw = self.__get_uav_yaw()
            goal_coordinate = self.__get_waypoint_global_position()[0]

            # ToDo: Find heuristic to determine if to use position or velocity control
            setpoint_apf = self.apf_new_setpoint(ctrl='position')
            setpoint_direct = self.direct_new_setpoint()

            if self.uav_is_stalled:
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
                    self.setpoint_local = copy.copy(setpoint)
                    rospy.sleep(.1)

                rospy.sleep(.5)

                rospy.loginfo(self.name + ": Searching local waypoints")
                wpts_new = self.apf_star_new_setpoint()

                if wpts_new is None:
                    rospy.loginfo(self.name + ": Target unrechable")
                else:
                    rospy.loginfo(self.name + ": Started publishing local waypoints")
                    for waypoint_local in wpts_new.waypoints:
                        rospy.loginfo(self.name + ": New local waypoint")
                        uav_pos = self.__get_uav_position()

                        # direction = np.array([waypoint_local.x_lat, waypoint_local.y_long]) - uav_pos[:2]
                        # direction_length = np.linalg.norm(direction)

                        # setpoint = PositionTarget()
                        # setpoint.type_mask = 0b101111111000
                        # setpoint.coordinate_frame = 1
                        # setpoint.header = Header()
                        # setpoint.header.stamp = rospy.Time.now()
                        # setpoint.position.x = waypoint_local.x_lat
                        # setpoint.position.y = waypoint_local.y_long
                        # setpoint.position.z = self.waypoint_global_next.z_alt  # ToDo: make 3D
                        # setpoint.yaw = np.arccos((direction[:2].dot(np.array([1, 0])) / direction_length))

                        # ToDo: Implement Timeout
                        while not all(np.isclose(uav_pos[:2], (waypoint_local.x_lat, waypoint_local.y_long),
                                                 atol=self.tol_wp_local_reached)) \
                                and not rospy.is_shutdown():
                            uav_pos = self.__get_uav_position()
                            setpoint = self.apf_new_setpoint(goal_waypoint=waypoint_local, ctrl='position')
                            self.setpoint_local = setpoint  # self.apf_new_setpoint(goal_waypoint=waypoint_local, ctrl='position')
                            self._rate_publish.sleep()

                    rospy.loginfo(self.name + ": Reached last local waypoint")

                rospy.loginfo(self.name + ": Returning to normal control")

            # If UAV is close to global waypooint, do position setpoint control
            if all(np.isclose(uav_pos[:2], goal_coordinate[:2],
                              atol=self.threshold_wp_position_control)):  # return after global WP is reached
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

        while (self.waypoint_global_next is None or self.waypoint_global_previous is None) \
                and not rospy.is_shutdown():
            self._rate_publish.sleep()

        rospy.loginfo(self.name + ': Starting local path planning')
        self.master()

        return
