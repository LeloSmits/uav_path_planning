#!/usr/bin/env python2

# This function creates a potential field as well as the gradient field as the basis for the APF path planning
# algorithm
# Input: Obstacles, position of the UAV, excel file for the danger classification, global path
# Output: Array of potential field for certain positions, Array of gradient field for certain positions


import math
import numpy as np
import rospy
import pandas as pd

from uav_path_planning.msg import obstacleListMsg
from mavros_msgs.msg import Waypoint
from uav_path_planning.srv import potential_field_msg, potential_field_msgResponse

ka = rospy.get_param("apf_ka")  # parameter for the attractiveness of the goal
kb = rospy.get_param("apf_kb")  # parameter for the attractiveness of the trench
kc = rospy.get_param("apf_kc")
kd = rospy.get_param("apf_kd")
rho0 = 10000.
x_factor = 1.
y_factor = 1.
z_factor = .3

limit = rospy.get_param("z_min")


class ArtificialPotentialField:

    def __init__(self, filepath):
        self.name = 'APF'
        self.potential = list()
        self.gradient = list()
        self.previous_waypoint = None  # ToDo adjust these or set these to None
        self.next_waypoint = None  # ToDo adjust these or set these to None
        self.df = pd.read_csv(filepath)
        rospy.Subscriber('obstacle_map', obstacleListMsg, self._obstacle_list_callback)
        rospy.Subscriber('waypoint_global_previous', Waypoint, self._previous_waypoint_callback)
        rospy.Subscriber('waypoint_global_next', Waypoint, self._next_waypoint_callback)
        while self.next_waypoint is None:
            rospy.sleep(.5)
        # Service Call to get the potential of different points
        rospy.Service('get_apf_potential', potential_field_msg, self._potential_callback)
        # Service Call to get the gradient of different points
        rospy.Service('get_apf_gradient', potential_field_msg, self._gradient_callback)

    def _obstacle_list_callback(self, data):
        # type: (obstacleListMsg) -> None
        self.current_obstacles = data.obstacleList
        return

    def _previous_waypoint_callback(self, data):
        # type: (Waypoint) -> None
        self.previous_waypoint = np.array([data.x_lat, data.y_long, data.z_alt])
        return

    def _next_waypoint_callback(self, data):
        # type: (Waypoint) -> None
        self.next_waypoint = np.array([data.x_lat, data.y_long, data.z_alt])

    # Waypoint is a Float32MultiArray
    def _potential_callback(self, message):
        # print(message)
        waypoint = message.req.data
        # Two modes. Mode 0: All potentials. Mode 1: Only obstacle and ground
        mode = message.mode

        # Defining key variables and parameters
        x_coordinate = []
        y_coordinate = []
        z_coordinate = []
        pot_coordinates = []

        for i in (range(len(waypoint) / 3)):
            x_coordinate.append(waypoint[0 + i * 3])
            y_coordinate.append(waypoint[1 + i * 3])
            z_coordinate.append(waypoint[2 + i * 3])

        pot_coordinates.append(x_coordinate)
        pot_coordinates.append(y_coordinate)
        pot_coordinates.append(z_coordinate)
        # Conversion of the list of lists to a numpy array
        pot_coordinates = np.array(pot_coordinates, dtype=float)


        obstacle_list = list()
        # print(self.current_obstacles)
        for i in range(len(self.current_obstacles)):
            obs_from_obstacle_map = self.current_obstacles[i]

            # Initiating a class instance for each current obstacle
            obs123 = Obstacle()
            obs123.name = obs_from_obstacle_map.name
            obs123.pose = obs_from_obstacle_map.pose
            obs123.geometry = obs_from_obstacle_map.geometry
            obs123.dim = obs_from_obstacle_map.dim
            obs123.typeOfObstacle = obs_from_obstacle_map.typeOfObstacle
            obs123.get_classification(df=self.df)

            if obs123.geometry == 'box':
                obs123.a = obs_from_obstacle_map.dim[0] * x_factor
                obs123.b = obs_from_obstacle_map.dim[1] * y_factor
                obs123.c = obs_from_obstacle_map.dim[2] * z_factor
            elif obs123.geometry == 'cylinder':
                obs123.a = obs_from_obstacle_map.dim[0] * x_factor
                obs123.b = obs123.a
                obs123.c = obs_from_obstacle_map.dim[1] * z_factor
            elif obs123.geometry == 'sphere':
                obs123.a = obs_from_obstacle_map.dim[0] * x_factor
                obs123.b = obs123.a
                obs123.c = obs123.a * z_factor  # * 2

            obstacle_list.append(obs123)
        my_length = len(waypoint) / 3
        # Calculating the potential for each obstacle at each coordinate

        potential = np.zeros(shape=(1, my_length))
        # Calculate the potential of all obstacles for all coordinates
        for i in range(len(obstacle_list)):
            potential[:] = potential + obstacle_list[i].obstacle_potential_function(uav_pose=self.next_waypoint,
                                                                                    waypoint=pot_coordinates, der=False)

        if mode == 0:
            # Calculate the potential of the goal point
            goal_point = self.next_waypoint
            transp_pot_coord = pot_coordinates.transpose()
            difference = transp_pot_coord - goal_point
            dist = np.linalg.norm(difference, axis=1)
            potential[:] = potential + ka * dist

            # Calculate distance in order to build trench potential

            last_point = self.previous_waypoint
            help_vector = transp_pot_coord - last_point
            line_vector = goal_point - last_point
            dist2 = np.linalg.norm(np.cross(help_vector, line_vector), axis=1) / np.linalg.norm(line_vector)
            potential[:] = potential + kb * dist2

            # Build a z-direction potential to enforce the minimum flight height
            # Minimum flight height is 0.5m
            # print(pot_coordinates)
            z_distance = np.array(pot_coordinates[2] - limit)
            # print(z_vector)
            # z_vector = np.ma.array(z_vector, mask=(z_vector == 0.5))        # Mask array to avoid runtime warning
            mask = z_distance < limit
            temp_potential = kd * abs(1 / z_distance)
            temp_potential[mask] = 10 ** 12
            potential[:] = temp_potential + potential
            # print(potential)

        # Define the message
        pot_list = potential[0].astype(np.float32).tolist()
        potential_field = potential_field_msgResponse()
        potential_field.resp.data = pot_list
        return potential_field

    # return the gradient for a series of coordinates
    def _gradient_callback(self, message):
        print(message)
        waypoint = message.req.data
        mode = message.mode
        # print(waypoint)
        # Defining key variables and parameters
        x_coordinate = []
        y_coordinate = []
        z_coordinate = []
        pot_coordinates = []

        for i in (range(len(waypoint) / 3)):
            x_coordinate.append(waypoint[0 + i * 3])
            y_coordinate.append(waypoint[1 + i * 3])
            z_coordinate.append(waypoint[2 + i * 3])

        pot_coordinates.append(x_coordinate)
        pot_coordinates.append(y_coordinate)
        pot_coordinates.append(z_coordinate)
        # Conversion of the list of lists to a numpy array
        pot_coordinates = np.array(pot_coordinates, dtype=float)
        # print(pot_coordinates)

        obstacle_list = list()
        # print(self.current_obstacles)
        for i in range(len(self.current_obstacles)):
            obs_from_obstacle_map = self.current_obstacles[i]

            # Initiating a class instance for each current obstacle
            obs123 = Obstacle()
            obs123.name = obs_from_obstacle_map.name
            obs123.pose = obs_from_obstacle_map.pose
            obs123.geometry = obs_from_obstacle_map.geometry
            obs123.dim = obs_from_obstacle_map.dim
            obs123.typeOfObstacle = obs_from_obstacle_map.typeOfObstacle
            obs123.get_classification(df=self.df)

            if obs123.geometry == 'box':
                obs123.a = obs_from_obstacle_map.dim[0] * x_factor
                obs123.b = obs_from_obstacle_map.dim[1] * y_factor
                obs123.c = obs_from_obstacle_map.dim[2] * z_factor
            elif obs123.geometry == 'cylinder':
                obs123.a = obs_from_obstacle_map.dim[0] * x_factor
                obs123.b = obs123.a
                obs123.c = obs_from_obstacle_map.dim[1] * z_factor
            elif obs123.geometry == 'sphere':
                obs123.a = obs_from_obstacle_map.dim[0] * x_factor
                obs123.b = obs123.a
                obs123.c = obs123.a * z_factor  # * 2

            obstacle_list.append(obs123)

        my_length = len(waypoint)
        if my_length != 3:
            rospy.logwarn("Gradient callback probably only works for one coordinate. "
                          "{0} coordinates have been entered.".format(my_length / 3))

        # Calculating the potential for each obstacle at each coordinate
        # potential = np.zeros()
        gradient = np.zeros(shape=(my_length / 3, my_length))
        for i in range(len(obstacle_list)):
            gradient[:] = gradient + obstacle_list[i].obstacle_potential_function(uav_pose=self.next_waypoint,
                                                                                  waypoint=pot_coordinates, der=True)
        if mode == 0:
            # Calculate the gradient due to the influence of the goal

            goal_point = self.next_waypoint
            transp_pot_coord = pot_coordinates.transpose()

            difference = transp_pot_coord - goal_point
            transp_difference = difference.transpose()
            distance = np.linalg.norm(difference, axis=1)

            gradient[:, 0] = gradient[:, 0] + ka * (transp_difference[0] / distance)
            gradient[:, 1] = gradient[:, 1] + ka * (transp_difference[1] / distance)
            gradient[:, 2] = gradient[:, 2] + ka * (transp_difference[2] / distance)

            # Calculate the gradient due to the influence of the trench
            last_point = self.previous_waypoint
            help_vector = transp_pot_coord - last_point
            transp_help_vector = help_vector.transpose()
            line_vector = goal_point - last_point

            term_1 = line_vector[2] * transp_help_vector[1] - line_vector[1] * transp_help_vector[2]
            term_2 = line_vector[0] * transp_help_vector[2] - line_vector[2] * transp_help_vector[0]
            term_3 = line_vector[1] * transp_help_vector[0] - line_vector[0] * transp_help_vector[1]

            gradient[:, 0] = gradient[:, 0] + (
                    -1 * line_vector[2] * term_2 * kb + line_vector[1] * term_3 * kb) / (np.linalg.norm(
                line_vector) * ((term_1 ** 2 + term_2 ** 2 + term_3 ** 2) ** 0.5))
            gradient[:, 1] = gradient[:, 1] + (
                    line_vector[2] * term_1 * kb - line_vector[0] * term_3 * kb) / (np.linalg.norm(
                line_vector) * ((term_1 ** 2 + term_2 ** 2 + term_3 ** 2) ** 0.5))
            gradient[:, 2] = gradient[:, 2] + (
                    -1 * line_vector[1] * term_1 * kb + line_vector[0] * term_2 * kb) / (np.linalg.norm(
                line_vector) * ((term_1 ** 2 + term_2 ** 2 + term_3 ** 2) ** 0.5))

            # Calculate the gradient to ensure the minimum flight height
            z_distance = pot_coordinates[2] - limit
            mask = z_distance < limit
            temp_gradient = kd * - z_distance / np.abs(z_distance ** 3)
            temp_gradient[mask] = kd * -10 ** 12
            gradient[:, 2] = gradient[:, 2] + temp_gradient

        # Define the message
        grad_list = gradient[:, 0].astype(np.float32).tolist()
        grad_list.extend(gradient[:, 1].astype(np.float32).tolist())
        grad_list.extend(gradient[:, 2].astype(np.float32).tolist())
        gradient_field = potential_field_msgResponse()
        gradient_field.resp.data = grad_list
        return gradient_field


class Obstacle:
    def __init__(self):
        self.name = str
        self.pose = list()
        self.geometry = 'box'
        self.dim = list()
        self.typeOfObstacle = 'default'  # Tree, house, street, powerline, car, bus
        self.classification = int()
        self.a = float()
        self.b = float()
        self.c = float()
        self.potential = float()

    def get_classification(self, df):
        found = self.typeOfObstacle in df.values
        if found:
            self.classification = df.loc[df['Hindernisart'] == self.typeOfObstacle, 'Gefahrenindex'].iloc[0]
        if not found:
            # set default value
            self.classification = kc
        return self.classification

    # Calculating the potential of a certain point
    def obstacle_potential_function(self, uav_pose, waypoint, der):
        close = 10 ** -8
        x_dist = waypoint[0] - self.pose[0]
        y_dist = waypoint[1] - self.pose[1]
        z_dist = waypoint[2] - self.pose[2]
        mask_1 = abs(x_dist) < close
        mask_2 = abs(y_dist) < close
        mask_3 = abs(z_dist) < close
        x_dist[mask_1] = 10 ** -8
        y_dist[mask_2] = 10 ** -8
        z_dist[mask_3] = 10 ** -8

        if not der:
            potential_test = .5 * self.classification * (1 /
                                                         ((x_dist ** 2) / self.a ** 2
                                                          + (y_dist ** 2) / self.b ** 2
                                                          + (z_dist ** 2) / self.c ** 2) - 1 / rho0 ** 2)
            return potential_test
        else:
            # rospy.loginfo("Class.: " + str(self.classification))
            if not x_dist ** 2 / self.a ** 2 + y_dist ** 2 / self.b ** 2 + z_dist ** 2 / self.c ** 2 > rho0 ** 2:
                x_grad = ((-1 * self.classification * x_dist) / (self.a ** 2)) / \
                         ((x_dist ** 2 / self.a ** 2 + y_dist ** 2 / self.b ** 2 + z_dist ** 2 / self.c ** 2) ** 2)
                y_grad = ((-1 * self.classification * y_dist) / (self.b ** 2)) / \
                         ((x_dist ** 2 / self.a ** 2 + y_dist ** 2 / self.b ** 2 + z_dist ** 2 / self.c ** 2) ** 2)
                z_grad = ((-1 * self.classification * z_dist) / (self.c ** 2)) / \
                         ((x_dist ** 2 / self.a ** 2 + y_dist ** 2 / self.b ** 2 + z_dist ** 2 / self.c ** 2) ** 2)
            else:
                return np.asarray([0, 0, 0]).reshape((1, 3))

            return np.asarray([x_grad, y_grad, z_grad]).reshape((1, 3))


if __name__ == "__main__":
    rospy.loginfo("Node started")
    while not rospy.has_param("path_to_classifier_csv"):
        rospy.loginfo("Param path_to_classifier_csv is not set, waiting.")
        rospy.sleep(.1)
    filepath = rospy.get_param("path_to_classifier_csv")
    rospy.init_node('calc_apf')
    ArtificialPotentialField(filepath=filepath)
    rospy.spin()
