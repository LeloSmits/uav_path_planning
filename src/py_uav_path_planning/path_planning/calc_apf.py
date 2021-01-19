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


class ArtificialPotentialField:

    def __init__(self):
        self.name = 'APF'
        self.potential = list()
        self.gradient = list()
        self.previous_waypoint = np.zeros(3)
        self.current_waypoint = np.zeros(3)
        self.df = pd.read_csv("/home/leonard/catkin_ws/src/path_planning_private/src/py_uav_path_planning/path_planning/Classification.csv")
        rospy.Subscriber('path_planning/obstacle_map', obstacleListMsg, self._obstacle_list_callback)
        rospy.Subscriber('path_planning/waypoint_global_previous', Waypoint, self._previous_waypoint_callback)
        rospy.Subscriber('path_planning/waypoint_global_current', Waypoint, self._current_waypoint_callback)
        # Service Call to get the potential of different points
        rospy.Service('get_apf_potential', potential_field_msg, self._potential_callback)
        rospy.Service('get_apf_gradient', potential_field_msg, self._gradient_callback)

    def _obstacle_list_callback(self, data):
        # type: (obstacleListMsg) -> None
        self.current_obstacles = data.obstacleList
        return

    def _previous_waypoint_callback(self, data):
        # type: (Waypoint) -> None
        self.previous_waypoint[0] = data.x_lat
        self.previous_waypoint[1] = data.y_long
        self.previous_waypoint[2] = data.z_alt
        return

    def _current_waypoint_callback(self, data):
        # type: (Waypoint) -> None
        self.current_waypoint[0] = data.x_lat
        self.current_waypoint[1] = data.y_long
        self.current_waypoint[2] = data.z_alt

    # def potential_obstacles(self):
    #     for i in range(len(self.current_obstacles)):
    #         self.current_obstacles[i].name =

    # Waypoint is a Float32MultiArray
    def _potential_callback(self, message):
        # print(message)
        waypoint = message.req.data
        # print(waypoint)
        # Defining key variables and parameters
        x_coordinate = []
        y_coordinate = []
        z_coordinate = []
        pot_coordinates = []
        x_factor = 10
        y_factor = 10
        z_factor = 2*y_factor

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
            obs123.classification = obs123.get_classification(df=self.df)

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
                obs123.c = obs123.a * 2

            obstacle_list.append(obs123)
        my_length = len(waypoint) / 3
        # Calculating the potential for each obstacle at each coordinate
        # potential = np.zeros()
        potential = np.zeros(shape=(1, my_length))
        for i in range(len(obstacle_list)):
            potential = potential + obstacle_list[i].obstacle_potential_function(uav_pose=self.current_waypoint,
                                                                                 waypoint=pot_coordinates, der=False)
        # Define the message
        pot_list = potential[0].astype(np.float32).tolist()
        print(pot_list)
        potential_field = potential_field_msgResponse()
        potential_field.resp.data = pot_list
        return potential_field

    # return the gradient for a series of coordinates
    def _gradient_callback(self, message):
        # print(message)
        waypoint = message.req.data
        # print(waypoint)
        # Defining key variables and parameters
        x_coordinate = []
        y_coordinate = []
        z_coordinate = []
        pot_coordinates = []
        x_factor = 10
        y_factor = 10
        z_factor = 2*y_factor

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
            obs123.classification = obs123.get_classification(df=self.df)

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
                obs123.c = obs123.a * 2

            obstacle_list.append(obs123)
        my_length = len(waypoint) / 3
        # Calculating the potential for each obstacle at each coordinate
        # potential = np.zeros()
        gradient = np.zeros(shape=(1, my_length))
        for i in range(len(obstacle_list)):
            gradient = gradient + obstacle_list[i].obstacle_potential_function(uav_pose=self.current_waypoint,
                                                                               waypoint=pot_coordinates, der=True)
        # Define the message
        grad_list = gradient[0].astype(np.float32).tolist()
        grad_list.extend(gradient[1].astype(np.float32).tolist())
        grad_list.extend(gradient[2].astype(np.float32).tolist())
        print(grad_list)
        gradient_field = potential_field_msgResponse()
        gradient_field.resp.data = grad_list
        return gradient_field

class Obstacle:
    def __init__(self):
        self.name = str
        self.pose = list()
        self.geometry = 'box'
        self.dim = list()
        self.typeOfObstacle = 'default'             # Tree, house, street, powerline, car, bus
        self.classification = int
        self.a = float
        self.b = float
        self.c = float
        self.potential = float

    def get_classification(self, df):
        found = bool
        found = self.typeOfObstacle in df.values
        if found:
            # test = myDF.loc[myDF['A']==11, 'B'].iloc[0]
            self.classification = df.loc[df['Hindernisart']==self.typeOfObstacle, 'Gefahrenindex'].iloc[0]
        if not found:
            # set default value
            self.classification = 1
        return self.classification

    # Calculating the potential of a certain point
    def obstacle_potential_function(self, uav_pose, waypoint, der):
        x_dist = self.pose[0] - waypoint[0]
        y_dist = self.pose[1] - waypoint[1]
        z_dist = self.pose[2] - waypoint[2]

        if not der:
            potential_test = self.classification/((x_dist**2)/self.a**2 + (y_dist**2)/self.b**2 + (z_dist**2)/self.c**2)
            return potential_test
        else:
            x_grad = ((-1*self.classification*2*x_dist**2)/(self.a**2))/(x_dist**2/self.a**2 + y_dist**2/self.b**2 + z_dist**2/self.c**2)
            y_grad = ((-1*self.classification*2*y_dist**2)/(self.b**2))/(x_dist**2/self.a**2 + y_dist**2/self.b**2 + z_dist**2/self.c**2)
            z_grad = ((-1*self.classification*2*z_dist**2)/(self.c**2))/(x_dist**2/self.a**2 + y_dist**2/self.b**2 + z_dist**2/self.c**2)
            return [x_grad, y_grad, z_grad]


if __name__ == "__main__":
    rospy.init_node('calc_apf')
    ArtificialPotentialField()
    rospy.spin()






