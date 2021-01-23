# Input ObstacleListMsg von der obstacle_map_node
# Output Potentialfeldfunktion

import numpy as np
import typing
from uav_path_planning.msg import obstacleMsg, obstacleListMsg
from helper import attractive_potential_function, obstacle_potential_function


# ToDo: Maybe write functions as service calls
kr = 10
rho0 = 10
ka = 2


def get_potential_field(uav_coordinate, goal_coordinate, obstacle_map):
    # type: (np.array, np.array, typing.List[obstacleMsg]) -> typing.Union[float, np.array]

    if len(uav_coordinate.shape) == 1:
        potential = 0
    else:
        potential = np.zeros(uav_coordinate.shape[1])

    for obstacle_i in obstacle_map:
        potential += obstacle_potential_function(uav_coordinate, obstacle_i.pose[:3], kr=kr, rho0=rho0)
    potential += attractive_potential_function(uav_coordinate, goal_coordinate, ka=ka)
    return potential


def get_vector_field(uav_coordinate, goal_coordinate, obstacle_map):
    # type: (np.array, np.array, typing.List[obstacleMsg]) -> typing.Union[float, np.array]
    vector = np.zeros(uav_coordinate.shape)

    for obstacle_i in obstacle_map:
        vector += obstacle_potential_function(uav_coordinate, obstacle_i.pose[:3], kr=kr, rho0=rho0, der=True)
    vector += attractive_potential_function(uav_coordinate, goal_coordinate, ka=ka, der=True)
    return vector
