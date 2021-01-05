# Input ObstacleListMsg von der obstacle_map_node
# Output Potentialfeldfunktion

import numpy as np
import typing
from uav_path_planning.msg import obstacleMsg, obstacleListMsg
from helper import attractive_potential_function, obstacle_potential_function


# ToDo: Maybe write functions as service calls

def get_potential_field(uav_coordinate, goal_coordinate, obstacle_map):
    # type: (np.array, np.array, typing.List[obstacleMsg]) -> typing.Union[float, np.array]
    if len(uav_coordinate.shape) == 1:
        potential = 0
    else:
        potential = np.zeros(uav_coordinate.shape[1])

    for obstacle_i in obstacle_map:
        potential += obstacle_potential_function(uav_coordinate, obstacle_i.pose[:3], kr=1, rho0=1)
    potential += attractive_potential_function(uav_coordinate, goal_coordinate, ka=1)
    return potential


def get_vector_field(uav_coordinate, goal_coordinate, obstacle_map):
    # type: (np.array, np.array, typing.List[obstacleMsg]) -> typing.Union[float, np.array]
    vector = np.zeros(uav_coordinate.shape)

    for obstacle_i in obstacle_map:
        vector += obstacle_potential_function(uav_coordinate, obstacle_i.pose[:3], kr=1, rho0=1, der=True)
    vector += attractive_potential_function(uav_coordinate, goal_coordinate, ka=1, der=True)
    return vector

# class PotentialField(object):
#     def __init__(self, goal_coordinate):
#         self.name = 'potential_field'
#         rospy.init_node(self.name)
#         self.loop_rate = rospy.Rate(1)
#
#         self.goalCoordinate = goal_coordinate
#         self.map = None  # type: typing.List[obstacleMsg]
#
#         # Wait until
#         rospy.Subscriber('~/obstacle_map', obstacleListMsg, self._map_callback)
#         while self.map is None:
#             self.loop_rate.sleep()
#             rospy.loginfo(self.name + ": Waiting for obstacle_map to be received")
#
#     def _map_callback(self, data):
#         # type: (obstacleListMsg) -> None
#         """Callback function for the subscription to /mavros/local_position/pose."""
#         self.map = data.obstacleList
#         return
#
#     def get_potential_field(self, uav_coordinate):
#         # type: (np.array) -> typing.Union[float, np.array]
#         if len(uav_coordinate.shape) == 1:
#             potential = 0
#         else:
#             potential = np.zeros(uav_coordinate.shape[1])
#
#         for obstacle_i in self.map:
#             potential += obstacle_potential_function(uav_coordinate, obstacle_i.pose[:3], kr=1, rho0=1)
#         potential += attractive_potential_function(uav_coordinate, self.goalCoordinate, ka=1)
#         return potential
#
#     def get_vector_field(self, uav_coordinate):
#         # type: (np.array) -> typing.Union[np.array]
#         vector = np.zeros(uav_coordinate.shape)
#
#         for obstacle_i in self.map:
#             vector += obstacle_potential_function(uav_coordinate, obstacle_i.pose[:3], kr=1, rho0=1, der=True)
#         vector += attractive_potential_function(uav_coordinate, self.goalCoordinate, ka=1, der=True)
#         return vector
#
#
# if __name__ == '__main__':
#     X0 = np.array(([[4, 4, 4], [2,1,2]]))
#
#     Xdes0 = np.zeros(3)
#     Xobs0 = np.ones(3)
#
#     print(obstacle_potential_function(X0, Xobs0, 1, 5))
#     print(obstacle_potential_function(X0, Xobs0, 1, 5, der=True))
#
#     print(attractive_potential_function(X0, Xobs0, 1))
#     print(attractive_potential_function(X0, Xobs0, 1, der=True))