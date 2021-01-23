#!/usr/bin/env python2

from uav_path_planning.msg import obstacleMsg
import warnings


class Obstacle:
    def __init__(self, name):
        warnings.warn("Using the Obstacle class is deprecated, use obstacleMsg instead")
        self.name = name
        self.pose = list()  # x y z alpha beta gamma
        self.geometry = 'box'  # box, cylinder, sphere
        self.dim = list()  # box: Length x Width x Height, cylinder: Radius x Height, sphere: Radius
        self.typeOfObstacle = 'default'  # tree, house, street, powerline, car, bus, bridge, statue, etc.
        return

    def to_rosmsg(self):
        msg = obstacleMsg()
        msg.name = self.name
        msg.typeOfObstacle = self.typeOfObstacle
        msg.geometry = self.geometry
        msg.pose = self.pose
        msg.dim = self.dim
        return msg
