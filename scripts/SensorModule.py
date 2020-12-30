#!/usr/bin/env python2

import rospy
import xml.etree.ElementTree as ET
import numpy as np

from geometry_msgs.msg import PoseStamped
from piendltest.msg import obstacleMsg, obstacleListMsg


class Obstacle:
    def __init__(self, name):
        self.name = name
        self.pose = list()  # x y z alpha beta gamma
        self.geometry = 'box'  # box, cylinder, sphere
        self.dim = list()  # box: Length x Width x Height, cylinder: Radius x Height, sphere: Radius
        self.typeOfObstacle = 'default'  # tree, house, street, powerline, car, bus, bridge, statue, etc.
        return


def readObstacleXML(filename):
    obstacle_prefix = 'obs_'

    root_node = ET.parse(filename).getroot()
    models = root_node.findall('world/model')

    allObstacles = list()

    for model in models:
        if model.attrib['name'].startswith(obstacle_prefix):
            newObstacle = Obstacle(model.attrib['name'])
            newObstacle.pose = np.fromstring(model.find('pose').text, dtype=float, sep=' ')
            geometryFirstChild = model.find('link/collision/geometry')[0]
            newObstacle.geometry = geometryFirstChild.tag
            if newObstacle.geometry == 'cylinder':
                newObstacle.dim = np.fromstring(geometryFirstChild[0].text + ' ' + geometryFirstChild[1].text,
                                                dtype=float,
                                                sep=' ').tolist()
            else:
                newObstacle.dim = np.fromstring(geometryFirstChild[0].text, dtype=float, sep=' ').tolist()

            newObstacle.typeOfObstacle = model.attrib['name'].split('_')[1]

            allObstacles.append(newObstacle)

    return allObstacles


class ObstacleSensor(object):
    def __init__(self):
        self.loop_rate = rospy.Rate(1)
        self.all_obstacles = list()
        self.active_obstacles = list()
        self.uav_pose = None
        self.range = 20

        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.poseCallback)
        self.pub_obstacle_list = rospy.Publisher("~/active_obstacles", obstacleListMsg, queue_size=100)

    def poseCallback(self, data):
        self.uav_pose = data
        return

    def get_distance_uav_obstacle(self, obstacle):
        assert isinstance(obstacle, Obstacle)
        uav_pos = np.array([self.uav_pose.pose.position.x, self.uav_pose.pose.position.y, self.uav_pose.pose.position.z])
        obs_pos = np.array(obstacle.pose[:3])

        return np.linalg.norm((uav_pos, obs_pos))

    def get_active_obstacles(self):
        msg = obstacleListMsg()
        for obstacle_i in self.all_obstacles:
            if self.get_distance_uav_obstacle(obstacle_i) <= self.range:
                obs = obstacleMsg()
                obs.name = obstacle_i.name
                obs.typeOfObstacle = obstacle_i.typeOfObstacle
                obs.geometry = obstacle_i.geometry
                obs.pose = obstacle_i.pose
                obs.dim = obstacle_i.dim
                msg.obstacleList.append(obs)

        rospy.loginfo("Publishing active obstacles")
        self.pub_obstacle_list.publish(msg)

    def start(self):
        filename = '/home/daniel/OneDrive/APF - Erste Implementierung/gazebo_xml_test'  # ToDo: Make Param

        rospy.loginfo("Obstacle Sensor initialized")

        self.all_obstacles = readObstacleXML(filename)
        rospy.loginfo("All Obstacles have been read from the XML-File")

        rospy.sleep(1)
        while self.uav_pose is None:
            rospy.loginfo("Obstacle Sensor waiting for UAV Pose")
            self.loop_rate.sleep()

        rospy.loginfo("UAV Pose received")
        while not rospy.is_shutdown():
            self.get_active_obstacles()
            self.loop_rate.sleep()

        rospy.spin()  # Vermutlich nicht benoetigt


if __name__ == '__main__':
    rospy.init_node('sensor_obstacles')
    node = ObstacleSensor()
    node.start()
