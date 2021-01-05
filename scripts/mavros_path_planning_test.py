#!/usr/bin/env python2

import math
import numpy as np
import rospy
import typing
from threading import Thread

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header


def _send_some_waypoints(self):
    new_wp_msg = PoseStamped()
    new_wp_msg.header = Header()

    for i in range(100):
        new_wp_msg.header.stamp = rospy.Time.now()

        new_wp_msg.pose.position.x = 0  # x
        new_wp_msg.pose.position.y = 0  # y
        new_wp_msg.pose.position.z = .5  # z

        self._pub_new_wp.publish(new_wp_msg)
    return


self._send_some_waypoints()
rospy.loginfo(self.name + ": Finished sending dummy waypoints at (0, 0, 0.5)")
