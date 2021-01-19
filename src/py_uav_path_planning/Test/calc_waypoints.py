#!/usr/bin/env python2
import rospy

from uav_path_planning.srv import potential_field_msg, potential_field_msgResponse
from rospy.numpy_msg import numpy_msg
from uav_path_planning.msg import onedimarray


def calculate_coordinates(req):
    sum_resp = potential_field_msgResponse()
    sum_resp.resp.data = req.req.data

    return sum_resp

# pub = rospy.Publisher('chatter2', Float64MultiArray, queue_size=10)
#     data_to_send = Float64MultiArray()  # the data to be sent, initialise the array
#     data_to_send.data = array # assign the array with the value you want to send
#     pub.publish(data_to_send)

def add_coordinates_server():
    rospy.init_node('add_coordinates_server')
    s = rospy.Service('add_coordinates', potential_field_msg, calculate_coordinates)
    rospy.spin()


if __name__ == "__main__":
    add_coordinates_server()
