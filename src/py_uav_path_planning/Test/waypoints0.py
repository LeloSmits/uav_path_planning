#!/usr/bin/env python2
import rospy
import numpy as np

from uav_path_planning.srv import potential_field_msg, potential_field_msgRequest
from uav_path_planning.msg import onedimarray
from rospy.numpy_msg import numpy_msg


def add_my_coordinates(coo_list1):
    coordinate = potential_field_msgRequest()
    coordinate.req.data = coo_list1
    add_coordinates = rospy.ServiceProxy('add_coordinates', potential_field_msg)
    resp1 = add_coordinates(coordinate)
    return resp1
    # except rospy.ServiceException as e:
    #     print("Service call failed")


if __name__ == "__main__":
    # coordinate_1 = [5, 5, 6]
    coordinate_2 = [5.0, 5.0, 5.0, 6.0, 8.0, 9.0]
    resp = add_my_coordinates(coordinate_2)
    print(resp.resp.data)