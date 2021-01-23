#!/usr/bin/env python2
import rospy
import numpy as np

from uav_path_planning.srv import potential_field_msg, potential_field_msgRequest
from uav_path_planning.msg import onedimarray
from rospy.numpy_msg import numpy_msg


def add_my_coordinates(coo_list1):

    coordinate = potential_field_msgRequest()
    coordinate.req.data = coo_list1
    coordinate.mode = 0

    get_apf_potential = rospy.ServiceProxy('get_apf_potential', potential_field_msg)

    resp1 = get_apf_potential(coordinate)

    return resp1
    # except rospy.ServiceException as e:
    #     print("Service call failed")


if __name__ == "__main__":
    # coordinate_1 = [5, 5, 6]
    coordinate_2 = [3.0, 1.0, 0.5, 0.0, 0.0, 0.5]

    resp = add_my_coordinates(coordinate_2)