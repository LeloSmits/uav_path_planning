#!/usr/bin/env python2

import numpy as np
import rospy
import typing

from uav_path_planning.srv import PotentialService, PotentialServiceRequest
from serialization import serialize_coordinates, deseralize_coordinates



def main(req):
    rospy.wait_for_service("get_potential")
    get_potential = rospy.ServiceProxy("get_potential", PotentialService)
    resp = get_potential(req)
    return resp


if __name__ == '__main__':
    array = np.ones((10, 3))
    message = PotentialServiceRequest()
    message.req.data = serialize_coordinates(array)
    print(message)
    res = main(message)
    print(res)


# import numpy as np
# import rospy
# import typing
#
# from uav_path_planning.srv import PotentialService, PotentialServiceRequest
# from uav_path_planning.msg import ThreeDimPos
#
#
# def main(req):
#     rospy.wait_for_service("get_potential")
#     get_potential = rospy.ServiceProxy("get_potential", PotentialService)
#     resp = get_potential(req)
#     return resp
#
#
# if __name__ == '__main__':
#     array = np.ones((10, 3))
#     message = PotentialServiceRequest()
#     for i, coord in enumerate(array):
#         pos = ThreeDimPos()
#         pos.coordinate = coord
#         message.req.position.append(pos)
#     print(message)
#     res = main(message)
#     print(res)
