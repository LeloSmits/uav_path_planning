#!/usr/bin/env python2

import numpy as np
import rospy
import typing

from uav_path_planning.srv import PotentialService, PotentialServiceResponse
from serialization import serialize_coordinates, deseralize_coordinates


def get_potential(req):
    array = deseralize_coordinates(req.req.data, 3)
    res = np.zeros(array.shape[0])

    res = np.sum(array, axis=1)

    resp = PotentialServiceResponse()
    resp.resp.data = res
    return resp


def main():
    rospy.init_node("get_potential_server")
    s = rospy.Service('get_potential', PotentialService, get_potential)
    print('service ready')
    rospy.spin()
    return


if __name__ == '__main__':
    main()



# import numpy as np
# import rospy
# import typing
#
# from uav_path_planning.srv import PotentialService, PotentialServiceResponse
#
#
# def get_potential(req):
#     array = req.req.position
#     resp = PotentialServiceResponse()
#     for position in array:
#         resp.resp.potential.append(np.sum(position.coordinate))
#     return resp
#
#
# def main():
#     rospy.init_node("get_potential_server")
#     s = rospy.Service('get_potential', PotentialService, get_potential)
#     print('service ready')
#     rospy.spin()
#     return
#
#
# if __name__ == '__main__':
#     main()
