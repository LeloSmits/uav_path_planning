#!/usr/bin/env python2

import math
import numpy as np
import rospy
import typing
from threading import Thread

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool, SetModeResponse


current_state = State()


def current_state_callback(data):
    global current_state
    current_state = data


def main():
    def send_waypoint():
        new_wp_msg = PoseStamped()
        new_wp_msg.header = Header()

        new_wp_msg.header.stamp = rospy.Time.now()

        new_wp_msg.pose.position.x = dummy_pos[0]  # x
        new_wp_msg.pose.position.y = dummy_pos[1]  # y
        new_wp_msg.pose.position.z = dummy_pos[2]  # z

        wp_publisher.publish(new_wp_msg)
        return

    script_name = "mavros_path_planning_test"
    rospy.init_node(script_name)

    dummy_pos = np.array([0, 0, .5])
    publish_rate = rospy.Rate(20)

    state_subscriber = rospy.Subscriber("mavros/state", State, current_state_callback)

    wp_publisher = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

    arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
    set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)

    for i in range(100):
        send_waypoint()

    offb_set_mode = SetMode()
    offb_set_mode.custom_mode = "OFFBOARD"

    arm_cmd = CommandBool()
    arm_cmd.value = True

    last_request = rospy.Time.now()

    while not rospy.is_shutdown():
        if current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_request > rospy.Duration(5)):
            # print offb_set_mode._response_class.mode_sent
            res = set_mode_client(0, offb_set_mode.custom_mode)
            if res and res.mode_sent:
                rospy.loginfo("Offboard enabled")
            last_request = rospy.Time.now()
        else:
            if not current_state.armed and (rospy.Time.now() - last_request > rospy.Duration(5)):
                res = arming_client(True)
                if res and res.success:
                    rospy.loginfo("Vehicle armed")
                last_request = rospy.Time.now()

        send_waypoint()
        publish_rate.sleep()

    return 0

    # rospy.loginfo(script_name + ": Finished sending dummy waypoints at " + dummy_pos.tostring())


if __name__ == '__main__':
    main()