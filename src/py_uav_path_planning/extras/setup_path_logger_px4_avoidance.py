#!/usr/bin/env python2

import rospy
import subprocess


def setup_px4_path_logger():
    rospy.set_param("path_logger_active", True)
    bashCommand = "rosnode kill /offb_node"
    process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
    output, error = process.communicate()
    return
