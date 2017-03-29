#!/usr/bin/env python
from __future__ import absolute_import, division, print_function

try:
    # easy way to check we have loaded the ROS environment
    import rospy
except ImportError:
    import pyros_setup
    # We rely on default configuration in the environment to point us to the proper distro and workspace
    pyros_setup.configurable_import().configure().activate()
    import rospy

from pyros_httpbin.node import HttpbinNode


if __name__ == '__main__':
    rospy.init_node('pyros_httpbin')
    node = HttpbinNode()
    node.spin()
