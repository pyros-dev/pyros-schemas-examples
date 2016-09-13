from __future__ import absolute_import
from __future__ import print_function

import sys

try:
    import genpy
    import rospy
    import roslaunch
    import pyros_rosclient
    import pyros_utils

except ImportError:
    # Because we need to access Ros message types here (from ROS env or from virtualenv, or from somewhere else)
    import pyros_setup
    # We rely on default configuration in the environment to point us ot the proper distro and workspace
    pyros_setup.configurable_import().configure().activate()
    import genpy
    import rospy
    import roslaunch
    import pyros_rosclient
    import pyros_utils

import unittest
import nose
import logging
import netaddr



# test node process not setup by default (rostest dont need it here)
httpbin_process = None


# This should have the same effect as the <name>.test file for rostest.
# Should be used only by nose ( or other python test tool )
def setup_module():
    if not pyros_utils.rostest_nose.is_rostest_enabled():
        pyros_utils.rostest_nose.rostest_nose_setup_module()

        # Start roslaunch
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()

        # start required nodes - needs to match the content of *.test files for rostest to match

        global httpbin_process

        httpbin_node = roslaunch.core.Node('pyros_rosclient', 'httpbin_node.py', name='httpbin_node')
        try:
            httpbin_process = launch.launch(httpbin_node)
        except roslaunch.RLException as rlexc:
            raise

        # we still need a node to run tests
        rospy.init_node('TestHttpBin', anonymous=True, disable_signals=True)
        # CAREFUL : this should be done only once per PROCESS
        # Here we enforce TEST MODULE 1<->1 PROCESS. ROStest style


def teardown_module():
    if not pyros_utils.rostest_nose.is_rostest_enabled():
        # finishing all process are finished
        if httpbin_process is not None:
            httpbin_process.stop()

        pyros_utils.rostest_nose.rostest_nose_teardown_module()


class TestHttpBin(unittest.TestCase):

    def test_ip(self):
        # following http://wiki.ros.org/ROS/Tutorials/WritingServiceClient(python)
        rospy.wait_for_service('httpbin_ip_service')
        try:
            httpbin_ip = rospy.ServiceProxy('httpbin_ip_service', pyros_rosclient.HttpbinIp)
            resp = httpbin_ip()

            try:
                addr = netaddr.IPAddress(resp)
            except:
                assert False and "Failed !"
        except rospy.ServiceException as e:
            print("Service call failed: {0}".format(e))



    def test_get(self):
        pass

    def test_post(self):
        pass


# Just in case we run this directly
if __name__ == '__main__':
    pyros_utils.rostest_or_nose_main('pyros_rosclient', 'Httpbin', TestHttpBin, sysargv=sys.argv)
