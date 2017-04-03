from __future__ import absolute_import
from __future__ import print_function

import sys


try:
    import genpy
    import rospy
    import roslaunch
    import pyros_httpbin.msg, pyros_httpbin.srv
    import pyros_utils
    import pyros_schemas

except ImportError:
    # Because we need to access Ros message types here (from ROS env or from virtualenv, or from somewhere else)
    import pyros_setup
    # We rely on default configuration in the environment to point us ot the proper distro and workspace
    pyros_setup.configurable_import().configure().activate()
    import genpy
    import rospy
    import roslaunch
    import pyros_httpbin.msg, pyros_httpbin.srv
    import pyros_utils
    import pyros_schemas

import unittest
import nose
import logging
import netaddr
import marshmallow

# test node process not setup by default (rostest dont need it here)
httpbin_process = None
httpbin_node_name = 'httpbin_node'

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

        httpbin_node = roslaunch.core.Node('pyros_httpbin', 'httpbin.py', name=httpbin_node_name)
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

    def test1_ip(self):
        ip_service_name = '/' + httpbin_node_name + '/ip_service'
        # following http://wiki.ros.org/ROS/Tutorials/WritingServiceClient(python)
        rospy.wait_for_service(ip_service_name)

        httpbin_ip = rospy.ServiceProxy(ip_service_name, pyros_httpbin.srv.HttpbinIp)
        resp = httpbin_ip()

        addr = netaddr.IPAddress(resp.origin)
        # Test considered passing if we can validate an ip in resp

    def test1_ip_raises(self):
        ip_service_name = '/' + httpbin_node_name + '/ip_service'
        # following http://wiki.ros.org/ROS/Tutorials/WritingServiceClient(python)
        rospy.wait_for_service(ip_service_name)

        httpbin_ip = rospy.ServiceProxy(ip_service_name, pyros_httpbin.srv.HttpbinIp)
        with nose.tools.assert_raises(TypeError) as cm:
            resp = httpbin_ip({'wrong': 'data'})
        ex = cm.exception  # raised exception is available through exception property of context
        assert ex.message == "Invalid number of arguments, args should be [] args are({'wrong': 'data'},)" or print(ex.message)

    def test2_useragent(self):
        useragent_service_name = '/' + httpbin_node_name + '/useragent_service'
        # following http://wiki.ros.org/ROS/Tutorials/WritingServiceClient(python)
        rospy.wait_for_service(useragent_service_name)

        httpbin_useragent = rospy.ServiceProxy(useragent_service_name, pyros_httpbin.srv.HttpbinUserAgent)
        resp = httpbin_useragent()

        # currently the webgateway node uses python-request user-agent string.
        assert 'python-requests' in resp.user_agent, resp.user_agent

    def test2_useragent_raises(self):
        useragent_service_name = '/' + httpbin_node_name + '/useragent_service'
        # following http://wiki.ros.org/ROS/Tutorials/WritingServiceClient(python)
        rospy.wait_for_service(useragent_service_name)

        httpbin_useragent = rospy.ServiceProxy(useragent_service_name, pyros_httpbin.srv.HttpbinUserAgent)
        with nose.tools.assert_raises(TypeError) as cm:
            resp = httpbin_useragent({'wrong': 'data'})
        ex = cm.exception  # raised exception is available through exception property of context
        assert ex.message == "Invalid number of arguments, args should be [] args are({'wrong': 'data'},)", ex.message

    def test3_headers(self):
        headers_service_name = '/' + httpbin_node_name + '/headers_service'
        # following http://wiki.ros.org/ROS/Tutorials/WritingServiceClient(python)
        rospy.wait_for_service(headers_service_name)

        httpbin_headers = rospy.ServiceProxy(headers_service_name, pyros_httpbin.srv.HttpbinHeaders)
        resp = httpbin_headers()

        #  We need to confirm we get the headers that should have been set by python-requests
        assert resp.headers.Accept == ['*/*']
        assert resp.headers.Accept_Encoding == ['gzip, deflate, compress']
        assert resp.headers.Host == ['httpbin.org']
        assert 'python-requests' in resp.headers.User_Agent[0]
        # we currently allow other headers to vary

    def test4_get(self):

        get_service_name = '/' + httpbin_node_name + '/get_service'
        # following http://wiki.ros.org/ROS/Tutorials/WritingServiceClient(python)
        rospy.wait_for_service(get_service_name)

        httpbin_get = rospy.ServiceProxy(get_service_name, pyros_httpbin.srv.HttpbinGet)
        req = pyros_httpbin.srv.HttpbinGetRequest(
            params=pyros_httpbin.msg.HttpbinGetArgs(
                arg='testarg',
                # argopt='', # optional, let not care about it
                arglist=['arg1', 'arg2']  # httpbin removes the list if only one arg here, but we do expect list in response.
            ),
            # headers=pyros_httpbin.msg.HttpRequestHeaders(), # optional, let not care about it
        )
        resp = httpbin_get(req)

        #  We need to confirm we get the headers that should have been set by python-requests
        assert resp.headers.Accept == ['*/*']
        assert resp.headers.Accept_Encoding == ['gzip, deflate, compress']
        assert resp.headers.Host == ['httpbin.org']
        assert 'python-requests' in resp.headers.User_Agent[0]
        # we currently allow other headers to vary

        # we need to confirm the arguments returned
        assert resp.args.arg == 'testarg'
        assert resp.args.arglist == ['arg1', 'arg2']

    def test5_postjson(self):

        post_service_name = '/' + httpbin_node_name + '/postjson_service'
        # following http://wiki.ros.org/ROS/Tutorials/WritingServiceClient(python)
        rospy.wait_for_service(post_service_name)

        httpbin_post = rospy.ServiceProxy(post_service_name, pyros_httpbin.srv.HttpbinPostJson)
        ros_data = pyros_httpbin.msg.HttpbinPostBody(
                testitem=pyros_httpbin.msg.HttpbinPostBody2(
                    subteststring='teststr',
                    # subtestoptstring='', optional, lets not care about it
                    subteststringarray=['str1', 'str2', 'str3'],
                    subtestint=42,
                    # subtestoptint=21, #optional, lets not care about it
                    subtestintarray=[4, 2, 1],
                    subtestfloat=42.,
                    # subtestoptfloat=21., #optional, lets not care about it
                    subtestfloatarray=[4., 2., 1.],
                ),
                # testoptitem optional lets not care about it
                testitemarray=[
                    pyros_httpbin.msg.HttpbinPostBody2(
                        subteststring='teststr1',
                        # subtestoptstring='', optional, lets not care about it
                        subteststringarray=['str1', 'str2', 'str3'],
                        subtestint=42,
                        # subtestoptint=21, #optional, lets not care about it
                        subtestintarray=[4, 2, 1],
                        subtestfloat=42.,
                        # subtestoptfloat=21., #optional, lets not care about it
                        subtestfloatarray=[4., 2., 1.],
                    ),
                    pyros_httpbin.msg.HttpbinPostBody2(
                        subteststring='teststr2',
                        # subtestoptstring='', optional, lets not care about it
                        subteststringarray=['str1', 'str2', 'str3'],
                        subtestint=42,
                        # subtestoptint=21, #optional, lets not care about it
                        subtestintarray=[4, 2, 1],
                        subtestfloat=42.,
                        # subtestoptfloat=21., #optional, lets not care about it
                        subtestfloatarray=[4., 2., 1.],
                    ),
                ]
            )
        req = pyros_httpbin.srv.HttpbinPostJson._request_class(
            params=pyros_httpbin.msg.HttpbinPostArgs(
                arg='testarg',
                # argopt='', # optional, let not care about it
                arglist=['arg1', 'arg2']
                # httpbin removes the list if only one arg here, but we do expect list in response.
                # TODO : fix this...
            ),
            # headers=pyros_httpbin.msg.HttpRequestHeaders(), # optional, let not care about it
            json=ros_data
        )
        resp = httpbin_post(req)

        #  We need to confirm we get the headers that should have been set by python-requests
        assert resp.headers.Accept == ['*/*']
        assert resp.headers.Accept_Encoding == ['gzip, deflate, compress']
        assert resp.headers.Host == ['httpbin.org']
        assert 'python-requests' in resp.headers.User_Agent[0]
        # we currently allow other headers to vary

        # we need to confirm the arguments returned
        assert resp.args.arg == 'testarg'
        assert resp.args.arglist == ['arg1', 'arg2']

        # we need to confirm the data returned (careful ros messages change array into tuple, except for strings...)
        assert resp.json.testitem.subteststring == 'teststr'
        assert resp.json.testitem.subteststringarray == ['str1', 'str2', 'str3']
        assert resp.json.testitem.subtestint == 42
        assert resp.json.testitem.subtestintarray == (4, 2, 1)
        assert resp.json.testitem.subtestfloat == 42.
        assert resp.json.testitem.subtestfloatarray == (4., 2., 1.)
        assert len(resp.json.testitemarray) == 2

        assert resp.json.testitemarray[0].subteststring == 'teststr1'
        assert resp.json.testitemarray[0].subteststringarray == ['str1', 'str2', 'str3']
        assert resp.json.testitemarray[0].subtestint == 42
        assert resp.json.testitemarray[0].subtestintarray == (4, 2, 1)
        assert resp.json.testitemarray[0].subtestfloat == 42.
        assert resp.json.testitemarray[0].subtestfloatarray == (4., 2., 1.)

        assert resp.json.testitemarray[1].subteststring == 'teststr2'
        assert resp.json.testitemarray[1].subteststringarray == ['str1', 'str2', 'str3']
        assert resp.json.testitemarray[1].subtestint == 42
        assert resp.json.testitemarray[1].subtestintarray == (4, 2, 1)
        assert resp.json.testitemarray[1].subtestfloat == 42.
        assert resp.json.testitemarray[1].subtestfloatarray == (4., 2., 1.)

        # # final serialization testing of content of data field in response
        # # TODO : avoid copying httpbin node content -> refactor
        # @pyros_schemas.with_explicitly_matched_type(pyros_httpbin.msg.HttpbinPostBody2)
        # class HttpbinPostBody2Schema(marshmallow.Schema):
        #     subteststring = pyros_schemas.RosString()  # test string
        #     subtestoptstring = pyros_schemas.RosOpt(pyros_schemas.RosString())  # test opt string
        #     subteststringarray = pyros_schemas.RosList(pyros_schemas.RosString())  # test list string
        #
        #     subtestint = pyros_schemas.RosInt32()  # test int
        #     subtestoptint = pyros_schemas.RosOpt(pyros_schemas.RosInt32())  # test opt int
        #     subtestintarray = pyros_schemas.RosList(pyros_schemas.RosInt32())  # test list int
        #
        #     subtestfloat = pyros_schemas.RosFloat32()  # test float
        #     subtestoptfloat = pyros_schemas.RosOpt(pyros_schemas.RosFloat32())  # test opt float
        #     subtestfloatarray = pyros_schemas.RosList(pyros_schemas.RosFloat32())  # test list float
        #
        # @pyros_schemas.with_explicitly_matched_type(pyros_httpbin.msg.HttpbinPostBody)
        # class HttpbinPostBodySchema(marshmallow.Schema):
        #     testitem = pyros_schemas.RosNested(HttpbinPostBody2Schema)  # test string
        #     testoptitem = pyros_schemas.RosOpt(pyros_schemas.RosNested(HttpbinPostBody2Schema))  # test opt string
        #     testitemarray = pyros_schemas.RosList(pyros_schemas.RosNested(HttpbinPostBody2Schema))  # test list string
        #
        # dataschema = HttpbinPostBodySchema()
        # resp_data_str, errors = dataschema.dumps(resp.json())
        # assert not errors and resp.data == resp_data_str

# TODO checking status and exceptions...

# Just in case we run this directly
if __name__ == '__main__':
    pyros_utils.rostest_or_nose_main('pyros_httpbin', 'Httpbin', TestHttpBin, sysargv=sys.argv)
