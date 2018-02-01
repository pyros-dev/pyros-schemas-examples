#!/usr/bin/env python
#
# License: Yujin
#
##############################################################################
# Documentation
##############################################################################
"""
Simple utility to start a gopher scheduler ROS node from the command line.
"""
##############################################################################
# Imports
##############################################################################

import sys
import argparse
import json
import requests
try:
    import rocon_console.console as console
    import pyros_schemas
    import pyros_httpbin

    import rospy
    from pyros_httpbin.srv import HttpbinPostJson, HttpbinPostJsonRequest, HttpbinPostJsonResponse
    from pyros_httpbin.msg import HttpRequestHeaders, HttpbinPostArgs, HttpbinPostBody, HttpbinPostBody2
    import pyros_msgs.opt_as_array  # This will duck punch the standard message type initialization code.

except ImportError:
    # Because we need to access Ros message types here (from ROS env or from virtualenv, or from somewhere else)
    import pyros_setup
    # We rely on default configuration in the environment to point us ot the proper distro and workspace
    pyros_setup.configurable_import().configure().activate()

    import rocon_console.console as console
    import pyros_schemas
    import pyros_httpbin

    import rospy
    from pyros_httpbin.srv import HttpbinPostJson, HttpbinPostJsonRequest, HttpbinPostJsonResponse
    from pyros_httpbin.msg import HttpRequestHeaders, HttpbinPostArgs, HttpbinPostBody, HttpbinPostBody2
    import pyros_msgs.opt_as_array  # This will duck punch the standard message type initialization code.


# patching messages types with optional fields
pyros_msgs.opt_as_array.duck_punch(HttpRequestHeaders, [
    'User_Agent',
    'Accept',
    'Accept_Encoding',
    'Accept_Language',
    'Host',
    'Referer',
    'Upgrade_Insecure_Requests',
])

pyros_msgs.opt_as_array.duck_punch(HttpbinPostArgs, ['argopt'])
pyros_msgs.opt_as_array.duck_punch(HttpbinPostJson._request_class, ['headers'])
pyros_msgs.opt_as_array.duck_punch(HttpbinPostArgs, ['argopt'])
pyros_msgs.opt_as_array.duck_punch(HttpbinPostBody, ['testoptitem'])
pyros_msgs.opt_as_array.duck_punch(HttpbinPostBody2, ['subtestoptstring', 'subtestoptint', 'subtestoptfloat'])
##############################################################################
# Helpers
##############################################################################


class TestPyrosSchemas(object):

    def __init__(self):
        rospy.Service('/test/pyros_schemas', HttpbinPostJson, self.test_pyros_schemas2)
        rospy.wait_for_service('/test/pyros_schemas', timeout=10)
        self.test_service_proxy = rospy.ServiceProxy('/test/pyros_schemas', HttpbinPostJson)

    @pyros_schemas.with_service_schemas(HttpbinPostJson)
    def test_pyros_schemas2(self, data, data_dict, errors):
        print (" => {0}".format(data_dict))  # to help with debugging
        h = data_dict.get('headers', {})
        h.update({"Content-type": "application/json"})
        p = data_dict.get('params')
        d = data_dict.get('json')
        response = requests.post('http://httpbin.org/post', headers=h, params=p, data=json.dumps(d))

        if response.status_code == requests.status_codes.codes.OK:  # TODO : easy way to check all "OK" codes
            print (" <= {0}".format(response.json()))
            return response.json()
        else:
            raise StatusCodeException(response.status_code)

    def test_pyros_schemas(self, req):
        resp = HttpbinPostJsonResponse()
        return resp

    def spin(self):
        count = 0
        while not rospy.core.is_shutdown():
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
            resp = self.test_service_proxy(req)
            rospy.loginfo("COUNT: {0}".format(count))
            rospy.loginfo("RESP: {0}".format(resp))
            count += 1
            rospy.sleep(0.01)

if __name__ == '__main__':
    rospy.init_node('test_pyros_schemas', log_level=rospy.INFO)
    _node = TestPyrosSchemas()
    _node.spin()
