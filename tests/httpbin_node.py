#!/usr/bin/env python
from __future__ import absolute_import

import functools

"""
 A very simple echo ROS node class.
 - echo from topic to echo_topic
 - echo service
"""

import roslib
import rospy
import std_msgs.msg as std_msgs


# TODO : get rid of this somehow ( dynamic generation or integration of more basic services in ROS )
from pyros_test.srv import HttpbinIp, HttpbinGet, HttpbinPost

import marshmallow
import pyros_schemas

import requests


class HttpbinIpRequestSchema(marshmallow.Schema):
    pass


class HttpbinIpResponseSchema(marshmallow.Schema):
    status_code = pyros_schemas.HttpStatusCode()
    ip = pyros_schemas.std_string()
    pass



class HttpbinNode(object):
    def __init__(self):
        rospy.loginfo('Httpbin node started. [' + rospy.get_name() + ']')

        httpbin_ip_service = rospy.Service('httpbin_ip_service', HttpbinIp, self.httpbin_ip_callback)

    def httpbin_ip_callback(self, data):
        # extract data
        request_schema = HttpbinIpRequestSchema()
        marshalled = request_schema.dump(data)

        # relaying request to matching service
        response = requests.get('http://httpbin.org/ip', data=marshalled)

        response_schema = HttpbinIpResponseSchema()
        unmarshalled = response_schema.load(response)


    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('pyros_test_echo')
    node = HttpbinNode()
    node.spin()
