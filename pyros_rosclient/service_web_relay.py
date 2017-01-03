#!/usr/bin/env python
from __future__ import absolute_import
from __future__ import print_function

import json

"""
 A very simple echo ROS node class.
 - echo from topic to echo_topic
 - echo service
"""

try:

    import roslib
    import rospy
    import std_msgs.msg as std_msgs
    import pyros_schemas

    from pyros_rosclient.msg import HttpStatusCode, HttpRequestHeaders, HttpResponseHeaders, HttpbinGetArgs, HttpbinPostArgs, HttpbinPostBody, HttpbinPostBody2
    from pyros_rosclient.srv import HttpbinIp, HttpbinUserAgent, HttpbinHeaders, HttpbinGet, HttpbinPostJson, HttpbinPostForm, HttpbinPostFiles

except ImportError:
    # Because we need to access Ros message types here (from ROS env or from virtualenv, or from somewhere else)
    import pyros_setup
    # We rely on default configuration in the environment to point us ot the proper distro and workspace
    pyros_setup.configurable_import().configure().activate()

    import roslib
    import rospy
    import std_msgs.msg as std_msgs
    import pyros_schemas

    from pyros_rosclient.msg import HttpStatusCode, HttpRequestHeaders, HttpResponseHeaders, HttpbinGetArgs, HttpbinPostArgs, HttpbinPostBody, HttpbinPostBody2
    from pyros_rosclient.srv import HttpbinIp, HttpbinUserAgent, HttpbinHeaders, HttpbinGet, HttpbinPostJson, HttpbinPostForm, HttpbinPostFiles

import marshmallow
import requests


class RosRequestSchemaMeta(type):
    @classmethod
    def __prepare__(mcs, name, bases, **kwargs):
        print('  Meta.__prepare__(mcs=%s, name=%r, bases=%s, **%s)' % (mcs, name, bases, kwargs))
        return {}

    def __new__(mcs, name, bases, attrs, **kwargs):
        print('  Meta.__new__(mcs=%s, name=%r, bases=%s, attrs=[%s], **%s)' % (mcs, name, bases, ', '.join(attrs), kwargs))
        return super(RosRequestSchemaMeta, mcs).__new__(mcs, name, bases, attrs)

    def __init__(cls, name, bases, attrs, **kwargs):
        print('  Meta.__init__(cls=%s, name=%r, bases=%s, attrs=[%s], **%s)' % (cls, name, bases, ', '.join(attrs), kwargs))
        return super(RosRequestSchemaMeta, cls).__init__(name, bases, attrs)

    def __call__(cls, *args, **kwargs):
        print('  Meta.__call__(cls=%s, args=%s, kwargs=%s)' % (cls, args, kwargs))
        return super(RosRequestSchemaMeta, cls).__call__(*args, **kwargs)


class RosResponseSchemaMeta(type):
    @classmethod
    def __prepare__(mcs, name, bases, **kwargs):
        print('  Meta.__prepare__(mcs=%s, name=%r, bases=%s, **%s)' % (mcs, name, bases, kwargs))
        return {}

    def __new__(mcs, name, bases, attrs, **kwargs):
        print('  Meta.__new__(mcs=%s, name=%r, bases=%s, attrs=[%s], **%s)' % (mcs, name, bases, ', '.join(attrs), kwargs))
        return super(RosResponseSchemaMeta, mcs).__new__(mcs, name, bases, attrs)

    def __init__(cls, name, bases, attrs, **kwargs):
        print('  Meta.__init__(cls=%s, name=%r, bases=%s, attrs=[%s], **%s)' % (cls, name, bases, ', '.join(attrs), kwargs))
        return super(RosResponseSchemaMeta, cls).__init__(name, bases, attrs)

    def __call__(cls, *args, **kwargs):
        print('  Meta.__call__(cls=%s, args=%s, kwargs=%s)' % (cls, args, kwargs))
        return super(RosResponseSchemaMeta, cls).__call__(*args, **kwargs)


def service_web_relay(service_class, web_url):
    def wrap(f):
        print("Inside wrap()")



        # class RequestSchema(marshmallow.Schema):
        #     pass
        #
        # class ResponseSchema(marshmallow.Schema):
        #     status = pyros_schemas.RosUInt16()
        #     ip = pyros_schemas.RosString()
        #
        #     # massaging the response from request into a dict accepted by this schema
        #     @marshmallow.pre_load
        #     def parse_response(self, response_data):
        #         return {
        #             'status': response_data.status_code,
        #             'ip': response_data.json().get('origin')
        #         }
        #
        #     @marshmallow.post_load
        #     def make_rosResponse(self, response_dict):
        #         return HttpbinIp._response_class(**response_dict)

        def wrapped_f(*args):
            assert len(args) == 1 or print("Multiple parameters passed to a function that should be a rOS service callback")  # TODO : move that one level up to do it at instanciation
            print("Inside wrapped_f()")
            print("Decorator arguments:", service_class, web_url)
            f(*args)
            print("After f(*args)")

            # extract data
            request_schema = RequestSchema(strict=True)
            marshalled, errors = request_schema.dump(data)

            if not errors:
                # relaying request to matching service
                response = requests.get(web_url, data=marshalled)
            else:
                rospy.logwarn("Errors when parsing request: {0}".format(data))
                rospy.logwarn("  {0}".format(errors))
                # TODO : return python eception like rospy
                return HttpbinIp._response_class(status=HttpStatusCode.BAD_REQUEST)

            response_schema = ResponseSchema()
            unmarshalled, errors = response_schema.load(response)

            if not errors:
                response = unmarshalled
            else:
                rospy.logwarn("Errors when parsing response: {0}".format(response))
                rospy.logwarn("  {0}".format(errors))
                # TODO : return python eception like rospy
                response = HttpbinIp._response_class(status=HttpStatusCode.BAD_GATEWAY)

            return response



        return wrapped_f

    return wrap


