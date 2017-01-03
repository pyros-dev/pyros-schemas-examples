#!/usr/bin/env python
from __future__ import absolute_import

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
    import pyros_msgs.opt_as_array  # This will duck punch the standard message type initialization code.

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
pyros_msgs.opt_as_array.duck_punch(HttpbinGetArgs, ['argopt'])
pyros_msgs.opt_as_array.duck_punch(HttpbinGet._request_class, ['headers'])

pyros_msgs.opt_as_array.duck_punch(HttpbinPostArgs, ['argopt'])
pyros_msgs.opt_as_array.duck_punch(HttpbinPostJson._request_class, ['headers'])
pyros_msgs.opt_as_array.duck_punch(HttpbinPostArgs, ['argopt'])
pyros_msgs.opt_as_array.duck_punch(HttpbinPostBody, ['testoptitem'])
pyros_msgs.opt_as_array.duck_punch(HttpbinPostBody2, ['subtestoptstring', 'subtestoptint', 'subtestoptfloat'])

import marshmallow
import requests

##########
# Services
##########


import requests



def httpbin_ip_callback(data):
    request_schema = pyros_schemas.create(HttpbinIp._request_class)
    data_obj, errors = request_schema.load(data)

    response = requests.get('http://httpbin.org/ip', data=data_obj)

    response_schema = pyros_schemas.create(HttpbinIp._response_class)
    response_ros, errors = response_schema.dump(response.json())

    return response_ros


def httpbin_useragent_callback(data):
    request_schema = pyros_schemas.create(HttpbinUserAgent._request_class)
    data_dict, errors = request_schema.load(data)

    response = requests.get('http://httpbin.org/user-agent', data=data_dict)

    response_schema = pyros_schemas.create(HttpbinUserAgent._response_class)
    response_ros, errors = response_schema.dump(response.json())

    return response_ros


def httpbin_headers_callback(data):
    request_schema = pyros_schemas.create(HttpbinHeaders._request_class)
    data_dict, errors = request_schema.load(data)

    response = requests.get('http://httpbin.org/headers', headers=data_dict.get('headers', {}))

    response_schema = pyros_schemas.create(HttpbinHeaders._response_class)
    response_ros, errors = response_schema.dump(response.json())

    return response_ros


# @pyros_schemas.with_explicitly_matched_type(HttpbinGetArgs)
# class HttpbinGetArgsSchema(marshmallow.Schema):
#     arg = pyros_schemas.RosString()  # test arg
#     argopt = pyros_schemas.RosOpt(pyros_schemas.RosString())  # test argopt
#     arglist = pyros_schemas.RosList(pyros_schemas.RosString())  # test arglist


def httpbin_get_callback(data):
    # # define schemas
    # @pyros_schemas.with_explicitly_matched_type(HttpbinGet._request_class)
    # class HttpbinGetRequestSchema(marshmallow.Schema):
    #     params = pyros_schemas.RosNested(HttpbinGetArgsSchema)
    #     headers = pyros_schemas.RosOpt(pyros_schemas.RosNested(HttpRequestHeadersSchema))
    #
    # @pyros_schemas.with_explicitly_matched_type(HttpbinGet._response_class)
    # class HttpbinGetResponseSchema(marshmallow.Schema):
    #     status = pyros_schemas.RosUInt16()
    #     args = pyros_schemas.RosNested(HttpbinGetArgsSchema)
    #     headers = pyros_schemas.RosNested(HttpRequestHeadersSchema)
    #     origin = pyros_schemas.RosString()
    #     url = pyros_schemas.RosString()
    #
    #     # massaging the response from request into a dict accepted by this schema
    #     @marshmallow.pre_load
    #     def parse_response(self, response_data):
    #         return {
    #             'status': response_data.status_code,
    #             'args': response_data.json().get('args'),
    #             # careful : headers here come from response data
    #             'headers': response_data.json().get('headers'),
    #             'origin': response_data.json().get('origin'),
    #             'url': response_data.json().get('url'),
    #         }
    #
    # # extract data
    # request_schema = HttpbinGetRequestSchema(strict=True)
    # marshalled, errors = request_schema.dump(data)
    #
    # if not errors:
    #     # relaying request to matching service
    #     response = requests.get('http://httpbin.org/get', headers=marshalled.get('headers'), params=marshalled.get('params'))
    # else:
    #     rospy.logwarn("Errors when parsing request: {0}".format(data))
    #     rospy.logwarn("  {0}".format(errors))
    #     return HttpbinGet._response_class(status=HttpStatusCode.BAD_REQUEST)
    # response_schema = HttpbinGetResponseSchema()
    # unmarshalled, errors = response_schema.load(response)
    #
    # if not errors:
    #     ros_response = unmarshalled
    # else:
    #     rospy.logwarn("Errors when parsing response: {0}".format(response))
    #     rospy.logwarn("  {0}".format(errors))
    #     ros_response = HttpbinGet._response_class(status=HttpStatusCode.BAD_GATEWAY)
    #
    # return ros_response


    request_schema = pyros_schemas.create(HttpbinGet._request_class)
    data_dict, errors = request_schema.load(data)

    print (" => {0}".format(data_dict))
    response = requests.get('http://httpbin.org/get', headers=data_dict.get('headers', {}), params=data_dict.get('params', {}))
    print (" <= {0}".format(response.json()))

    response_schema = pyros_schemas.create(HttpbinGet._response_class)
    response_ros, errors = response_schema.dump(response.json())

    return response_ros

#
# @pyros_schemas.with_explicitly_matched_type(HttpbinPostArgs)
# class HttpbinPostArgsSchema(marshmallow.Schema):
#     arg = pyros_schemas.RosString()  # test arg
#     argopt = pyros_schemas.RosOpt(pyros_schemas.RosString())  # test argopt
#     arglist = pyros_schemas.RosList(pyros_schemas.RosString())  # test arglist
#
#
# @pyros_schemas.with_explicitly_matched_type(HttpbinPostBody2)
# class HttpbinPostBody2Schema(marshmallow.Schema):
#     subteststring = pyros_schemas.RosString()  # test string
#     subtestoptstring = pyros_schemas.RosOpt(pyros_schemas.RosString())  # test opt string
#     subteststringarray = pyros_schemas.RosList(pyros_schemas.RosString())  # test list string
#
#     subtestint = pyros_schemas.RosInt32()  # test int
#     subtestoptint = pyros_schemas.RosOpt(pyros_schemas.RosInt32())  # test opt int
#     subtestintarray = pyros_schemas.RosList(pyros_schemas.RosInt32())  # test list int
#
#     subtestfloat= pyros_schemas.RosFloat32()  # test float
#     subtestoptfloat = pyros_schemas.RosOpt(pyros_schemas.RosFloat32())  # test opt float
#     subtestfloatarray = pyros_schemas.RosList(pyros_schemas.RosFloat32())  # test list float
#
#
# @pyros_schemas.with_explicitly_matched_type(HttpbinPostBody)
# class HttpbinPostBodySchema(marshmallow.Schema):
#     testitem = pyros_schemas.RosNested(HttpbinPostBody2Schema)  # test string
#     testoptitem = pyros_schemas.RosOpt(pyros_schemas.RosNested(HttpbinPostBody2Schema))  # test opt string
#     testitemarray = pyros_schemas.RosList(pyros_schemas.RosNested(HttpbinPostBody2Schema))  # test list string


def httpbin_postjson_callback(data):
    # # define schemas
    # @pyros_schemas.with_explicitly_matched_type(HttpbinPostJson._request_class)
    # class HttpbinPostRequestSchema(marshmallow.Schema):
    #     params = pyros_schemas.RosNested(HttpbinPostArgsSchema)
    #     headers = pyros_schemas.RosOpt(pyros_schemas.RosNested(HttpRequestHeadersSchema))
    #     json = pyros_schemas.RosNested(HttpbinPostBodySchema)
    #
    # @pyros_schemas.with_explicitly_matched_type(HttpbinPostJson._response_class)
    # class HttpbinPostResponseSchema(marshmallow.Schema):
    #     status = pyros_schemas.RosUInt16()
    #     args = pyros_schemas.RosNested(HttpbinPostArgsSchema)
    #     data = pyros_schemas.RosString()
    #     headers = pyros_schemas.RosNested(HttpRequestHeadersSchema)
    #     json = pyros_schemas.RosNested(HttpbinPostBodySchema)
    #     origin = pyros_schemas.RosString()
    #     url = pyros_schemas.RosString()
    #
    #     # massaging the response from request into a dict accepted by this schema
    #     @marshmallow.pre_load
    #     def parse_response(self, response):
    #         return {
    #             'status': response.status_code,
    #             'args': response.json().get('args'),
    #             'data': response.json().get('data'),
    #             'headers': response.json().get('headers'),
    #             'json': response.json().get('json'),
    #             'origin': response.json().get('origin'),
    #             'url': response.json().get('url'),
    #         }
    #
    #
    # # extract data
    # request_schema = HttpbinPostRequestSchema(strict=True)
    # marshalled, errors = request_schema.dump(data)
    #
    # if not errors:
    #     # relaying request to matching service
    #     response = requests.post('http://httpbin.org/post', headers=marshalled.get('headers'), params=marshalled.get('params'), json=marshalled.get('json'))
    # else:
    #     rospy.logwarn("Errors when parsing request: {0}".format(data))
    #     rospy.logwarn("  {0}".format(errors))
    #     return HttpbinPost._response_class(status=HttpStatusCode.BAD_REQUEST)
    # response_schema = HttpbinPostResponseSchema()
    # unmarshalled, errors = response_schema.load(response)
    #
    # if not errors:
    #     ros_response = unmarshalled
    # else:
    #     rospy.logwarn("Errors when parsing response: {0}".format(response))
    #     rospy.logwarn("  {0}".format(errors))
    #     ros_response = HttpbinPost._response_class(status=HttpStatusCode.BAD_GATEWAY)

    try:
        request_schema = pyros_schemas.create(HttpbinPostJson._request_class)
        data_dict, errors = request_schema.load(data)

        print (" => {0}".format(data_dict))
        response = requests.post('http://httpbin.org/post', headers=data_dict.get('headers', {}), params=data_dict.get('params'), json=data_dict.get('json'))
    except Exception as e:
        raise RostfulRequestServiceRequestException(e)

    if response.status_code == requests.status_codes.codes.OK:  # TODO : easy way to check all "OK" codes
        print (" <= {0}".format(response.json()))

        response_schema = pyros_schemas.create(HttpbinPostJson._response_class)
        response_ros, errors = response_schema.dump(response.json())

        return response_ros
    else:
        raise RostfulRequestServiceResponseException(response.status_code)



##########
# Node
##########

class HttpbinNode(object):
    def __init__(self):
        rospy.loginfo('Httpbin node started. [' + rospy.get_name() + ']')

        rospy.Service('~ip_service', HttpbinIp, httpbin_ip_callback)
        rospy.Service('~useragent_service', HttpbinUserAgent, httpbin_useragent_callback)
        rospy.Service('~headers_service', HttpbinHeaders, httpbin_headers_callback)
        rospy.Service('~get_service', HttpbinGet, httpbin_get_callback)
        rospy.Service('~postjson_service', HttpbinPostJson, httpbin_postjson_callback)
        # TODO
        #rospy.Service('~postform_service', HttpbinPostForm, httpbin_postform_callback)
        #rospy.Service('~postfiles_service', HttpbinPostFiles, httpbin_postfiles_callback)

    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('pyros_rosclient_httpbin')
    node = HttpbinNode()
    node.spin()
