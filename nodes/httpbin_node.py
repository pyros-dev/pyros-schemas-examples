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

##########
# Services
##########


def httpbin_ip_callback(data):

    # define schemas
    class HttpbinIpRequestSchema(marshmallow.Schema):
        pass

    class HttpbinIpResponseSchema(marshmallow.Schema):
        status = pyros_schemas.RosUInt16()
        ip = pyros_schemas.RosString()

        # massaging the response from request into a dict accepted by this schema
        @marshmallow.pre_load
        def parse_response(self, response_data):
            return {
                'status': response_data.status_code,
                'ip': response_data.json().get('origin')
            }

        @marshmallow.post_load
        def make_rosResponse(self, response_dict):
            return HttpbinIp._response_class(**response_dict)


    # extract data
    request_schema = HttpbinIpRequestSchema(strict=True)
    marshalled, errors = request_schema.dump(data)

    if not errors:
        # relaying request to matching service
        response = requests.get('http://httpbin.org/ip', data=marshalled)
    else:
        rospy.logwarn("Errors when parsing request: {0}".format(data))
        rospy.logwarn("  {0}".format(errors))
        return HttpbinIp._response_class(status=HttpStatusCode.BAD_REQUEST)
    response_schema = HttpbinIpResponseSchema()
    unmarshalled, errors = response_schema.load(response)

    if not errors:
        response = unmarshalled
    else:
        rospy.logwarn("Errors when parsing response: {0}".format(response))
        rospy.logwarn("  {0}".format(errors))
        response = HttpbinIp._response_class(status=HttpStatusCode.BAD_GATEWAY)

    return response


def httpbin_useragent_callback(data):
    # define schemas
    class HttpbinUserAgentRequestSchema(marshmallow.Schema):
        pass

    class HttpbinUserAgentResponseSchema(marshmallow.Schema):
        status = pyros_schemas.RosUInt16()
        user_agent = pyros_schemas.RosString()

        # massaging the response from request into a dict accepted by this schema
        @marshmallow.pre_load
        def parse_response(self, response_data):
            return {
                'status': response_data.status_code,
                'user_agent': response_data.json().get('user-agent')
            }

        @marshmallow.post_load
        def make_rosResponse(self, response_dict):
            return HttpbinUserAgent._response_class(**response_dict)

    # extract data
    request_schema = HttpbinUserAgentRequestSchema(strict=True)
    marshalled, errors = request_schema.dump(data)

    if not errors:
        # relaying request to matching service
        response = requests.get('http://httpbin.org/user-agent', data=marshalled)
    else:
        rospy.logwarn("Errors when parsing request: {0}".format(data))
        rospy.logwarn("  {0}".format(errors))
        return HttpbinUserAgent._response_class(status=HttpStatusCode.BAD_REQUEST)
    response_schema = HttpbinUserAgentResponseSchema()
    unmarshalled, errors = response_schema.load(response)

    if not errors:
        response = unmarshalled
    else:
        rospy.logwarn("Errors when parsing response: {0}".format(response))
        rospy.logwarn("  {0}".format(errors))
        response = HttpbinUserAgent._response_class(status=HttpStatusCode.BAD_GATEWAY)

    return response


@pyros_schemas.with_explicitly_matched_type(HttpRequestHeaders)
class HttpRequestHeadersSchema(marshmallow.Schema):
    Accept = pyros_schemas.RosOpt(pyros_schemas.RosString())  # "text/html,application/xhtml+xml,application/xml;q=0.9,image/webp,*/*;q=0.8"  "text/html,application/xhtml+xml,application/xml;q=0.9,*/*;q=0.8"
    Accept_Encoding = pyros_schemas.RosOpt(pyros_schemas.RosString())  # "gzip, deflate, sdch"  "gzip, deflate"
    Accept_Language = pyros_schemas.RosOpt(pyros_schemas.RosString())  # "en-US,en;q=0.8"  "en-US,en;q=0.5"
    Cache_Control = pyros_schemas.RosOpt(pyros_schemas.RosString())  # "max-age=0"
    Cookie = pyros_schemas.RosOpt(pyros_schemas.RosString())  # "_ga=GA1.2.13884453.1450081587; _gat=1"
    Host = pyros_schemas.RosOpt(pyros_schemas.RosString())  # "httpbin.org"  "httpbin.org"
    Referer = pyros_schemas.RosOpt(pyros_schemas.RosString())  # "http://httpbin.org/"
    Upgrade_Insecure_Requests = pyros_schemas.RosOpt(pyros_schemas.RosString())  # "1"  "1"
    User_Agent = pyros_schemas.RosOpt(pyros_schemas.RosString())

    # massaging the response from request into a dict accepted by this schema
    @marshmallow.pre_load
    def key_mod(self, headers_data):
        # careful : headers key name are slightly different
        return {h.replace('-', '_'): v for h, v in headers_data.items()}


def httpbin_headers_callback(data):
    # define schemas

    @pyros_schemas.with_explicitly_matched_type(HttpbinHeaders._request_class)
    class HttpbinHeadersRequestSchema(marshmallow.Schema):
        headers = pyros_schemas.RosNested(HttpRequestHeadersSchema)

    @pyros_schemas.with_explicitly_matched_type(HttpbinHeaders._response_class)
    class HttpbinHeadersResponseSchema(marshmallow.Schema):
        status = pyros_schemas.RosUInt16()
        headers = pyros_schemas.RosNested(HttpRequestHeadersSchema)

        # massaging the response from request into a dict accepted by this schema
        @marshmallow.pre_load
        def parse_response(self, response_data):
            return {
                'status': response_data.status_code,
                # careful : headers here come from response data
                'headers': response_data.json().get('headers')
            }

    # extract data
    request_schema = HttpbinHeadersRequestSchema(strict=True)
    marshalled, errors = request_schema.dump(data)

    if not errors:
        # relaying request to matching service
        response = requests.get('http://httpbin.org/headers', headers=marshalled.get('headers'))
    else:
        rospy.logwarn("Errors when parsing request: {0}".format(data))
        rospy.logwarn("  {0}".format(errors))
        return HttpbinHeaders._response_class(status=HttpStatusCode.BAD_REQUEST)
    response_schema = HttpbinHeadersResponseSchema()
    unmarshalled, errors = response_schema.load(response)

    if not errors:
        ros_response = unmarshalled
    else:
        rospy.logwarn("Errors when parsing response: {0}".format(response))
        rospy.logwarn("  {0}".format(errors))
        ros_response = HttpbinHeaders._response_class(status=HttpStatusCode.BAD_GATEWAY)

    return ros_response


@pyros_schemas.with_explicitly_matched_type(HttpbinGetArgs)
class HttpbinGetArgsSchema(marshmallow.Schema):
    arg = pyros_schemas.RosString()  # test arg
    argopt = pyros_schemas.RosOpt(pyros_schemas.RosString())  # test argopt
    arglist = pyros_schemas.RosList(pyros_schemas.RosString())  # test arglist


def httpbin_get_callback(data):
    # define schemas
    @pyros_schemas.with_explicitly_matched_type(HttpbinGet._request_class)
    class HttpbinGetRequestSchema(marshmallow.Schema):
        params = pyros_schemas.RosNested(HttpbinGetArgsSchema)
        headers = pyros_schemas.RosOpt(pyros_schemas.RosNested(HttpRequestHeadersSchema))

    @pyros_schemas.with_explicitly_matched_type(HttpbinGet._response_class)
    class HttpbinGetResponseSchema(marshmallow.Schema):
        status = pyros_schemas.RosUInt16()
        args = pyros_schemas.RosNested(HttpbinGetArgsSchema)
        headers = pyros_schemas.RosNested(HttpRequestHeadersSchema)
        origin = pyros_schemas.RosString()
        url = pyros_schemas.RosString()

        # massaging the response from request into a dict accepted by this schema
        @marshmallow.pre_load
        def parse_response(self, response_data):
            return {
                'status': response_data.status_code,
                'args': response_data.json().get('args'),
                # careful : headers here come from response data
                'headers': response_data.json().get('headers'),
                'origin': response_data.json().get('origin'),
                'url': response_data.json().get('url'),
            }

    # extract data
    request_schema = HttpbinGetRequestSchema(strict=True)
    marshalled, errors = request_schema.dump(data)

    if not errors:
        # relaying request to matching service
        response = requests.get('http://httpbin.org/get', headers=marshalled.get('headers'), params=marshalled.get('params'))
    else:
        rospy.logwarn("Errors when parsing request: {0}".format(data))
        rospy.logwarn("  {0}".format(errors))
        return HttpbinGet._response_class(status=HttpStatusCode.BAD_REQUEST)
    response_schema = HttpbinGetResponseSchema()
    unmarshalled, errors = response_schema.load(response)

    if not errors:
        ros_response = unmarshalled
    else:
        rospy.logwarn("Errors when parsing response: {0}".format(response))
        rospy.logwarn("  {0}".format(errors))
        ros_response = HttpbinGet._response_class(status=HttpStatusCode.BAD_GATEWAY)

    return ros_response


@pyros_schemas.with_explicitly_matched_type(HttpbinPostArgs)
class HttpbinPostArgsSchema(marshmallow.Schema):
    arg = pyros_schemas.RosString()  # test arg
    argopt = pyros_schemas.RosOpt(pyros_schemas.RosString())  # test argopt
    arglist = pyros_schemas.RosList(pyros_schemas.RosString())  # test arglist


@pyros_schemas.with_explicitly_matched_type(HttpbinPostBody2)
class HttpbinPostBody2Schema(marshmallow.Schema):
    subteststring = pyros_schemas.RosString()  # test string
    subtestoptstring = pyros_schemas.RosOpt(pyros_schemas.RosString())  # test opt string
    subteststringarray = pyros_schemas.RosList(pyros_schemas.RosString())  # test list string

    subtestint = pyros_schemas.RosInt32()  # test int
    subtestoptint = pyros_schemas.RosOpt(pyros_schemas.RosInt32())  # test opt int
    subtestintarray = pyros_schemas.RosList(pyros_schemas.RosInt32())  # test list int

    subtestfloat= pyros_schemas.RosFloat32()  # test float
    subtestoptfloat = pyros_schemas.RosOpt(pyros_schemas.RosFloat32())  # test opt float
    subtestfloatarray = pyros_schemas.RosList(pyros_schemas.RosFloat32())  # test list float


@pyros_schemas.with_explicitly_matched_type(HttpbinPostBody)
class HttpbinPostBodySchema(marshmallow.Schema):
    testitem = pyros_schemas.RosNested(HttpbinPostBody2Schema)  # test string
    testoptitem = pyros_schemas.RosOpt(pyros_schemas.RosNested(HttpbinPostBody2Schema))  # test opt string
    testitemarray = pyros_schemas.RosList(pyros_schemas.RosNested(HttpbinPostBody2Schema))  # test list string


def httpbin_postjson_callback(data):
    # define schemas
    @pyros_schemas.with_explicitly_matched_type(HttpbinPostJson._request_class)
    class HttpbinPostRequestSchema(marshmallow.Schema):
        params = pyros_schemas.RosNested(HttpbinPostArgsSchema)
        headers = pyros_schemas.RosOpt(pyros_schemas.RosNested(HttpRequestHeadersSchema))
        json = pyros_schemas.RosNested(HttpbinPostBodySchema)

    @pyros_schemas.with_explicitly_matched_type(HttpbinPostJson._response_class)
    class HttpbinPostResponseSchema(marshmallow.Schema):
        status = pyros_schemas.RosUInt16()
        args = pyros_schemas.RosNested(HttpbinPostArgsSchema)
        data = pyros_schemas.RosString()
        headers = pyros_schemas.RosNested(HttpRequestHeadersSchema)
        json = pyros_schemas.RosNested(HttpbinPostBodySchema)
        origin = pyros_schemas.RosString()
        url = pyros_schemas.RosString()

        # massaging the response from request into a dict accepted by this schema
        @marshmallow.pre_load
        def parse_response(self, response):
            return {
                'status': response.status_code,
                'args': response.json().get('args'),
                'data': response.json().get('data'),
                'headers': response.json().get('headers'),
                'json': response.json().get('json'),
                'origin': response.json().get('origin'),
                'url': response.json().get('url'),
            }


    # extract data
    request_schema = HttpbinPostRequestSchema(strict=True)
    marshalled, errors = request_schema.dump(data)

    if not errors:
        # relaying request to matching service
        response = requests.post('http://httpbin.org/post', headers=marshalled.get('headers'), params=marshalled.get('params'), json=marshalled.get('json'))
    else:
        rospy.logwarn("Errors when parsing request: {0}".format(data))
        rospy.logwarn("  {0}".format(errors))
        return HttpbinIp._response_class(status=HttpStatusCode.BAD_REQUEST)
    response_schema = HttpbinPostResponseSchema()
    unmarshalled, errors = response_schema.load(response)

    if not errors:
        ros_response = unmarshalled
    else:
        rospy.logwarn("Errors when parsing response: {0}".format(response))
        rospy.logwarn("  {0}".format(errors))
        ros_response = HttpbinPost._response_class(status=HttpStatusCode.BAD_GATEWAY)

    return ros_response



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
