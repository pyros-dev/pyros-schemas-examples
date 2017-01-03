# -*- coding: utf-8 -*-

"""
rostful_requests.api
~~~~~~~~~~~~~~~~~~~~

This module implements the Rostful Requests API.
It is intentionally reproducing the requests library API

"""

try:

    import roslib
    import rospy
    import rosmsg
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
    import rosmsg
    import std_msgs.msg as std_msgs
    import pyros_schemas

    from pyros_rosclient.msg import HttpStatusCode, HttpRequestHeaders, HttpResponseHeaders, HttpbinGetArgs, HttpbinPostArgs, HttpbinPostBody, HttpbinPostBody2
    from pyros_rosclient.srv import HttpbinIp, HttpbinUserAgent, HttpbinHeaders, HttpbinGet, HttpbinPostJson, HttpbinPostForm, HttpbinPostFiles

import requests


class RostfulRequestServiceException(Exception):
    """Base class for rostful_requests service-related exceptions"""
    pass


class RostfulRequestServiceRequestException(RostfulRequestServiceException):
    """Base class for rostful_requests service-related exceptions"""
    pass


class RostfulRequestServiceResponseException(RostfulRequestServiceException):
    """Base class for rostful_requests service-related exceptions"""
    pass

# TODO : exceptions with meaningful web error code explanation...


def request(method, url, ros_data, service_class, **kwargs):
    """Constructs and sends a :class:`Request <Request>`.

    :param method: method for the new :class:`Request` object.
    :param url: URL for the new :class:`Request` object.
    :param service_type: The ROS service class
    :param
    For other param documentation, please refer to the requests library

    Usage::

      >>> import requests
      >>> req = requests.request('GET', 'http://httpbin.org/get')
      <Response [200]>
    """

    # dynamically define request schemas by service introspection
    req_schema_members = zip(service_class._request_class.__slots__, [pyros_schemas.ros_msgtype_mapping(t) for t in service_class._request_class._slot_types])
    RequestSchema = type('RequestSchema', (pyros_schemas.RosSchema,), req_schema_members)

    # Attempting to marshall the request data
    request_schema = RequestSchema(strict=True)
    marshalled, errors = request_schema.dump(ros_data)

    if errors:
        raise RostfulRequestServiceRequestException(
            "Unable to marshall request:\n" +
            "  %s\n\nsrv file:\n%s" % (errors, rosmsg.get_srv_text(service_class._type)))
    else:
        # relaying request to matching web service
        response = requests.request(method=method, url=url, data=marshalled, **kwargs)


    # dynamically define response schema by service introspection
    resp_schema_members = zip(service_class._response_class.__slots__,
                              [pyros_schemas.ros_msgtype_mapping(t) for t in service_class._response_class._slot_types])
    resp_schema_members['parse_response'] = pyros_schemas.pre_load(lambda data: {k: d for k, d in data.json()})
    resp_schema_members['make_ros_response'] = pyros_schemas.post_load(lambda resp_dict: service_class._response_class(resp_dict))
    ResponseSchema = type('ResponseSchema', (pyros_schemas.RosSchema,), resp_schema_members)

    # Attempting to marshall the response data
    response_schema = ResponseSchema()
    unmarshalled, errors = response_schema.load(response)

    if errors:
        raise RostfulRequestServiceResponseException(
            "Unable to marshall response. One of the fields has an incorrect type:\n" +
            "  %s\n\nsrv file:\n%s" % (errors, rosmsg.get_srv_text(service_class._type)))
    else:
        response = unmarshalled

    return response

def get(url, ros_data, params=None, **kwargs):
    """Sends a GET request.

    :param url: URL for the new :class:`Request` object.
    :param params: (optional) Dictionary or bytes to be sent in the query string for the :class:`Request`.
    :param \*\*kwargs: Optional arguments that ``request`` takes.
    :return: :class:`Response <Response>` object
    :rtype: requests.Response
    """

    kwargs.setdefault('allow_redirects', True)
    return request('get', url, params=params, **kwargs)


def options(url, **kwargs):
    """Sends a OPTIONS request.

    :param url: URL for the new :class:`Request` object.
    :param \*\*kwargs: Optional arguments that ``request`` takes.
    :return: :class:`Response <Response>` object
    :rtype: requests.Response
    """

    kwargs.setdefault('allow_redirects', True)
    return request('options', url, **kwargs)


def head(url, **kwargs):
    """Sends a HEAD request.

    :param url: URL for the new :class:`Request` object.
    :param \*\*kwargs: Optional arguments that ``request`` takes.
    :return: :class:`Response <Response>` object
    :rtype: requests.Response
    """

    kwargs.setdefault('allow_redirects', False)
    return request('head', url, **kwargs)


def post(url, data=None, json=None, **kwargs):
    """Sends a POST request.

    :param url: URL for the new :class:`Request` object.
    :param data: (optional) Dictionary, bytes, or file-like object to send in the body of the :class:`Request`.
    :param json: (optional) json data to send in the body of the :class:`Request`.
    :param \*\*kwargs: Optional arguments that ``request`` takes.
    :return: :class:`Response <Response>` object
    :rtype: requests.Response
    """

    return request('post', url, data=data, json=json, **kwargs)


def put(url, data=None, **kwargs):
    """Sends a PUT request.

    :param url: URL for the new :class:`Request` object.
    :param data: (optional) Dictionary, bytes, or file-like object to send in the body of the :class:`Request`.
    :param \*\*kwargs: Optional arguments that ``request`` takes.
    :return: :class:`Response <Response>` object
    :rtype: requests.Response
    """

    return request('put', url, data=data, **kwargs)


def patch(url, data=None, **kwargs):
    """Sends a PATCH request.

    :param url: URL for the new :class:`Request` object.
    :param data: (optional) Dictionary, bytes, or file-like object to send in the body of the :class:`Request`.
    :param \*\*kwargs: Optional arguments that ``request`` takes.
    :return: :class:`Response <Response>` object
    :rtype: requests.Response
    """

    return request('patch', url,  data=data, **kwargs)


def delete(url, **kwargs):
    """Sends a DELETE request.

    :param url: URL for the new :class:`Request` object.
    :param \*\*kwargs: Optional arguments that ``request`` takes.
    :return: :class:`Response <Response>` object
    :rtype: requests.Response
    """

    return request('delete', url, **kwargs)
