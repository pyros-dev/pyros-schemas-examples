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
import os
import psutil
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

class StatusCodeException(Exception):
    pass

from pyros_schemas.ros.schemagic import create

import six
from pyros_schemas.ros.types_mapping import ros_msgtype_mapping
from pyros_schemas.ros.utils import _get_msg_class, _get_rosmsg_fields_as_dict, _get_rosmsg_members_as_dict
from pyros_schemas.ros.schema import RosSchema, pre_load, post_load, pre_dump, post_dump
from pyros_schemas.ros.basic_fields import (
    RosNested,
    RosList,
)
from pyros_schemas.ros.optional_fields import (
    RosOpt,
)

import marshmallow

# Statically proxying marshmallow useful decorators for methods
pre_load = marshmallow.pre_load
post_load = marshmallow.post_load
pre_dump = marshmallow.pre_dump
post_dump = marshmallow.post_dump

from pyros_schemas.ros.exceptions import PyrosSchemasValidationError

class RosSchema(marshmallow.Schema):
    """Inheriting the Marshmallow schema to extend behavior introspecting into slots for ROS messages
    Not using pre_load, post_load, pre_dump or post_dump here, to simplify things for when we need to create schemas dynamically.
    pre_load, post_load, pre_dump, post_dump should still be used in derived Schemas, to customize the serialization
    This class only factor serialization behavior required by ROS generated message types.
    """

    _valid_ros_msgtype = None  # fill this in your Schema class for enforcing msgtype validation on load
    _generated_ros_msgtype = None  # fill this in your Schema class for automatically generating msgtype on dump

    def __init__(self, strict=True, **kwargs):  # default to strict behavior
        super(RosSchema, self).__init__(strict=strict, **kwargs)

    def load(self, data, many=None, partial=None):
        """Overloading load function to transform a ROS msg type into a dict for marshmallow"""
        # early type validation if required
        if self.strict and self._valid_ros_msgtype and not isinstance(data, self._valid_ros_msgtype):
            raise PyrosSchemasValidationError('data type should be {0}'.format(self._valid_ros_msgtype))
        data_dict = _get_rosmsg_members_as_dict(data)
        try:
            unmarshal_result = super(RosSchema, self).load(data_dict, many=many, partial=partial)
        except marshmallow.ValidationError as ve:
            raise PyrosSchemasValidationError('ERROR occurred during deserialization: {ve}'.format(**locals()))
        return unmarshal_result

    def dump(self, obj, many=None, update_fields=True, **kwargs):
        """Overloading dump function to transform a dict into a ROS msg from marshmallow"""
        try:
            obj_dict = _get_rosmsg_members_as_dict(obj)  # in case we get something that is not a dict...
            # because ROS field naming conventions are different than python dict key conventions
            obj_rosfixed_dict = {k.replace('-', '_'): v for k, v in obj_dict.items()}  # TODO : come up with a generic <ROS_field encode> function
            data_dict, errors = super(RosSchema, self).dump(obj_rosfixed_dict, many=many, update_fields=update_fields, **kwargs)
        except marshmallow.ValidationError as ve:
            raise PyrosSchemasValidationError('ERROR occurred during serialization: {ve}'.format(**locals()))
        if self._generated_ros_msgtype and not errors:
            obj = self._generated_ros_msgtype(**data_dict)
        else:
            obj = data_dict  # we return directly
        return marshmallow.MarshalResult(obj, errors)


# TODO : find cleaner way, maybe a RosMagikSchema class like thing...
def create(ros_msg_class,
                pre_load_fun=None,
                post_load_fun=None,
                pre_dump_fun=None,
                post_dump_fun=None,
                **kwargs):
    """
    Factory method that creates a Schema class for this ROS message type by introspecting the ros_msg_class, and then instanciate it.
    :param ros_msg_class: the message class for which we need serialization. It can be the string specifying the message type or the type itself
    :param pre_load_fun: a callable that will be run before load(). It should be of the form : schema, data -> data
    :param post_load_fun: a callable that will be run after load(). It should be of the form : schema, data -> data
    Note that type validation is already implemented internally. check with_explicitly_matched_type decorator for more details
    :param pre_dump_fun: a callable that will be run before dump(). It should be of the form : schema, data -> data
    Note that type validation is already implemented internally. check with_explicitly_matched_type decorator for more details
    :param post_dump_fun: a callable that will be run after dump(). It should be of the form : schema, data -> data
    :param kwargs: any keyword argument will be added to the schema class.
    :return: A Schema that handles all (dict --load()--> ros_msg_class --dump()--> dict) serialization
    """

    if isinstance(ros_msg_class, six.string_types):  # if we get a string it s a ros description, not the class itself
        ros_msg_class = _get_msg_class(ros_msg_class)
        # and keep going

    members_types = _get_rosmsg_fields_as_dict(ros_msg_class)
    members = {}
    schema_instance = RosSchema()
    for s, stype in members_types.iteritems():
        # Note here we rely entirely on _opt_slots from the class to be set properly
        # for both Nested or List representation of optional fields
        ros_schema_inst = None
        if stype.endswith("[]"):
            if stype[:-2] in ros_msgtype_mapping:
                # ENDING RECURSION with well known array type
                if hasattr(ros_msg_class, '_opt_slots') and s in ros_msg_class._opt_slots:
                    ros_schema_inst = RosOpt(ros_msgtype_mapping[stype[:-2]]())
                else:
                    ros_schema_inst = RosList(ros_msgtype_mapping[stype[:-2]]())
            else:
                # RECURSING in Nested fields
                if hasattr(ros_msg_class, '_opt_slots') and s in ros_msg_class._opt_slots:
                    ros_schema_inst = RosOpt(RosNested(create(stype[:-2])))  # we need to nest the next (Ros)Schema
                else:
                    ros_schema_inst = RosList(RosNested(create(stype[:-2])))  # we need to nest the next (Ros)Schema
        else:
            if stype in ros_msgtype_mapping:
                # ENDING RECURSION with well known basic type
                ros_schema_inst = ros_msgtype_mapping[stype]()  # TODO : shouldn't we check for opt slots here ?
            else:
                # RECURSING in Nested fields
                if hasattr(ros_msg_class, '_opt_slots') and s in ros_msg_class._opt_slots:
                    ros_schema_inst = RosOpt(create(stype))
                else:
                    ros_schema_inst = RosNested(create(stype))  # we need to nest the next (Ros)Schema

        members.setdefault(s, ros_schema_inst)
        schema_instance.declared_fields[s] = ros_schema_inst
        schema_instance.fields[s] = ros_schema_inst
    # supporting extra customization of the serialization
    if pre_load_fun:
        schema_instance.declared_fields['_helper_pre_load'] = pre_load(pre_load_fun)
    if post_load_fun:
        schema_instance.declared_fields['_helper_post_load'] = post_load(post_load_fun)
    if pre_dump_fun:
        schema_instance.declared_fields['_helper_pre_dump'] = pre_dump(pre_dump_fun)
    if post_dump_fun:
        schema_instance.declared_fields['_helper_post_dump'] = post_dump(post_dump_fun)

    # adding extra members if needed
    for k, v in kwargs:
        schema_instance.declared_fields[k] = v
        #members[k] = v

    #members['_valid_ros_msgtype'] = ros_msg_class
    #members['_generated_ros_msgtype'] = ros_msg_class

    schema_instance._valid_ros_msgtype = ros_msg_class
    schema_instance._generated_ros_msgtype = ros_msg_class
    #rosobj.__class__.__name__ = ros_msg_class.__name__ + 'Schema'


    # MsgSchema = type(ros_msg_class.__name__ + 'Schema', (RosSchema,), members)
    # schema_instance = MsgSchema()

    #print('2', rosobj.__dict__)

    #return schema_instance
    return schema_instance

from pyros_schemas.ros.exceptions import PyrosSchemasServiceRequestException, PyrosSchemasServiceResponseException

from functools import wraps
def with_service_schemas(service_class):
    def with_service_schemas_decorator(func):
        @wraps(func)
        # TODO : handle funcitons AND methods ?
        def func_wrapper(*data):  # we need to expose only one argument for ROS or two for methods

            try:
                request_schema = create(service_class._request_class)
                data_dict, errors = request_schema.load(data[-1])  # we assume the last argument always contains the ROS data
                # print('......', data[-1], data_dict, errors)
            except Exception as e:
                raise PyrosSchemasServiceRequestException(e)

            # we should call the function with original and parsed argument,
            # including potential errors, just in case, at least temporarily...
            data_extended = data + (data_dict, errors)
            response = func(*data_extended)
            #  we also let the function trigger its own exceptions

            try:
                response_schema = create(service_class._response_class)
                response_ros, errors = response_schema.dump(response)
                return response_ros
            except Exception as e:
                raise PyrosSchemasServiceResponseException(e)

        return func_wrapper
    return with_service_schemas_decorator

class TestPyrosSchemas(object):

    def __init__(self):
        rospy.Service('/test/pyros_schemas', HttpbinPostJson, self.test_pyros_schemas2)
        rospy.wait_for_service('/test/pyros_schemas', timeout=10)
        self.test_service_proxy = rospy.ServiceProxy('/test/pyros_schemas', HttpbinPostJson)

    @with_service_schemas(HttpbinPostJson)
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
        pid = os.getpid()
        process = psutil.Process(pid)
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
            mem = process.get_memory_info()[0]
            rospy.loginfo("Memory usage(%s):%d" % (pid, mem))
            rospy.sleep(0.01)

if __name__ == '__main__':
    rospy.init_node('test_pyros_schemas', log_level=rospy.INFO)
    _node = TestPyrosSchemas()
    _node.spin()
