# -*- coding: utf-8 -*-

#   __
#  /__)  _  _     _   _ _/   _
# / (   (- (/ (/ (- _)  /  _)
#          /

"""
Requests HTTP library for ROS
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Rostful Requests is an HTTP library, written in Python, for ROS robots.
The intent here is to wrap the requests library in a thin layer and make it usable intuitively with ROS for robotics developers.
"""


from .api import with_service_schemas