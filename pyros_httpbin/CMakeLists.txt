##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(pyros_httpbin)

##############################################################################
# Find Packages
##############################################################################

find_package(catkin REQUIRED COMPONENTS
    catkin_pip
    roslint
    pyros_msgs
    std_msgs
    message_generation
    message_runtime
)


##############################################################################
# Python Configuration
##############################################################################


#####
# SERVICES
#####

add_message_files(
  DIRECTORY
    msg
  FILES
    HttpbinGetArgs.msg
    HttpbinPostArgs.msg
    HttpbinPostBody.msg
    HttpbinPostBody2.msg
    HttpFormItem.msg
    HttpParam.msg
    HttpStatusCode.msg
    HttpRequestHeaders.msg
    HttpResponseHeaders.msg
)

add_service_files(
  DIRECTORY
    srv
  FILES
    HttpbinIp.srv
    HttpbinUserAgent.srv
    HttpbinHeaders.srv
    HttpbinGet.srv
    HttpbinPostFiles.srv
    HttpbinPostForm.srv
    HttpbinPostJson.srv
)

generate_messages(DEPENDENCIES pyros_msgs std_msgs)


##############################################################################
# Catkin
##############################################################################

catkin_pip_package(pyros_httpbin)

#########
# Tests
#######

if (CATKIN_ENABLE_TESTING)
  catkin_add_pytests(tests/test_httpbin.py)
  # Removing. somehow rostest cannot run the httpbin node. Rostest is too buggy we should drop it.
  #find_package(rostest REQUIRED)
  #add_rostest(tests/test_httpbin.test)
endif()

#####
# Install
#####

install(
    PROGRAMS
        nodes/httpbin.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
