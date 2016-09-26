##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(pyros_rosclient)

##############################################################################
# Find Packages
##############################################################################

find_package(catkin REQUIRED COMPONENTS
    roslint
    pyros_msgs
    std_msgs
    message_generation
    message_runtime
)


##############################################################################
# Python Configuration
##############################################################################

catkin_python_setup()


# Lint Python modules
file(GLOB_RECURSE ${PROJECT_NAME}_PY_SRCS
     RELATIVE ${PROJECT_SOURCE_DIR} pyros_rosclient/*.py)
roslint_python(${${PROJECT_NAME}_PY_SRCS})


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

catkin_package()

#########
# Tests
#######

if (CATKIN_ENABLE_TESTING)
  #catkin_add_nosetests(tests)
  find_package(rostest REQUIRED)
  add_rostest(tests/test_httpbin.test)
endif()

#####
# Install
#####

install(
    PROGRAMS
        nodes/httpbin_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
