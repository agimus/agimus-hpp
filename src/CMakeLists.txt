# Copyright (c) 2018, 2019, 2020 CNRS and Airbus S.A.S
# Author: Joseph Mirabel
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:

# 1. Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above
# copyright notice, this list of conditions and the following
# disclaimer in the documentation and/or other materials provided
# with the distribution.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
# INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
# OF THE POSSIBILITY OF SUCH DAMAGE.
SET(LIBRARY_NAME ${PROJECT_NAME})

INCLUDE(${HPP_CORBASERVER_CMAKE_PLUGIN})
INCLUDE(${HPP_CORBASERVER_OMNIIDL_CMAKE})
OMNIIDL_INCLUDE_DIRECTORIES(
  ${CMAKE_SOURCE_DIR}/idl ${HPP_CORBASERVER_DATAROOTDIR}/idl
  )
INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR}/src)

MAKE_DIRECTORY(${CMAKE_BINARY_DIR}/src/hpp/agimus_idl)
FOREACH(IDL server discretization point-cloud)
  GENERATE_IDL_CPP (hpp/agimus_idl/${IDL} ${CMAKE_SOURCE_DIR}/idl/hpp/agimus_idl
    HEADER_SUFFIX -idl.hh)
  GENERATE_IDL_PYTHON (${IDL} ${CMAKE_SOURCE_DIR}/idl/hpp/agimus_idl
    ENABLE_DOCSTRING
    STUBS hpp_stubs.agimus
    ARGUMENTS
    -Wbinline
    -Wbmodules=hpp_idl
    -Wbextern=common:hpp_stubs
    -Wbextern=constraints:hpp_stubs.constraints
    -Wbextern=robots:hpp_stubs.pinocchio
    -Wbextern=paths:hpp_stubs.core
    -Wbextern=_constraints:hpp_stubs.core
    )
ENDFOREACH()

GENERATE_IDL_CPP_IMPL (hpp/agimus_idl/discretization ${CMAKE_SOURCE_DIR}/idl/hpp/agimus_idl
  ARGUMENTS
  -Wbguard_prefix=hpp_agimus_idl
  -Wbinc_prefix=hpp/agimus_idl
  HH_SUFFIX -idl.hh)

GENERATE_IDL_CPP_IMPL (hpp/agimus_idl/point-cloud ${CMAKE_SOURCE_DIR}/idl/hpp/agimus_idl
  ARGUMENTS
  -Wbguard_prefix=hpp_agimus_idl
  -Wbinc_prefix=hpp/agimus_idl
  HH_SUFFIX -idl.hh)

INSTALL(DIRECTORY ${CMAKE_SOURCE_DIR}/idl/hpp/agimus_idl
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/idl/hpp)

INSTALL(
  DIRECTORY
  ${CMAKE_CURRENT_BINARY_DIR}/hpp_idl/hpp/agimus_idl
  DESTINATION ${PYTHON_SITELIB}/agimus_idl/hpp/)

INSTALL(
  FILES
  ${CMAKE_CURRENT_BINARY_DIR}/hpp_idl/__init__.py
  DESTINATION ${PYTHON_SITELIB}/agimus_idl)

INSTALL(
  FILES
  ${CMAKE_CURRENT_BINARY_DIR}/hpp_idl/hpp/__init__.py
  DESTINATION ${PYTHON_SITELIB}/agimus_idl/hpp)

INSTALL(
  DIRECTORY
  ${CMAKE_CURRENT_BINARY_DIR}/hpp_stubs/agimus
  DESTINATION ${PYTHON_SITELIB}/agimus_stubs)

INSTALL(
  FILES
  ${CMAKE_CURRENT_BINARY_DIR}/hpp_stubs/__init__.py
  DESTINATION ${PYTHON_SITELIB}/agimus_stubs)

ADD_CUSTOM_TARGET(generate_idl_cpp DEPENDS ${ALL_IDL_CPP_STUBS} ${ALL_IDL_CPP_IMPL_STUBS})
ADD_CUSTOM_TARGET(generate_idl_python DEPENDS ${ALL_IDL_PYTHON_STUBS})

IF(BUILD_HPP_PLUGIN)
  SET(AGIMUS_HPP_PLUGIN_SOURCES
    server.cc
    discretization.cc
    point-cloud.cc
    ${ALL_IDL_CPP_STUBS}
    ${ALL_IDL_CPP_IMPL_STUBS}
    )

  IF(CLIENT_TO_GEPETTO_VIEWER)
    ADD_DEFINITIONS(-DCLIENT_TO_GEPETTO_VIEWER)
    HPP_ADD_SERVER_PLUGIN(agimus-hpp
      SOURCES ${AGIMUS_HPP_PLUGIN_SOURCES}
      LINK_DEPENDENCIES PUBLIC hpp-corbaserver::hpp-corbaserver
      hpp-manipulation::hpp-manipulation
      gepetto-viewer-corba::gepetto-viewer-corba
      PKG_CONFIG_DEPENDENCIES omniORB4 roscpp dynamic_graph_bridge_msgs)
  ELSE()
    HPP_ADD_SERVER_PLUGIN(agimus-hpp
      SOURCES ${AGIMUS_HPP_PLUGIN_SOURCES}
      LINK_DEPENDENCIES PUBLIC hpp-corbaserver::hpp-corbaserver
      hpp-manipulation::hpp-manipulation
      PKG_CONFIG_DEPENDENCIES omniORB4 roscpp dynamic_graph_bridge_msgs)
  ENDIF()
  ADD_DEPENDENCIES (agimus-hpp generate_idl_cpp generate_idl_python)
ELSE(BUILD_HPP_PLUGIN)
  ADD_CUSTOM_TARGET(generate_idl ALL)
  # No need to generate C++ bindings
  ADD_DEPENDENCIES (generate_idl generate_idl_python)
ENDIF(BUILD_HPP_PLUGIN)

# Install Python files

IF(BUILD_ROS_INTERFACE)
  SET (PYTHON_FILE
      estimation.py
      trajectory_publisher.py
      planning_request_adapter.py
      client.py
      ros_tools.py
      tools.py
      __init__.py)

  FOREACH(F ${PYTHON_FILE})
    PYTHON_INSTALL_ON_SITE("agimus_hpp" ${F})
  ENDFOREACH()

  SET (PYTHON_FILE
      planning_request_adapter.py
      __init__.py)

  FOREACH(F ${PYTHON_FILE})
    PYTHON_INSTALL_ON_SITE("agimus_hpp/manipulation" ${F})
  ENDFOREACH()
ENDIF(BUILD_ROS_INTERFACE)

SET (PYTHON_FILE
    client.py
    __init__.py)
FOREACH(F ${PYTHON_FILE})
  PYTHON_INSTALL_ON_SITE("agimus_hpp/plugin" ${F})
ENDFOREACH()
