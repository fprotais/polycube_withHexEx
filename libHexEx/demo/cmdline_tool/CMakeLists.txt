#
# Copyright 2016 Computer Graphics Group, RWTH Aachen University
# Author: Hans-Christian Ebke <ebke@cs.rwth-aachen.de>
#         Max Lyon <lyon@cs.rwth-aachen.de
#
# This file is part of HexEx.
# 
# HexEx is free software: you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 3 of the License, or (at your
# option) any later version.
# 
# HexEx is distributed in the hope that it will be useful, but WITHOUT ANY
# WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
# for more details.
# 
# You should have received a copy of the GNU General Public License
# along with HexEx.  If not, see <http://www.gnu.org/licenses/>.
#

cmake_minimum_required(VERSION 3.4)

find_package (OpenVolumeMesh REQUIRED)

# disable OpenVolumeMesh debug output
add_definitions(-DNDEBUG)

set (CMDLINE_TOOL_SOURCES "")
set (CMDLINE_TOOL_INCLUDE_DIRS "")
set (CMDLINE_TOOL_LIBRARIES "")
set (CMDLINE_TOOL_LIBRARY_DIRS "")

list (APPEND CMDLINE_TOOL_SOURCES main.cc)
list (APPEND CMDLINE_TOOL_INCLUDE_DIRS
    ${OPENVOLUMEMESH_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src
    ${HEXEX_INCLUDE_DIRS}
)
list (APPEND CMDLINE_TOOL_LIBRARIES
    ${OPENVOLUMEMESH_LIBRARY}
    HexExStatic
)

list (APPEND CMDLINE_TOOL_LIBRARY_DIRS ${OPENVOLUMEMESH_LIBRARY_DIR})

add_executable (cmdline_tool ${CMDLINE_TOOL_SOURCES})

LINK_DIRECTORIES(cmdline_tool ${CMDLINE_TOOL_LIBRARY_DIRS} )
target_link_libraries(cmdline_tool ${CMDLINE_TOOL_LIBRARIES})

if ( NOT WIN32 )
  # Set compiler flags
  if (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set_target_properties(cmdline_tool PROPERTIES COMPILE_FLAGS "-Wno-long-long")
  else()
    set_target_properties(cmdline_tool PROPERTIES COMPILE_FLAGS "-pedantic -Wno-long-long")
  endif()
endif()

set_target_properties(cmdline_tool
    PROPERTIES
    INCLUDE_DIRECTORIES "${CMDLINE_TOOL_INCLUDE_DIRS}"
    CXX_STANDARD 11
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)
