# Copyright 2013
#
# Olivier Stasse (JRL/LAAS, CNRS/AIST)
#
# This file is part of metapod-ard.
# metapod-ard is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# metapod-ard is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Lesser Public License for more details.
# You should have received a copy of the GNU Lesser General Public License
# along with metapod-ard.  If not, see <http://www.gnu.org/licenses/>.
#
# Creation: 07/02/2013


# Add Boost path to include directories.
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/include/)
INCLUDE_DIRECTORIES(${METAPOD_INCLUDE_DIRS})

MESSAGE(STATUS "${CMAKE_SOURCE_DIR}/include/")
# Make Boost.Test generate the main function in test cases.
ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN)

# ADD_TESTCASE(NAME SOURCE_FILE1 [SOURCE_FILE2])
# ------------------------
#
# Define a test named `NAME'.
#
# This macro will
#
# * create a binary from user-provided source files and common.hh
# * link it against Boost
# * add it to the test suite.
#
# The test should #include the common.hh header
#

MACRO(ADD_TESTCASE NAME)
  ADD_EXECUTABLE(${NAME} ${ARGN})
  #ADD_TEST(${NAME} ${RUNTIME_OUTPUT_DIRECTORY}/${NAME})
  TARGET_LINK_LIBRARIES(${NAME} ${Boost_LIBRARIES})
ENDMACRO(ADD_TESTCASE)

SET(CURRENT_MODEL "simple_humanoid")
ADD_SUBDIRECTORY(use_model use_simple_humanoid)

SET(CURRENT_MODEL "simple_arm")
ADD_SUBDIRECTORY(use_model use_simple_arm)

