# Copyright 2013
#
# Olivier Stasse, LAAS/CNRS.
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
# Creation: 06/02/2013

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/cpack.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/metapodfromurdf.cmake)

SET(PROJECT_NAME metapod-ard)
SET(PROJECT_DESCRIPTION "Implementation of a robot dynamics library")
SET(PROJECT_URL "https://github.com/laas/metapod-ard")

SET(CXX_DISABLE_WERROR True)
CMAKE_POLICY(SET CMP0012 NEW)

IF(UNIX OR MINGW)
  ADD_DEFINITIONS("-fvisibility=hidden")
  ADD_DEFINITIONS("-Wno-variadic-macros")
ENDIF()


SETUP_PROJECT()

# Search for Eigen.
ADD_REQUIRED_DEPENDENCY("eigen3 >= 3.0.0")

# Search for Abstract Robot Dynamics
ADD_REQUIRED_DEPENDENCY("abstract-robot-dynamics >= 1.16")

# Search for metapod.
ADD_REQUIRED_DEPENDENCY("metapod >= 1.0.6")

# Search for Boost.
# Boost.Test is used by the test suite.
SET(BOOST_COMPONENTS
  unit_test_framework)
SEARCH_FOR_BOOST()

MESSAGE(STATUS "METAPOD_INCLUDE_DIRS: ${METAPOD_INCLUDE_DIRS}")
# Eigen (at least version 3.0.5) makes gcc report conversion warnings
# when one assigns to a block. This floods the compilation results.
# So we include Eigen headers as if they were system headers: the compiler
# won't report warning coming from them.
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIRS} 
  ${ABSTRACT_ROBOT_DYNAMICS_INCLUDE_DIRS}
  ${METAPOD_INCLUDE_DIRS})

# Set header files.
SET(${PROJECT_NAME}_ROOT_HEADERS
  ${CMAKE_CURRENT_BINARY_DIR}/include/metapod/config.hh
  )
SET(${PROJECT_NAME}_ALGOS_HEADERS
  include/${PROJECT_NAME}/algos/rnea.hh
  include/${PROJECT_NAME}/algos/crba.hh
  include/${PROJECT_NAME}/algos/jac_point_relative.hh
  include/${PROJECT_NAME}/algos/jac_point_chain.hh
  )

INCLUDE_DIRECTORIES("include" "${CMAKE_CURRENT_BINARY_DIR}/include")

ADD_SUBDIRECTORY(tests)

SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()
