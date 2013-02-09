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
# Creation: 08/02/2013

INCLUDE_DIRECTORIES(${PROJECT_BINARY_DIR}/include/metapod/models/${CURRENT_MODEL}/
  ${PROJECT_SOURCE_DIR}/tests
  )

ADD_DEFINITIONS(" -DCURRENT_MODEL_ROBOT=${CURRENT_MODEL}")
STRING(TOUPPER "${CURRENT_MODEL}" CURRENT_MODEL_UPPER)
ADD_DEFINITIONS(" -DCURRENT_MODEL_IS_${CURRENT_MODEL_UPPER}=1")

MACRO(ADD_MODEL_TESTCASE name)
  # Generate the metapod files from URDF
  ADD_METAPOD_FROM_URDF_MODEL(${CURRENT_MODEL})
  # Create the related inclusion files
  CONFIGURE_FILE(include_model.hh.cmake ${PROJECT_BINARY_DIR}/include/metapod/models/${CURRENT_MODEL}/include_model.hh)
  # Create the new test target program 
  SET(new_name ${name}_${CURRENT_MODEL})      
  TARGET_LINK_LIBRARIES(${new_name})
  SET(new_name_sources ${name}.cc ${robot_name_sources})
  # Add it as a test case.
  ADD_TESTCASE(${new_name} ${new_name_sources})
ENDMACRO(ADD_MODEL_TESTCASE)

ADD_MODEL_TESTCASE(test_rnea )

