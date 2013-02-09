/*
 * Copyright 2013
 *
 * Olivier Stasse
 *
 * LAAS, CNRS
 *
 * This file is part of metapod-ard.
 * metapod-ard is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * metapod-ard is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Lesser Public License for more details.
 * You should have received a copy of the GNU Lesser General Public License
 * along with metapod-ard.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef METAPOD_ARD_MATH_SUPPORT_HH_
#define METAPOD_ARD_MATH_SUPPORT_HH_

#include <metapod/tools/common.hh>

typedef metapod::Vector1d vector1d;
typedef metapod::Vector3d vector3d;
typedef metapod::Vector6d vector6d;

typedef metapod::Matrix3d matrix3d;
typedef Eigen::Matrix< metapod::FloatType, 4, 4 > matrix4d;
typedef Eigen::MatrixXd matrixNxP;
typedef Eigen::VectorXd vectorN;
#endif /* METAPOD_ARD_MATH_SUPPORT_HH_ */
