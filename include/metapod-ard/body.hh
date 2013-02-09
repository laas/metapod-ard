// Copyright 2013,
//
// Olivier Stasse, 2013
//
// This file is part of metapod-ard.
// metapod-ard is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// metapod-ard is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// metapod-ard.  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef METAPOD_ARD_BODY_HH_
# define METAPOD_ARD_BODY_HH_
# include <metapod-ard/fwd.hh>

#include "boost/shared_ptr.hpp"
#include "abstract-robot-dynamics/body.hh"
#include "metapod/tools/common.hh"

/*
  Forward declaration
*/
class CjrlJoint;

namespace metapod {
namespace ard {

/** \ingroup userclasses
    \brief This class implements a body
    
    See CjrlJoint for documentation.
*/
template <class Node>
class METAPOD_ARD_EXPORT Body : virtual public CjrlBody
{
private:
  Node & node_;
      
public:
  
  /**
     \name Constructor and destructor
  */
  ~DynamicBody() {};
  
  DynamicBody(const Node& inNode);
  
  /**
     @}
  */
  /**
     \brief Get position of center of mass in joint local reference frame.
  */
  const vector3d& localCenterOfMass() const
  {
    return node_.I.h();
  }

  /**
     \brief Set postion of center of mass in joint reference frame.
  */
  void localCenterOfMass(const vector3d& inlocalCenterOfMass)
  {
    node_.I.h()=inlocalCenterOfMass;
  }
  
  /**
     \brief Get Intertia matrix expressed in joint local reference frame.
  */
  const matrix3d& inertiaMatrix() const
  {
    return node_.I.I().toMatrix();
  }
  
  /**
     \brief Set inertia matrix.
  */
  void inertiaMatrix(const matrix3d& inInertiaMatrix)
  {
    node_.I.I(inInertiaMatrix);
  }
    
  /**
     \brief Get mass.
  */
  double mass() const
  {
    return node_::mass;
  }
  
  /**
     \brief Set mass.
  */
  void mass(double inMass)
  {
    node_.I.m(inMass);
  }
  
  /**
     \brief Get const pointer to the joint the body is attached to.
  */
  const CjrlJoint* joint() const
  { return 0;}
  
};

} // end of namespace ard
} // end of namespace metapod


#endif /* METAPOD_ARD_BODY_HH_*/
