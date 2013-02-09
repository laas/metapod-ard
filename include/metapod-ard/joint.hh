/*
 * Copyright 2013,
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

#ifndef METAPOD_ARD_JOINT_HH_
#define METAPOD_ARD_JOINT_HH_

#include "abstract-robot-dynamics/joint.hh"
#include "metapod/tools/joint.hh"

#define std::cout << __FILE__ <<< std::endl; \
    std::cout << "Not implemented yet" << std::endl;

/*
  Forward declaration
*/
class CjrlBody;

namespace metapod {
namespace ard{

/** \ingroup userclasses
    \brief This class represents a robot joint.

    Implements abstract interface CjrlJoint.
*/
template<class Node>
class METAPOD_ARD_DLLEXPORT Joint : public virtual CjrlJoint
{
private:
  Node & node_;
  
public:

  /**
     \name Constructor and destructor
  */
  virtual ~Joint() {};

  Joint(const Node & aNode)
  { node_ = aNode;}

protected:
  Joint();

public:

  /// \name Joint name
  /// \{
    
  /// \brief Get joint name
  const std::string& getName() const
  {
    return node_.joint_name;
  }
  
  
  /// \brief Set joint name
  virtual void setName(const std::string & name)
  {
    node_.joint_name = name;
  }

  /// \}
  /**
     @}
  */
  /**
     \name Joint hierarchy
     @{
  */

  /**
     \brief Get a pointer to the parent joint (if any).
  */
  CjrlJoint* parentJoint() const
  {
    return static_cast<CjrlJoint *>(0);
  }

  /**
     \brief Add a child joint.
  */
  bool addChildJoint (CjrlJoint& inJoint)
  {
  }

  /**
     \brief Get the number of children.
  */
  unsigned int countChildJoints() const
  {
    return
        ( Node::child0_id ==-2 ? 0 :1 ) +
        ( Node::child1_id ==-2 ? 0 :1 ) +
        ( Node::child2_id ==-2 ? 0 :1 ) +
        ( Node::child3_id ==-2 ? 0 :1 ) +
        ( Node::child4_id ==-2 ? 0 :1 );
  }

  /**
     \brief  	Returns the child joint at the given rank.
  */
  CjrlJoint* childJoint(unsigned int inJointRank) const
  {
    return 0;
  }

  /**
     \brief Get a vector containing references of the joints between the
     rootJoint and this joint. The root Joint and this Joint are included in the vector.
  */
  std::vector<CjrlJoint*> jointsFromRootToThis() const
  {
    std::vector<CjrlJoint *> r;
    r.clear();
    return r;
  }

  /**
     \brief Get the rank of this joint in the robot configuration vector.
     If the Joint has several degrees of freedom, it is the rank of the first degree of freedom.
  */
  unsigned int rankInConfiguration() const
  {
    return Node::q_idx;
  }

  /**
     @}
  */

  /**
     \name Joint kinematics
     @{
  */

  /**
     \brief Get the initial position of the joint.

     The initial position of the joint is the position of the local frame of
     the joint.
  */
  const matrix4d& initialPosition() const
  {
    return node_.joint.sXp;
  }

  /**
     \brief Update this joint's transformation according to degree of freedom value from argument robot configuration. This does not update the transformations of child joints.
     \param inDofVector is a robot configuration vector.
     \return false if argument vector's size is not equal to the robot's number of degrees of freedom
  */
  bool updateTransformation(const vectorN& inDofVector)
  {
    node_.joint.bcalc(inDofVector);
  }

  /**
     \brief Get the current transformation of the joint.

     The current transformation of the joint is the transformation
     moving the joint from the position in initial configuration to
     the current position.

     The current transformation is determined by the configuration
     \f${\bf q}\f$ of the robot.
  */
  const matrix4d &currentTransformation() const;
  {
    Spatial::Transform aXj;
    aXj =  node_.joint.Xj;
    matrix4d r;
    r.block<3,3>(0,0) = aXj.E();
    r.block<3,1>(3,0) = aXj.r();
    return r;
  }

  /**
     \brief Get the velocity \f$({\bf v}, {\bf \omega})\f$ of the joint.

     The velocity is determined by the configuration of the robot and its time derivative: \f$({\bf q},{\bf \dot{q}})\f$.

     \return the linear velocity \f${\bf v}\f$ of the origin of the joint frame
     and the angular velocity \f${\bf \omega}\f$ of the joint frame.
  */
  CjrlRigidVelocity jointVelocity() const
  {
    CjrlRigidVelocty arv(node_.body.vi.v(),
                         node_.body.vi.w());
    return arv;
  }

  /**
     \brief Get the acceleration of the joint.

     The acceleratoin is determined by the configuration of the robot and its first and second time derivative: \f$({\bf q},{\bf \dot{q}}, {\bf \ddot{q}})\f$.
  */
  CjrlRigidAcceleration jointAcceleration() const
  {
    CjrlRigidAcceleration ara(node_.body.ai.v(),
                              node_.body.ai.w());
  }

  /**
     \brief Get the number of degrees of freedom of the joint.
  */
  unsigned int numberDof() const
  {
    return Node::Joint::NBDOF;
  }

  /**
     @}
  */

  /**
     \name Bounds of the degrees of freedom
     @{
  */
  /**
     \brief Get the lower bound of a given degree of freedom of the joint.

     \param inDofRank Id of the dof in the joint
  */
  double lowerBound(unsigned int inDofRank) const
  { return 0;}

  /**
     \brief Get the upper bound of a given degree of freedom of the joint.

     \param inDofRank Id of the dof in the joint
  */
  double upperBound(unsigned int inDofRank) const
  { return 0;}

  /**
     \brief Set the lower bound of a given degree of freedom of the joint.

     \param inDofRank Id of the dof in the joint
     \param inLowerBound lower bound
  */
  void lowerBound(unsigned int inDofRank, double inLowerBound)
  {
  }

  /**
     \brief Set the upper bound of a given degree of freedom of the joint.

     \param inDofRank Id of the dof in the joint
     \param inUpperBound Upper bound.
  */
  void upperBound(unsigned int inDofRank, double inUpperBound)
  { }

  /**
     \brief Get the lower velocity bound of a given degree of freedom of the joint.

     \param inDofRank Id of the dof in the joint
  */
  double lowerVelocityBound(unsigned int inDofRank) const
  {}

  /**
     \brief Get the upper veocity bound of a given degree of freedom of the joint.

     \param inDofRank Id of the dof in the joint
  */
  double upperVelocityBound(unsigned int inDofRank) const
  {}

  /**
     \brief Set the lower velocity bound of a given degree of freedom of the joint.

     \param inDofRank Id of the dof in the joint
     \param inLowerBound lower bound
  */
  void lowerVelocityBound(unsigned int inDofRank, double inLowerBound)
  {}

  /**
     \brief Set the upper velocity bound of a given degree of freedom of the joint.

     \param inDofRank Id of the dof in the joint
     \param inUpperBound Upper bound.
  */
  void upperVelocityBound(unsigned int inDofRank, double inUpperBound)
  {}

  /**
     \brief Get the lower torque bound of a given degree of freedom of the joint.

     \param inDofRank Id of the dof in the joint
  */
  double lowerTorqueBound(unsigned int inDofRank) const
  {}

  /**
     \brief Get the upper veocity bound of a given degree of freedom of the joint.

     \param inDofRank Id of the dof in the joint
  */
  double upperTorqueBound(unsigned int inDofRank) const
  {}

  /**
     \brief Set the lower torque bound of a given degree of freedom of the joint.

     \param inDofRank Id of the dof in the joint
     \param inLowerBound lower bound
  */
  void lowerTorqueBound(unsigned int inDofRank, double inLowerBound)
  {}
  /**
     \brief Set the upper torque bound of a given degree of freedom of the joint.

     \param inDofRank Id of the dof in the joint
     \param inUpperBound Upper bound.
  */
  void upperTorqueBound(unsigned int inDofRank, double inUpperBound)
  {}

  /**
     @}
  */

  /**
     \name Jacobian functions wrt configuration.
     @{
  */

  /**
     \brief Get the Jacobian matrix of the joint position and orientation wrt the robot configuration.
     Kinematical constraints from interaction with the environment are not taken into account for this computation.

     The corresponding computation can be done by the robot for each of its joints or by the joint.

     \return a matrix \f$J \in {\bf R}^{6\times n_{dof}}\f$ defined by
     \f[
     J = \left(\begin{array}{llll}
     {\bf v_1} & {\bf v_2} & \cdots & {\bf v_{n_{dof}}} \\
     {\bf \omega_1} & {\bf \omega_2} & \cdots & {\bf \omega_{n_{dof}}}
     \end{array}\right)
     \f]
     where \f${\bf v_i}\f$ and \f${\bf \omega_i}\f$ are respectively the linear
     and angular velocities of the joint
     implied by the variation of degree of freedom \f$q_i\f$. The velocity of the
     joint returned by
     CjrlJoint::jointVelocity can thus be obtained through the following formula:
     \f[
     \left(\begin{array}{l} {\bf v} \\ {\bf \omega}\end{array}\right) = J {\bf \dot{q}}
     \f]
  */
  const matrixNxP& jacobianJointWrtConfig() const
  { matrixNxP r;
    return r;}

  /**
     \brief Compute the joint's jacobian wrt the robot configuration.
  */
  void computeJacobianJointWrtConfig()
  {}

  /**
     \brief Get the jacobian of the point specified in local frame by inPointJointFrame.
     The output matrix outjacobian is automatically resized if necessary

  */
  void getJacobianPointWrtConfig(const vector3d& inPointJointFrame, matrixNxP& outjacobian) const;
  {} 
  /**
     @}
  */

  /**
     \name Body linked to the joint
     @{
  */

  /**
     \brief Get a pointer to the linked body (if any).
  */
  CjrlBody* linkedBody() const
  { return static_cast<CjrlBody *>(0);} 

  /**
     \brief Link a body to the joint.
  */
  void setLinkedBody (CjrlBody& inBody)
  {}

  /**
     @}
  */

};

/**
   \brief Free flyer joint
*/
class METAPOD_ARD_DLLEXPORT JointFreeflyer : public Joint
{
public:
  JointFreeflyer(const matrix4d& inInitialPosition)
  {}
};

/**
   \brief Rotation joint
*/
class METAPOD_ARD_DLLEXPORT JointRotation : public Joint
{
public:
  JointRotation(const matrix4d& inInitialPosition)
  {}
};

/**
   \brief Translation joint
*/
class METAPOD_ARD_DLLEXPORT JointTranslation : public Joint
{
public:
  JointTranslation(const matrix4d& inInitialPosition)
  {}
};

/**
   \brief Anchor joint
*/
class METAPOD_ARD_DLLEXPORT JointAnchor : public Joint
{
public:
  JointAnchor(const matrix4d& inInitialPosition)
  {}
};


}
}


#endif /* METAPOD_ARD_JOINT_HH_ */
