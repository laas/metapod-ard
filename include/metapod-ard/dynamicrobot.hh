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

#ifndef METAPOD_ARD_DYNAMIC_ROBOT_HH_
#define METAPOD_ARD_DYNAMIC_ROBOT_HH_

/* System headers */
#include <cassert>
#include <iostream>

#define NOT_IMPLEMENTED_YET() \
  std::cout << "Function " << __FUNCTION__ << " not implemented yet" << endl;

/* Metapod-ard headers */
#include <metapod-ard/mathsupport.hh>

/* Abstract Robot dynamics headers. */
#include <abstract-robot-dynamics/joint.hh>
#include <abstract-robot-dynamics/dynamic-robot.hh>
#include <abstract-robot-dynamics/robot-dynamics-object-constructor.hh>

/* Metapod headers */
#include <metapod/algos/rnea.hh>
#include <metapod/algos/crba.hh>

namespace metapodDelegate {

/** \ingroup userclasses
    \brief Classes to implement a non abstract class for a robot with dynamic properties
    from an object factory.
*/
template <typename Robot>
class dynamicRobot : public virtual CjrlDynamicRobot
{
 private:
  Robot robot_;
  typename Robot::confVector q_, dq_, ddq_;
  
 public:
  /**
     \name Initialization
     @{
  */
  
  
  /**
     \brief Initialize data-structure necessary to dynamic computations
     This function should be called after building the tree of joints.
  */
  bool initialize()
  {
    return true;
  }
  
  /**
     \brief Destructor
  */
  virtual ~dynamicRobot()
  {
  };

  /**
     @}
  */
  /**
     \name Kinematic chain
     @{
  */

  /**
     \brief Set the root joint of the robot.
  */
  void rootJoint(CjrlJoint& inJoint)
  {}

  /**
     \brief Get the root joint of the robot.
  */
  CjrlJoint* rootJoint() const
  { return & (boost::fusion::at_c<0>(robot_.nodes).joint);
  }

  /**
     \brief Get a vector containing all the joints.
  */
  std::vector< CjrlJoint* > jointVector()
  {
    std::vector<CjrlJoint *> r;
    r.clear();
    return r;
  }

  /**
     \brief Get the chain of joints between two joints
     \param inStartJoint First joint.
     \param inEndJoint Second joint.
  */
  std::vector<CjrlJoint*> jointsBetween(const CjrlJoint& inStartJoint,
                                        const CjrlJoint& inEndJoint) const
  {
    std::vector<CjrlJoint *> r;
    r.clear();
    return r;
  }

  /**
     \brief Get the upper bound for ith dof.
  */
  double upperBoundDof(unsigned int inRankInConfiguration)
  {
    return 0;
  }
  /**
     \brief Get the lower bound for ith dof.
  */
  double lowerBoundDof(unsigned int inRankInConfiguration)
  {
    return 0;
  }

  /**
     \brief Compute the upper bound for ith dof using other configuration values if possible.
  */
  double upperBoundDof(unsigned int inRankInConfiguration,
                       const vectorN& inConfig)
  {
    return 0;
  }
  
  /**
     \brief Compute the lower bound for ith dof using other configuration values if possible.
  */
  double lowerBoundDof(unsigned int inRankInConfiguration,
                       const vectorN& inConfig)
  {
    return 0;
  }

  /**
     \brief Get the number of degrees of freedom of the robot.
  */
  unsigned int numberDof() const
  {
    return Robot::NBDOF;
  }

  /**
     \brief Set the joint ordering in the configuration vector

     \param inJointVector Vector of the robot joints

     Specifies the order of the joints in the configuration vector.
     The vector should contain all the joints of the current robot.
  */
  void setJointOrderInConfig(std::vector<CjrlJoint*> inJointVector)
  {}

  /**
     @}
  */

  /**
     \name Configuration, velocity and acceleration
  */

  /**
     \brief Set the current configuration of the robot.

     \param inConfig the configuration vector \f${\bf q}\f$.

     \return true if success, false if failure (the dimension of the
     input vector does not fit the number of degrees of freedom of the
     robot).
  */
  bool currentConfiguration(const vectorN& inConfig)
  {
    q_ = inConfig;
    return true;
  }

  /**
     \brief Get the current configuration of the robot.

     \return the configuration vector \f${\bf q}\f$.
  */
  const vectorN& currentConfiguration() const
  {
    return q_;
  }

  /**
     \brief Set the current velocity of the robot.

     \param inVelocity the velocity vector \f${\bf \dot{q}}\f$.

     \return true if success, false if failure (the dimension of the
     input vector does not fit the number of degrees of freedom of the
     robot).
  */
  bool currentVelocity(const vectorN& inVelocity)
  {
    dq_ = inVelocity;
    return true;
  }

  /**
     \brief Get the current velocity of the robot.

     \return the velocity vector \f${\bf \dot{q}}\f$.
  */
  const vectorN& currentVelocity() const
  {
    return dq_;
  }
  
  /**
     \brief Set the current acceleration of the robot.

     \param inAcceleration the acceleration vector \f${\bf \ddot{q}}\f$.

     \return true if success, false if failure (the dimension of the
     input vector does not fit the number of degrees of freedom of the
     robot).
  */
  bool currentAcceleration(const vectorN& inAcceleration)
  {
    ddq_ = inAcceleration;
    return true;
  }

  /**
     \brief Get the current acceleration of the robot.

     \return the acceleration vector \f${\bf \ddot{q}}\f$.
  */
  const vectorN& currentAcceleration() const
  {
    return ddq_;
  }

  /**
     \brief Get the current forces of the robot.

     \return the force vector \f${\bf f}\f$.
  */
  const matrixNxP& currentForces() const
  {
    matrixNxP r;
    return r;
  }

  /**
     \brief Get the current torques of the robot.

     \return the torque vector \f${\bf \tau }\f$.
  */
  const matrixNxP& currentTorques() const
  {
    matrixNxP r;
    return r;
  }

  /**
     \brief Get the current joint torques of the robot.
       
     \return the torque vector \f${\bf \tau }\f$.
  */
  const vectorN& currentJointTorques() const 
  {
    vectorN r;
    return r;
  }

  /**
     @}
  */

  /**
     \name Forward kinematics and dynamics
  */


  /**
     \brief Compute forward kinematics.

     Update the position, velocity and accelerations of each
     joint wrt \f${\bf {q}}\f$, \f${\bf \dot{q}}\f$, \f${\bf \ddot{q}}\f$.

  */
  bool computeForwardKinematics()
  {
    metapod::rnea<Robot, true>::run(robot_,q_,dq_,ddq_);
    return true;
  }


  /**
     \brief Compute the dynamics of the center of mass.

     Compute the linear and  angular momentum and their time derivatives, at the center of mass.
  */
  bool computeCenterOfMassDynamics()
  {
    return true;
  }

  /**
     \brief Get the position of the center of mass.
  */
  const vector3d& positionCenterOfMass() const
  {
    vector3d r;
    return r;
  }

  /**
     \brief Get the velocity of the center of mass.
  */
  const vector3d& velocityCenterOfMass()
  {
    vector3d r;
    return r;
  }

  /**
     \brief Get the acceleration of the center of mass.
  */
  const vector3d& accelerationCenterOfMass()
  {
    vector3d r;
    return r;
  }

  /**
     \brief Get the linear momentum of the robot.
  */
  const vector3d& linearMomentumRobot()
  {
    vector3d r;
    return r;        
  }

  /**
     \brief Get the time-derivative of the linear momentum.
  */
  const vector3d& derivativeLinearMomentum()
  {
    vector3d r;
    return r;
  }

  /**
     \brief Get the angular momentum of the robot at the center of mass.
  */
  const vector3d& angularMomentumRobot()
  {
    vector3d r;
    return r;
  }

  /**
     \brief Get the time-derivative of the angular momentum at the center of mass.
  */
  const vector3d& derivativeAngularMomentum()
  {
    vector3d r;
    return r;
  }

  /**
     \brief Get the total mass of the robot
  */
  double mass() const
  {
    return robot_.mass_;
  }
  /**
     @}
  */

  /**
     @}
  */

  /**
     \name Control of the implementation
     @{
  */
    
  /**
     \brief Whether the specified property in implemented.
  */
  bool isSupported(const std::string &inProperty)
  {
    return false;
  }

  /**
     \brief Get property corresponding to command name.

     \param inProperty name of the property.
     \retval outValue value of the property if implemented.

     \note The returned string needs to be cast into the right type (double, int,...).
  */
  bool getProperty(const std::string &inProperty,
                   std::string& outValue)
  {
    return false;
  }

  /**
     \brief Set property corresponding to command name.

     \param inProperty name of the property.
     \param inValue value of the property.

     \note The value string is obtained by writing the
     corresponding value in a string (operator<<).
  */
  bool setProperty(std::string &inProperty,
                   const std::string& inValue)
  {
    return false;
  }

  /**
     @}
  */


  /**
     \brief Compute and get position and orientation jacobian

     \param inStartJoint First joint in the chain of joints influencing
     the jacobian.
     \param inEndJoint Joint where the control frame is located.
     \param inFrameLocalPosition Position of the control frame in inEndJoint
     local frame.
     \retval outjacobian computed jacobian matrix.
     \param offset is the rank of first non zero outjacobian column.
     \param inIncludeStartFreeFlyer Option to include the contribution of a
     fictive freeflyer superposed with inStartJoint

     \return false if matrix has inadequate size. Number of columns
     in matrix outJacobian must be at least numberDof() if
     inIncludeStartFreeFlyer = true.
     It must be at least numberDof()-6 otherwise.
  */
  bool getJacobian(const CjrlJoint& inStartJoint,
                   const CjrlJoint& inEndJoint,
                   const vector3d& inFrameLocalPosition,
                   matrixNxP& outjacobian,
                   unsigned int offset = 0,
                   bool inIncludeStartFreeFlyer = true)
  {
    return false;
  }

  bool getPositionJacobian(const CjrlJoint& inStartJoint,
                           const CjrlJoint& inEndJoint,
                           const vector3d& inFrameLocalPosition,
                           matrixNxP& outjacobian,
                           unsigned int offset = 0,
                           bool inIncludeStartFreeFlyer = true)
  {
    return false;
  }

  bool getOrientationJacobian(const CjrlJoint& inStartJoint,
                              const CjrlJoint& inEndJoint,
                              matrixNxP& outjacobian,
                              unsigned int offset = 0,
                              bool inIncludeStartFreeFlyer = true)
  {
    return false;
  }

  
  bool getJacobianCenterOfMass(const CjrlJoint& inStartJoint,
                               matrixNxP& outjacobian,
                               unsigned int offset = 0,
                               bool inIncludeStartFreeFlyer = true)
  {
    return false;
  }

  /*! \name Inertia matrix related methods
    @{ */
  /*! \brief Compute the inertia matrix of the robot according wrt \f${\bf q}\f$.
   */
  void computeInertiaMatrix()
  {
    robot_.H = metapod::MatrixN::Zero(Robot::NBDOF, Robot::NBDOF);
    metapod::crba< Robot, true >::run(robot_, q_); // Update geometry and 
  }

  /*! \brief Get the inertia matrix of the robot according wrt \f${\bf q}\f$.
   */
  const matrixNxP& inertiaMatrix() const
  {
    return robot_.H;
  }
  /*! @} */

  /*! \name Actuated joints related methods.
    @{
  */

  /**
     \brief Returns the list of actuated joints.
  */
  const std::vector<CjrlJoint*>& getActuatedJoints() const
  {
    std::vector<CjrlJoint*> r;
    return r;
  }

  /**
     \brief Specifies the list of actuated joints.
  */
  void setActuatedJoints(std::vector<CjrlJoint*>& lActuatedJoints)
  {
  }

  /*!
    @}
  */


  /*! \brief Compute Speciliazed InverseKinematics between two joints.

    Specialized means that this method can be re implemented to be
    extremly efficient and used the particularity of your robot.
    For instance in some case, it is possible to use an exact inverse
    kinematics to compute a set of articular value.

    This method does not intend to replace an architecture computing
    inverse kinematics through the Jacobian.

    jointRootPosition and jointEndPosition have to be expressed in the same
    frame.

    \param[in] jointRoot: The root of the joint chain for which the specialized
    inverse kinematics should be computed.

    \param[in] jointEnd: The end of the joint chain for which the specialized
    inverse kinematics should be computed.

    \param[in] jointRootPosition: The desired position of the root.

    \param[in] jointEndPosition: The end position of the root.

    \param[out] q: Result i.e. the articular values.
  */
  bool getSpecializedInverseKinematics(const CjrlJoint & jointRoot,
                                       const CjrlJoint & jointEnd,
                                       const matrix4d & jointRootPosition,
                                       const matrix4d & jointEndPosition,
                                       vectorN &q)
  {
    return true;
  }


};

} // end of namespace metapod-ard.

#endif /* METAPOD_ARD_DYNAMIC_ROBOT_HH_ */
