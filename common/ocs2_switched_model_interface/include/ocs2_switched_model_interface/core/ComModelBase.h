#pragma once

#include <Eigen/Dense>

namespace switched_model {

/**
 * CoM Model Base Class
 */
template <typename SCALAR_T>
class ComModelBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using base_coordinate_t = Eigen::Matrix<SCALAR_T, 6, 1>;
  using joint_coordinate_t = Eigen::Matrix<SCALAR_T, 12, 1>;

  using vector3s_t = Eigen::Matrix<SCALAR_T, 3, 1>;
  using matrix3s_t = Eigen::Matrix<SCALAR_T, 3, 3>;
  using matrix6s_t = Eigen::Matrix<SCALAR_T, 6, 6>;

  ComModelBase() = default;

  virtual ~ComModelBase() = default;

  /**
   * Clone ComModelBase class.
   */
  virtual ComModelBase<SCALAR_T>* clone() const = 0;

  /**
  * Set default joint configuration. Updates the CoM position and inertia
  */
  virtual void setJointConfiguration(const joint_coordinate_t& q) = 0;

  /**
   * Calculate CoM Position in Base frame from Base origin
   */
  virtual vector3s_t comPositionBaseFrame() const = 0;

  /**
   *   Calculate CoM inertia tensor around CoM
   */
  virtual matrix6s_t comInertia() const = 0;

  /**
   * Total mass of robot
   * @return mass in kg
   */
  virtual SCALAR_T totalMass() const = 0;

  /**
   * Calculate Base Pose from CoM pose
   * The Base pose (basePose) consists of:
   * 		+ Base orientation w.r.t origin frame (3-states)
   * 		+ Base position w.r.t origin frame (3-states)
   *
   * @param [in]  comPose
   * @param [out] basePose
   */
  base_coordinate_t calculateBasePose(const base_coordinate_t& comPose) const;

  base_coordinate_t calculateComPose(const base_coordinate_t& basePose) const;

  /**
   * Calculates the Base local velocities based on the CoM local velocities (comLocalVelocities).
   * The Base local velocities (baseLocalVelocities) consists of angular and
   * linear velocities in base frame (inertia frame coincide at Base frame)
   * (6-states)
   */
  base_coordinate_t calculateBaseLocalVelocities(const base_coordinate_t& comLocalVelocities) const;

  base_coordinate_t calculateComLocalVelocities(const base_coordinate_t& baseLocalVelocities) const;
};

}  // end of namespace switched_model

#include "implementation/ComModelBase.h"
