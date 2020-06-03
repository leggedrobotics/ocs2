#pragma once

#include <ocs2_switched_model_interface/core/SwitchedModel.h>

namespace switched_model {

/**
 * CoM Model Base Class
 */
template <typename SCALAR_T>
class ComModelBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ComModelBase() = default;

  virtual ~ComModelBase() = default;

  /**
   * Clone ComModelBase class.
   */
  virtual ComModelBase<SCALAR_T>* clone() const = 0;

  /**
   * Set default joint configuration. Updates the CoM position and inertia
   */
  virtual void setJointConfiguration(const joint_coordinate_s_t<SCALAR_T>& q) = 0;

  /**
   * Calculate CoM Position in Base frame from Base origin
   */
  virtual vector3_s_t<SCALAR_T> comPositionBaseFrame() const = 0;

  /**
   *   Calculate CoM inertia tensor around CoM
   */
  virtual matrix6_s_t<SCALAR_T> comInertia() const = 0;

  matrix3_s_t<SCALAR_T> rotationalInertia() const;

  matrix6_s_t<SCALAR_T> comInertiaInverse() const;

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
  base_coordinate_s_t<SCALAR_T> calculateBasePose(const base_coordinate_s_t<SCALAR_T>& comPose) const;

  base_coordinate_s_t<SCALAR_T> calculateComPose(const base_coordinate_s_t<SCALAR_T>& basePose) const;

  /**
   * Calculates the Base local velocities based on the CoM local velocities (comLocalVelocities).
   * The Base local velocities (baseLocalVelocities) consists of angular and
   * linear velocities in base frame (inertia frame coincide at Base frame)
   * (6-states)
   */
  base_coordinate_s_t<SCALAR_T> calculateBaseLocalVelocities(const base_coordinate_s_t<SCALAR_T>& comLocalVelocities) const;

  base_coordinate_s_t<SCALAR_T> calculateComLocalVelocities(const base_coordinate_s_t<SCALAR_T>& baseLocalVelocities) const;

  base_coordinate_s_t<SCALAR_T> calculateBaseLocalAccelerations(const base_coordinate_s_t<SCALAR_T>& comLocalAccelerations,
                                                                const base_coordinate_s_t<SCALAR_T>& comLocalVelocities) const;

  base_coordinate_s_t<SCALAR_T> calculateComLocalAccelerations(const base_coordinate_s_t<SCALAR_T>& baseLocalAccelerations,
                                                               const base_coordinate_s_t<SCALAR_T>& baseLocalVelocities) const;
};

extern template class ComModelBase<scalar_t>;
extern template class ComModelBase<ocs2::CppAdInterface::ad_scalar_t>;

}  // end of namespace switched_model
