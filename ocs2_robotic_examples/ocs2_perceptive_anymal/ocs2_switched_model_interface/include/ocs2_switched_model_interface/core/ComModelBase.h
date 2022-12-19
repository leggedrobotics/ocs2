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
   * Total mass of robot
   * @return mass in kg
   */
  virtual SCALAR_T totalMass() const = 0;

  /**
   * Computes the base acceleration based on the top 6 rows of the full rigid body dynamics of the system.
   *
   * @param basePose : {EulerXYZ (3x1), base position in Origin frame (3x1)}
   * @param baseLocalVelocities : {base angular velocity in Base Frame (3x1), base linear velocity in Base Frame (3x1)}
   * @param jointPositions : Joint positions (12x1)
   * @param jointVelocities : Joint velocities (12x1)
   * @param jointAccelerations : Joint accelerations (12x1)
   * @param forcesOnBaseInBaseFrame : { torque on base in Base Frame (3x1), linear forces on the base in Base Frame (3x1) }.
   * Forces on the base due to contact forces, i.e. J^T * F should also be included here.
   * @return baseLocalAccelerations : {base angular acceleration in Base Frame (3x1), base linear acceleration in Base Frame (3x1)}
   */
  virtual base_coordinate_s_t<SCALAR_T> calculateBaseLocalAccelerations(
      const base_coordinate_s_t<SCALAR_T>& basePose, const base_coordinate_s_t<SCALAR_T>& baseLocalVelocities,
      const joint_coordinate_s_t<SCALAR_T>& jointPositions, const joint_coordinate_s_t<SCALAR_T>& jointVelocities,
      const joint_coordinate_s_t<SCALAR_T>& jointAccelerations,
      const switched_model::base_coordinate_s_t<SCALAR_T>& forcesOnBaseInBaseFrame) const = 0;
};

extern template class ComModelBase<scalar_t>;
extern template class ComModelBase<ocs2::CppAdInterface::ad_scalar_t>;

/**
 * Distribute the weight equally among all stance feet.
 *
 * @return model inputs with {Forces in base, joint velocies}
 */
comkino_input_t weightCompensatingInputs(const ComModelBase<scalar_t>& comModel, const contact_flag_t& contactFlags,
                                         const vector3_t& baseOrientation);

comkino_input_t weightCompensatingInputs(scalar_t mass, const contact_flag_t& contactFlags, const vector3_t& baseOrientation);

}  // end of namespace switched_model
