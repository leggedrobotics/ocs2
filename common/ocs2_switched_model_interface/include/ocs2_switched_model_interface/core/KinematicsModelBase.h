/*
 * KinematicsModelBase.h
 *
 *  Created on: Aug 2, 2017
 *      Author: Farbod
 */

#ifndef MODELKINEMATICSBASE_H_
#define MODELKINEMATICSBASE_H_

#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <memory>

#include "ocs2_switched_model_interface/core/SwitchedModel.h"

namespace switched_model {

/**
 * ModelKinematics Base Class
 * @tparam SCALAR_T
 */
template <typename SCALAR_T = double>
class KinematicsModelBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using base_coordinate_t = Eigen::Matrix<SCALAR_T, BASE_COORDINATE_SIZE, 1>;
  using joint_coordinate_t = Eigen::Matrix<SCALAR_T, JOINT_COORDINATE_SIZE, 1>;
  using generalized_coordinate_t = Eigen::Matrix<SCALAR_T, GENERALIZED_COORDINATE_SIZE, 1>;

  using geometric_jacobian_t = Eigen::Matrix<SCALAR_T, 6, GENERALIZED_COORDINATE_SIZE>;
  using joint_jacobian_t = Eigen::Matrix<SCALAR_T, 6, JOINT_COORDINATE_SIZE>;

  typedef Eigen::Matrix<SCALAR_T, 3, 1> vector3d_t;
  typedef Eigen::Matrix<SCALAR_T, 3, 3> matrix3d_t;

  KinematicsModelBase() = default;

  virtual ~KinematicsModelBase() = default;

  /**
   * Clone KinematicsModelBase class.
   */
  virtual KinematicsModelBase<SCALAR_T>* clone() const = 0;

  /**
   * Gets a (6+JOINT_COORD_SIZE)-by-1 generalized coordinate and calculates the rotation ...
   * @param [in] generalizedCoordinate
   */
  void update(const generalized_coordinate_t& generalizedCoordinate);

  /**
   * Gets the base and joint coordinate calculates the rotation ...
   * @param [in] qBase
   * @param [in] qJoint
   */
  void update(const base_coordinate_t& qBase, const joint_coordinate_t& qJoint);

  /**
   * calculate the feet position in the Base frame
   * 		+ single foot:
   * 			 using the internal coordinate values set by update method.
   *
   *		+ four feet:
   * 			 using the internal coordinate values set by update method.
   *
   * @param [in] footIndex
   * @param [out] footPosition
   */
  virtual vector3d_t footPositionBaseFrame(size_t footIndex) const = 0;

  /**
   * Calculate the feet position in the Base frame
   * @param [out] feetPositions
   */
  std::array<vector3d_t, NUM_CONTACT_POINTS> feetPositionsBaseFrame() const;

  /**
   * Calculate the feet position in the Origin frame
   * 		+ single foot:
   * 			 using the internal coordinate values set by update method.
   * 			 using the input coordinate values.
   *		+ four feet:
   * 			 using the internal coordinate values set by update method.
   * 			 using the input coordinate values.
   * @param [in] footIndex
   * @param [out] footPosition
   */
  vector3d_t footPositionOriginFrame(size_t footIndex) const;

  /**
   * Calculate feet Positions in Origin Frame
   * @param [out] feetPositions
   */
  std::array<vector3d_t, NUM_CONTACT_POINTS> feetPositionsOriginFrame() const;

  /**
   * Calculate foot Jacobian in Base frame
   * @param [in] footIndex
   * @param [out] footJacobian
   */
  virtual joint_jacobian_t footJacobianBaseFrame(size_t footIndex) const = 0;

  /**
   * calculates the Jacobian matrix in the Inertia frame using rotation "i_R_b" from the Base frame to Inertia
   * Frame and the Jacobian matrix expressed in the Base frame.
   * @param [in] i_R_b
   * @param [in] b_r_point
   * @param [in] b_J_point
   * @return i_J_point
   */
  static geometric_jacobian_t FromBaseJacobianToInertiaJacobian(const matrix3d_t& i_R_b, const vector3d_t& b_r_point,
                                                                const joint_jacobian_t& b_J_point);

  /**
   * calculates the velocity in the Inertia frame using rotation "i_R_b" from the Base frame to Inertia
   * Frame, local velocities of base, and the velocity of the point expressed in the Base frame.
   *
   * @param i_R_b
   * @param baseLocalVelocities
   * @param b_r_point
   * @param b_v_point
   * @return i_v_point
   */
  static vector3d_t FromBaseVelocityToInertiaVelocity(const matrix3d_t& i_R_b, const base_coordinate_t& baseLocalVelocities,
                                                      const vector3d_t& b_r_point, const vector3d_t& b_v_point);

 protected:
  base_coordinate_t qBase_;
  joint_coordinate_t qJoint_;
  matrix3d_t b_R_o_;
};

}  // end of namespace switched_model

#include "implementation/KinematicsModelBase.h"

#endif /* MODELKINEMATICSBASE_H_ */
