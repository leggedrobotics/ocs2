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
 * @tparam JOINT_COORD_SIZE
 * @tparam SCALAR_T
 */
template <typename SCALAR_T = double>
class KinematicsModelBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using base_coordinate_t = Eigen::Matrix<SCALAR_T, BASE_COORDINATE_SIZE, 1>;
  using joint_coordinate_t = Eigen::Matrix<SCALAR_T, JOINT_COORDINATE_SIZE, 1>;
  using generalized_coordinate_t = Eigen::Matrix<SCALAR_T, GENERALIZED_COORDINATE_SIZE, 1>;

  using geometic_jacobian_t = Eigen::Matrix<SCALAR_T, 6, GENERALIZED_COORDINATE_SIZE>;
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
  template <typename BASE_COORDINATE, typename JOINT_COORDINATE>
  void update(const Eigen::DenseBase<BASE_COORDINATE>& qBase, const Eigen::DenseBase<JOINT_COORDINATE>& qJoint);

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
  virtual void footPositionBaseFrame(const size_t& footIndex, vector3d_t& footPosition) = 0;

  /**
   * Calculate the feet position in the Base frame
   * @param [out] feetPositions
   */
  void feetPositionsBaseFrame(std::array<vector3d_t, 4>& feetPositions);

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
  void footPositionOriginFrame(const size_t& footIndex, vector3d_t& footPosition);

  /**
   * Calculate feet Positions in Origin Frame
   * @param [in] generalizedCoordinate
   * @param [in] footIndex
   * @param [out] footPosition
   */
  void footPositionOriginFrame(const generalized_coordinate_t& generalizedCoordinate, const size_t& footIndex, vector3d_t& footPosition);

  /**
   * Calculate feet Positions in Origin Frame
   * @param [out] feetPositions
   */
  void feetPositionsOriginFrame(std::array<vector3d_t, 4>& feetPositions);

  /**
   * Calculate feet Positions in Origin Frame
   * @param [in] generalizedCoordinate
   * @param [out] feetPositions
   */
  void feetPositionsOriginFrame(const generalized_coordinate_t& generalizedCoordinate, std::array<vector3d_t, 4>& feetPositions);

  /**
   * Calculate foot Jacobian in Base frame
   * @param [in] footIndex
   * @param [out] footJacobain
   */
  virtual void footJacobainBaseFrame(const size_t& footIndex, joint_jacobian_t& footJacobain) = 0;

  /**
   * calculates the Jacobian matrix in the Inertia frame using rotation "i_R_b" from the Base frame to Inertia
   * Frame and the Jacobian matrix expressed in the Base frame.
   * @param [in] i_R_b
   * @param [in] b_r_point
   * @param [in] b_J_point
   * @param [out] i_J_point
   */
  static void FromBaseJacobianToInertiaJacobian(const matrix3d_t& i_R_b, const vector3d_t& b_r_point,
                                                const joint_jacobian_t& b_J_point,
                                                geometic_jacobian_t& i_J_point);

  /**
   * calculates the velocity in the Inertia frame using rotation "i_R_b" from the Base frame to Inertia
   * Frame, local velocities of base, and the velocity of the point expressed in the Base frame.
   *
   * @param i_R_b
   * @param baseLocalVelocities
   * @param b_r_point
   * @param b_v_point
   * @param i_v_point
   */
  static void FromBaseVelocityToInertiaVelocity(const matrix3d_t& i_R_b, const base_coordinate_t& baseLocalVelocities,
                                                const vector3d_t& b_r_point, const vector3d_t& b_v_point, vector3d_t& i_v_point);


 protected:
  base_coordinate_t qBase_;
  joint_coordinate_t qJoint_;
  matrix3d_t b_R_o_;
};

}  // end of namespace switched_model

#include "implementation/KinematicsModelBase.h"

#endif /* MODELKINEMATICSBASE_H_ */
