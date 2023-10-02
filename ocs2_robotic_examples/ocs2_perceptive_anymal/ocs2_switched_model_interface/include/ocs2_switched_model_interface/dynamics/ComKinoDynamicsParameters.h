//
// Created by rgrandia on 04.06.21.
//

#pragma once

#include "ocs2_switched_model_interface/core/SwitchedModel.h"

namespace switched_model {

/**
 * Parameters of the system dynamics (constant over the MPC horizon)
 */
template <typename SCALAR_T>
struct ComKinoSystemDynamicsParameters {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  //! External forces on the base, expressed in the origin frame
  vector3_s_t<SCALAR_T> externalForceInOrigin = vector3_s_t<SCALAR_T>::Zero();
  //! External torques on the base, expressed in the base frame
  vector3_s_t<SCALAR_T> externalTorqueInBase = vector3_s_t<SCALAR_T>::Zero();

  /**
   * Provides the number of parameters in this struct. Needs to be fixed before model generation and stay constant afterwards.
   */
  static size_t getNumParameters() { return 6; };

  /** Construct with default values */
  ComKinoSystemDynamicsParameters() = default;

  /** Construct from vector */
  explicit ComKinoSystemDynamicsParameters(const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& parameterVector);

  /** Flatten to a vector */
  Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1> asVector() const;
};

//! Explicit instantiations
extern template class ComKinoSystemDynamicsParameters<scalar_t>;
extern template class ComKinoSystemDynamicsParameters<ad_scalar_t>;

}  // namespace switched_model