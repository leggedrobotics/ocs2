//
// Created by rgrandia on 04.06.21.
//

#include "ocs2_switched_model_interface/dynamics/ComKinoDynamicsParameters.h"

namespace switched_model {

template <typename SCALAR_T>
ComKinoSystemDynamicsParameters<SCALAR_T>::ComKinoSystemDynamicsParameters(
    const Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>& parameterVector)
    : externalForceInOrigin(parameterVector.template segment<3>(0)), externalTorqueInBase(parameterVector.template segment<3>(3)) {}

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1> ComKinoSystemDynamicsParameters<SCALAR_T>::asVector() const {
  Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1> parameters(ComKinoSystemDynamicsParameters::getNumParameters());
  parameters.template head<3>() = externalForceInOrigin;
  parameters.template tail<3>() = externalTorqueInBase;
  return parameters;
}

template class ComKinoSystemDynamicsParameters<scalar_t>;
template class ComKinoSystemDynamicsParameters<ad_scalar_t>;

}  // namespace switched_model