//
// Created by rgrandia on 23.09.20.
//

#pragma once

#include <ocs2_switched_model_interface/core/SwitchedModel.h>

namespace switched_model {

template <typename SCALAR_T>
class WholebodyDynamics {
 public:
  struct DynamicsTerms {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Matrix<SCALAR_T, GENERALIZED_COORDINATE_SIZE, GENERALIZED_COORDINATE_SIZE> M;
    generalized_coordinate_s_t<SCALAR_T> C;
    generalized_coordinate_s_t<SCALAR_T> G;
  };

  WholebodyDynamics() = default;
  virtual ~WholebodyDynamics() = default;

  virtual WholebodyDynamics* clone() const = 0;

  WholebodyDynamics(const WholebodyDynamics&) = delete;
  WholebodyDynamics& operator=(const WholebodyDynamics&) = delete;
  WholebodyDynamics(WholebodyDynamics&&) = delete;
  WholebodyDynamics& operator=(WholebodyDynamics&&) = delete;

  virtual DynamicsTerms getDynamicsTerms(const switched_model::rbd_state_s_t<SCALAR_T>& rbdState) const = 0;
};

}  // namespace switched_model
