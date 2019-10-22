#pragma once

#include <ocs2_core/dynamics/SystemDynamicsBaseAD.h>

#include "ocs2_switched_model_interface/core/SwitchedModel.h"
#include "ocs2_switched_model_interface/core/ComModelBase.h"
#include "ocs2_switched_model_interface/core/KinematicsModelBase.h"

namespace switched_model {

class ComKinoSystemDynamicsAd : public ocs2::SystemDynamicsBaseAD<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Base = ocs2::SystemDynamicsBaseAD<STATE_DIM, INPUT_DIM>;
  using typename Base::ad_dynamic_vector_t;
  using typename Base::ad_scalar_t;

  using ad_com_model_t = ComModelBase<ad_scalar_t>;
  using ad_kinematic_model_t = KinematicsModelBase<ad_scalar_t>;

  using Vector3Ad = Eigen::Matrix<ad_scalar_t, 3, 1>;
  using Vector6Ad = Eigen::Matrix<ad_scalar_t, 6, 1>;
  using Matrix3Ad = Eigen::Matrix<ad_scalar_t, 3, 3>;
  using Matrix6Ad = Eigen::Matrix<ad_scalar_t, 6, 6>;
  using ad_joint_coordinate_t = Eigen::Matrix<ad_scalar_t, 12, 1>;
  using ad_base_coordinate_t = Eigen::Matrix<ad_scalar_t, 6, 1>;

  explicit ComKinoSystemDynamicsAd(const ad_kinematic_model_t& adKinematicModel, const ad_com_model_t& adComModel, bool recompileModel);

  ComKinoSystemDynamicsAd(const ComKinoSystemDynamicsAd& rhs);

  ~ComKinoSystemDynamicsAd() override = default;

  ComKinoSystemDynamicsAd* clone() const override;

  void systemFlowMap(ad_scalar_t time, const ad_dynamic_vector_t& state, const ad_dynamic_vector_t& input,
                     ad_dynamic_vector_t& stateDerivative) const override;

 private:
  std::unique_ptr<ad_kinematic_model_t> adKinematicModelPtr_;
  std::unique_ptr<ad_com_model_t> adComModelPtr_;
};

}  // namespace switched_model
