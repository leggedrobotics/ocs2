#pragma once

#include <ocs2_core/dynamics/SystemDynamicsBaseAD.h>

#include "ocs2_switched_model_interface/core/ComModelBase.h"
#include "ocs2_switched_model_interface/core/KinematicsModelBase.h"
#include "ocs2_switched_model_interface/core/SwitchedModel.h"

namespace switched_model {

class ComKinoSystemDynamicsAd : public ocs2::SystemDynamicsBaseAD {
 public:
  using Base = ocs2::SystemDynamicsBaseAD;

  using ad_com_model_t = ComModelBase<ocs2::ad_scalar_t>;
  using ad_kinematic_model_t = KinematicsModelBase<ocs2::ad_scalar_t>;

  explicit ComKinoSystemDynamicsAd(const ad_kinematic_model_t& adKinematicModel, const ad_com_model_t& adComModel, bool recompileModel);

  ComKinoSystemDynamicsAd(const ComKinoSystemDynamicsAd& rhs);

  ~ComKinoSystemDynamicsAd() override = default;

  ComKinoSystemDynamicsAd* clone() const override;

  ocs2::ad_vector_t systemFlowMap(ocs2::ad_scalar_t time, const ocs2::ad_vector_t& state, const ocs2::ad_vector_t& input,
                                  const ocs2::ad_vector_t& parameters) const override;

  template <typename SCALAR_T>
  static com_state_s_t<SCALAR_T> computeComStateDerivative(const ComModelBase<SCALAR_T>& comModel,
                                                           const KinematicsModelBase<SCALAR_T>& kinematicsModel,
                                                           const comkino_state_s_t<SCALAR_T>& comKinoState,
                                                           const comkino_state_s_t<SCALAR_T>& comKinoInput);

 private:
  std::unique_ptr<ad_kinematic_model_t> adKinematicModelPtr_;
  std::unique_ptr<ad_com_model_t> adComModelPtr_;
};

//! Explicit instantiations of computeComStateDerivative
extern template com_state_t ComKinoSystemDynamicsAd::computeComStateDerivative(const ComModelBase<scalar_t>& comModel,
                                                                               const KinematicsModelBase<scalar_t>& kinematicsModel,
                                                                               const comkino_state_t& comKinoState,
                                                                               const comkino_state_t& comKinoInput);
extern template com_state_ad_t ComKinoSystemDynamicsAd::computeComStateDerivative(
    const ComModelBase<ocs2::CppAdInterface::ad_scalar_t>& comModel,
    const KinematicsModelBase<ocs2::CppAdInterface::ad_scalar_t>& kinematicsModel, const comkino_state_ad_t& comKinoState,
    const comkino_state_ad_t& comKinoInput);

}  // namespace switched_model
