#pragma once

#include <ocs2_switched_model_interface/dynamics/ComKinoSystemDynamicsAd.h>

namespace anymal {

 class AnymalSystemDynamicsAd final : public switched_model::ComKinoSystemDynamicsAd {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Base = switched_model::ComKinoSystemDynamicsAd;

  explicit AnymalSystemDynamicsAd(bool recompileModel = true);

   AnymalSystemDynamicsAd(const AnymalSystemDynamicsAd& rhs);

  ~AnymalSystemDynamicsAd() override = default;

   AnymalSystemDynamicsAd* clone() const override;
};

}  // namespace anymal
