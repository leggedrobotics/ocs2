#pragma once

#include <ocs2_core/dynamics/SystemDynamicsBaseAD.h>

namespace anymal {

 class AnymalSystemDynamicsAd final : public ocs2::SystemDynamicsBaseAD<24, 24>
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE = ocs2::SystemDynamicsBaseAD<24, 24>;
  using typename BASE::ad_scalar_t;
  using typename BASE::ad_dynamic_vector_t;

   using Vector3Ad = Eigen::Matrix<ad_scalar_t, 3, 1>;
   using Vector6Ad = Eigen::Matrix<ad_scalar_t, 6, 1>;
   using Matrix3Ad = Eigen::Matrix<ad_scalar_t, 3, 3>;
   using Matrix6Ad = Eigen::Matrix<ad_scalar_t, 6, 6>;
   using ad_joint_coordinate_t = Eigen::Matrix<ad_scalar_t, 12, 1>;
   using ad_base_coordinate_t = Eigen::Matrix<ad_scalar_t, 6, 1>;

  explicit AnymalSystemDynamicsAd(bool recompileModel = true);

  ~AnymalSystemDynamicsAd() override = default;

   void systemFlowMap(ad_scalar_t time, const ad_dynamic_vector_t& state, const ad_dynamic_vector_t& input,
                      ad_dynamic_vector_t& stateDerivative) const override;
};

} // namespace anymal

