//
// Created by rgrandia on 30.04.21.
//

#pragma once

#include <ocs2_core/cost/StateInputGaussNewtonCostAd.h>

#include "ocs2_switched_model_interface/core/ComModelBase.h"
#include "ocs2_switched_model_interface/core/KinematicsModelBase.h"
#include "ocs2_switched_model_interface/core/ModelSettings.h"
#include "ocs2_switched_model_interface/core/SwitchedModel.h"
#include "ocs2_switched_model_interface/logic/SwitchedModelModeScheduleManager.h"

namespace switched_model {

/**
 * Defines a cost over tracking base and feet references
 */
class MotionTrackingCost final : public ocs2::StateInputCostGaussNewtonAd {
 public:
  /**
   * Cost settings per motion target, weights are applied as: cost = sum_i {w_i * (target_i - ref_i)^2}
   * Magic values set as default
   */
  struct Weights {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    vector3_t eulerXYZ{100.0, 200.0, 200.0};
    vector3_t comPosition{1000.0, 1000.0, 1500.0};
    vector3_t comAngularVelocity{5.0, 10.0, 10.0};
    vector3_t comLinearVelocity{15.0, 15.0, 30.0};
    feet_array_t<vector3_t> jointPosition = constantFeetArray(vector3_t(2.0, 2.0, 1.0));
    feet_array_t<vector3_t> footPosition = constantFeetArray(vector3_t(60.0, 60.0, 60.0));
    feet_array_t<vector3_t> jointVelocity = constantFeetArray(vector3_t(0.02, 0.02, 0.01));
    feet_array_t<vector3_t> footVelocity = constantFeetArray(vector3_t(1.0, 1.0, 1.0));
    feet_array_t<vector3_t> contactForce = constantFeetArray(vector3_t(0.001, 0.001, 0.001));
  };

  using kinematic_model_t = KinematicsModelBase<ocs2::scalar_t>;
  using ad_kinematic_model_t = KinematicsModelBase<ocs2::ad_scalar_t>;
  using com_model_t = ComModelBase<ocs2::scalar_t>;
  using ad_com_model_t = ComModelBase<ocs2::ad_scalar_t>;

  MotionTrackingCost(const Weights& settings, const ad_kinematic_model_t& adKinematicModel, const ModelSettings& modelSettings);

  ~MotionTrackingCost() override = default;
  MotionTrackingCost* clone() const { return new MotionTrackingCost(*this); }

  ocs2::vector_t getParameters(ocs2::scalar_t time, const ocs2::TargetTrajectories& targetTrajectories,
                               const ocs2::PreComputation& preComputation) const override;

 protected:
  MotionTrackingCost(const MotionTrackingCost& other);

  ocs2::ad_vector_t costVectorFunction(ocs2::ad_scalar_t time, const ocs2::ad_vector_t& state, const ocs2::ad_vector_t& input,
                                       const ocs2::ad_vector_t& parameters) const override;

 private:
  std::unique_ptr<ad_kinematic_model_t> adKinematicModelPtr_;
  ocs2::vector_t sqrtWeights_;
  const ModelSettings& modelSettings_;
};

MotionTrackingCost::Weights loadWeightsFromFile(const std::string& filename, const std::string& fieldname, bool verbose = true);

}  // namespace switched_model
