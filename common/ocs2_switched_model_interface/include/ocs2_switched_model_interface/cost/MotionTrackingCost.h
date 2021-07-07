//
// Created by rgrandia on 30.04.21.
//

#pragma once

#include <ocs2_core/cost/StateInputGaussNewtonCostAd.h>

#include "ocs2_switched_model_interface/core/ComModelBase.h"
#include "ocs2_switched_model_interface/core/KinematicsModelBase.h"
#include "ocs2_switched_model_interface/core/SwitchedModel.h"
#include "ocs2_switched_model_interface/logic/SwitchedModelModeScheduleManager.h"

namespace switched_model {

/**
 * Defines a cost over tracking base and feet references
 */
class MotionTrackingCost : public ocs2::StateInputCostGaussNewtonAd {
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
    vector3_t jointPosition{2.0, 2.0, 1.0};
    vector3_t footPosition{vector3_t::Constant(60.0)};
    vector3_t jointVelocity{0.02, 0.02, 0.01};
    vector3_t footVelocity{vector3_t::Constant(1.0)};
    vector3_t contactForce{vector3_t::Constant(0.001)};
  };

  using com_model_t = ComModelBase<ocs2::scalar_t>;
  using kinematic_model_t = KinematicsModelBase<ocs2::scalar_t>;
  using ad_kinematic_model_t = KinematicsModelBase<ocs2::CppAdInterface::ad_scalar_t>;

  MotionTrackingCost(const Weights& settings, const SwitchedModelModeScheduleManager& modeScheduleManager,
                     const kinematic_model_t& kinematicModel, const ad_kinematic_model_t& adKinematicModel, const com_model_t& comModel,
                     bool recompile);

  ~MotionTrackingCost() override = default;
  MotionTrackingCost* clone() const { return new MotionTrackingCost(*this); }

  ocs2::vector_t getParameters(ocs2::scalar_t time, const ocs2::TargetTrajectories& targetTrajectories) const override;

 protected:
  MotionTrackingCost(const MotionTrackingCost& other);

  ocs2::ad_vector_t costVectorFunction(ocs2::ad_scalar_t time, const ocs2::ad_vector_t& state, const ocs2::ad_vector_t& input,
                                       const ocs2::ad_vector_t& parameters) const override;

 private:
  const SwitchedModelModeScheduleManager* modeScheduleManagerPtr_;
  std::unique_ptr<ad_kinematic_model_t> adKinematicModelPtr_;
  std::unique_ptr<kinematic_model_t> kinematicModelPtr_;
  std::unique_ptr<com_model_t> comModelPtr_;

  // Only needed during model generation
  ocs2::ad_vector_t sqrtWeights_;
};

MotionTrackingCost::Weights loadWeightsFromFile(const std::string& filename, const std::string& fieldname, bool verbose = true);

}  // namespace switched_model
