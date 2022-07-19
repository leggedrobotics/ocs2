//
// Created by rgrandia on 25.03.22.
//

#pragma once

#include <ocs2_core/cost/StateInputGaussNewtonCostAd.h>

#include <ocs2_switched_model_interface/core/SwitchedModel.h>

#include <ocs2_anymal_motion_tracking/ModelParentSkeleton.h>

namespace switched_model {

class CartesianMotionTrackingCost : public ocs2::StateInputCostGaussNewtonAd {
 public:
  CartesianMotionTrackingCost(ModelParentSkeleton<ad_scalar_t> skeleton, const std::vector<AbsoluteTrackingTask<scalar_t>>& tasks,
                              const std::vector<TrackingOffset<scalar_t>>& offsets, std::vector<MotionTarget<scalar_t>> targetFrames,
                              bool recompile);

  ~CartesianMotionTrackingCost() override = default;
  CartesianMotionTrackingCost* clone() const { return new CartesianMotionTrackingCost(*this); }

  ocs2::vector_t getParameters(ocs2::scalar_t time, const ocs2::TargetTrajectories& targetTrajectories,
                               const ocs2::PreComputation& preComputation) const override;

  void updateTargets(const std::vector<MotionTarget<scalar_t>>& targetFrames) { targetFrames_ = targetFrames; }

 protected:
  CartesianMotionTrackingCost(const CartesianMotionTrackingCost& other) = default;

  ocs2::ad_vector_t costVectorFunction(ocs2::ad_scalar_t time, const ocs2::ad_vector_t& state, const ocs2::ad_vector_t& input,
                                       const ocs2::ad_vector_t& parameters) const override;

 private:
  ModelParentSkeleton<ad_scalar_t> skeleton_;
  std::vector<AbsoluteTrackingTask<ad_scalar_t>> tasks_;
  std::vector<TrackingOffset<ad_scalar_t>> offsets_;
  std::vector<MotionTarget<scalar_t>> targetFrames_;
};

}  // namespace switched_model