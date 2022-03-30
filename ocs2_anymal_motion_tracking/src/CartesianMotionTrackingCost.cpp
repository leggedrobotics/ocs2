//
// Created by rgrandia on 25.03.22.
//

#include "ocs2_anymal_motion_tracking/CartesianMotionTrackingCost.h"

namespace {
const int paramsPerTarget = 18;
const int costsPerTask = 3;
}  // namespace

namespace switched_model {

CartesianMotionTrackingCost::CartesianMotionTrackingCost(ModelParentSkeleton<ad_scalar_t> skeleton,
                                                         const std::vector<AbsoluteTrackingTask<scalar_t>>& tasks,
                                                         const std::vector<TrackingOffset<scalar_t>>& offsets,
                                                         std::vector<MotionTarget<scalar_t>> targetFrames, bool recompile)
    : skeleton_(std::move(skeleton)), targetFrames_(std::move(targetFrames)) {
  tasks_.reserve(tasks.size());
  for (const auto& task : tasks) {
    tasks_.push_back(task.cast<ad_scalar_t>());
  }

  offsets_.reserve(offsets.size());
  for (const auto& offset : offsets) {
    offsets_.push_back(offset.cast<ad_scalar_t>());
  }

  initialize(STATE_DIM, INPUT_DIM, paramsPerTarget * targetFrames_.size(), "CartesianMotionTrackingCost", "/tmp/ocs2", recompile);
}

ocs2::vector_t CartesianMotionTrackingCost::getParameters(ocs2::scalar_t time, const ocs2::TargetTrajectories& targetTrajectories) const {
  ocs2::vector_t parameters(paramsPerTarget * targetFrames_.size());
  for (int i = 0; i < targetFrames_.size(); ++i) {
    addToVector(targetFrames_[i], parameters, i * paramsPerTarget);
  }
  return parameters;
}

ocs2::ad_vector_t CartesianMotionTrackingCost::costVectorFunction(ocs2::ad_scalar_t time, const ocs2::ad_vector_t& state,
                                                                  const ocs2::ad_vector_t& input,
                                                                  const ocs2::ad_vector_t& parameters) const {
  // Unpack targets
  std::vector<MotionTarget<ad_scalar_t>> adTargets;
  adTargets.reserve(targetFrames_.size());
  for (int i = 0; i < targetFrames_.size(); ++i) {
    adTargets.push_back(fromVector(parameters, i * paramsPerTarget));
  }

  // Compute parent frames
  auto skeleton = skeleton_;
  const auto& parents = skeleton.update(state, input);

  // Compute sources
  std::vector<MotionTarget<ad_scalar_t>> sources;
  sources.resize(offsets_.size());
  updateSourceFrames(parents, offsets_, sources);

  // Compute costs
  ocs2::ad_vector_t y(costsPerTask * tasks_.size());
  for (int i = 0; i < tasks_.size(); ++i) {
    y.segment(costsPerTask * i, 3) = getCostVector(tasks_[i], sources, adTargets);
  }
  return y;
}

}  // namespace switched_model