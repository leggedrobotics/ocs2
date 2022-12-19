//
// Created by rgrandia on 25.03.22.
//

#include <gtest/gtest.h>

#include <ocs2_core/automatic_differentiation/CppAdInterface.h>
#include <ocs2_robotic_assets/package_path.h>

#include "ocs2_anymal_motion_tracking/ModelParentSkeleton.h"

namespace {
const std::string URDF_FILE = ocs2::robotic_assets::getPath() + "/resources/anymal_c/urdf/anymal.urdf";
}  // unnamed namespace

using namespace switched_model;

TEST(testModelParentSkeleton, pinocchioInterface) {
  auto pinocchioInterface = getPinocchioInterface<scalar_t>(URDF_FILE);
  const auto& frames = getAllFrames(pinocchioInterface);
  for (const auto& frame : frames) {
    std::cout << frame << std::endl;
  }
  ModelParentSkeleton<scalar_t> skeleton(std::move(pinocchioInterface), {"base", "LF_FOOT"});
  vector_t x = state_vector_t::Zero();
  vector_t u = input_vector_t::Zero();
  skeleton.update(x, u);
}

TEST(testModelParentSkeleton, autodiff) {
  auto pinocchioInterface = getPinocchioInterface<ad_scalar_t>(URDF_FILE);
  ModelParentSkeleton<ad_scalar_t> skeleton(std::move(pinocchioInterface), {"base", "LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"});

  auto func = [&](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) {
    const ad_vector_t state = x.head(STATE_DIM);
    const ad_vector_t input = x.tail(INPUT_DIM);
    auto targets = skeleton.update(state, input);

    y.resize(targets.size() * 18);
    int idx = 0;
    for (const auto& target : targets) {
      y.segment(idx, 3) = target.poseInWorld.position;
      y.segment(idx + 3, 3) = target.poseInWorld.orientation.col(0);
      y.segment(idx + 6, 3) = target.poseInWorld.orientation.col(1);
      y.segment(idx + 9, 3) = target.poseInWorld.orientation.col(2);
      y.segment(idx + 12, 3) = target.twistInWorld.linear;
      y.segment(idx + 15, 3) = target.twistInWorld.angular;
      idx += 18;
    }
  };
  ocs2::CppAdInterface adInterface(func, STATE_DIM + INPUT_DIM, 0, "test_skeleton", "/tmp/ocs2");

  adInterface.createModels(ocs2::CppAdInterface::ApproximationOrder::First, true);
}

TEST(testModelParentSkeleton, tracking) {
  auto pinocchioInterface = getPinocchioInterface<ad_scalar_t>(URDF_FILE);
  ModelParentSkeleton<ad_scalar_t> skeleton(std::move(pinocchioInterface), {"base", "LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"});

  MotionTarget<ad_scalar_t> worldTarget = MotionTarget<ad_scalar_t>::Zero();

  auto func = [&](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) {
    const ad_vector_t state = x.head(STATE_DIM);
    const ad_vector_t input = x.tail(INPUT_DIM);
    auto sources = skeleton.update(state, input);

    y.resize(sources.size() * 6);
    int idx = 0;
    for (size_t i = 0; i < sources.size(); ++i) {
      AbsoluteTrackingTask<ad_scalar_t> task;
      task.targetId = 0;
      task.sourceId = i;
      task.poseWeights.setOnes();
      task.twistweights.setOnes();

      // Add both position and orientation costs
      task.type = TrackingTaskType::POSITION;
      y.segment(idx, 3) = getCostVector(task, sources, {worldTarget});
      task.type = TrackingTaskType::ORIENTATION;
      y.segment(idx + 3, 3) = getCostVector(task, sources, {worldTarget});
      idx += 6;
    }
  };
  ocs2::CppAdInterface adInterface(func, STATE_DIM + INPUT_DIM, 0, "test_tracking", "/tmp/ocs2");

  adInterface.createModels(ocs2::CppAdInterface::ApproximationOrder::First, true);
}