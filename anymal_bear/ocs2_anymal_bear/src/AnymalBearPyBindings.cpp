#include "ocs2_anymal_bear/AnymalBearPyBindings.h"

#include <ocs2_core/constraint/RelaxedBarrierPenalty.h>

#include <ocs2_quadruped_interface/QuadrupedSlqMpc.h>

#include "ocs2_anymal_bear/AnymalBearInterface.h"

namespace anymal {

AnymalBearPyBindings::AnymalBearPyBindings(std::string taskName) : taskName_(std::move(taskName)) {
  auto anymalBearInterface = getAnymalBearInterface(getTaskFileFolderBear(taskName_));
  const auto mpcSettings = ocs2::mpc::loadSettings(getTaskFilePathBear(taskName_));
  const auto ddpSettings = ocs2::ddp::loadSettings(getTaskFilePathBear(taskName_));

  init(*anymalBearInterface, switched_model::getMpc(*anymalBearInterface, mpcSettings, ddpSettings));

  penalty_.reset(new ocs2::RelaxedBarrierPenalty(ddpSettings.inequalityConstraintMu_, ddpSettings.inequalityConstraintDelta_));
}

void AnymalBearPyBindings::visualizeTrajectory(const ocs2::scalar_array_t& t, const ocs2::vector_array_t& x, const ocs2::vector_array_t& u,
                                               ocs2::scalar_t speed) {
  if (!visualizer_) {
    auto anymalBearInterface = getAnymalBearInterface(taskName_);
    int fake_argc = 1;
    char arg0[] = "no_name";
    char* fake_argv[] = {arg0};
    ros::init(fake_argc, fake_argv, "anymal_visualization_node");
    ros::NodeHandle n;
    visualizer_.reset(
        new switched_model::QuadrupedVisualizer(anymalBearInterface->getKinematicModel(), anymalBearInterface->getComModel(), n));
  }

  assert(t.size() == x.size());
  std::vector<ocs2::SystemObservation> observations(t.size());

  const bool inputProvided = !u.empty();

  for (int i = 0; i < t.size(); i++) {
    observations[i].time = t[i];
    observations[i].state = x[i];
    if (inputProvided) {
      observations[i].input = u[i];
    }
  }

  visualizer_->publishTrajectory(observations, speed);
}

}  // namespace anymal
