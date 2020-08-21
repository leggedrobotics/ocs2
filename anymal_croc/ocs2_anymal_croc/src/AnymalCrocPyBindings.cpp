#include "ocs2_anymal_croc/AnymalCrocPyBindings.h"

#include <ocs2_core/constraint/RelaxedBarrierPenalty.h>

#include <ocs2_quadruped_interface/QuadrupedSlqMpc.h>

#include "ocs2_anymal_croc/AnymalCrocInterface.h"

namespace anymal {

AnymalCrocPyBindings::AnymalCrocPyBindings(std::string taskName) : taskName_(std::move(taskName)) {
  auto anymalCrocInterface = getAnymalCrocInterface(getTaskFileFolderCroc(taskName_));
  const auto mpcSettings = ocs2::mpc::loadSettings(getTaskFilePathCroc(taskName_));
  const auto ddpSettings = ocs2::ddp::loadSettings(getTaskFilePathCroc(taskName_));

  init(*anymalCrocInterface, switched_model::getMpc(*anymalCrocInterface, mpcSettings, ddpSettings));

  penalty_.reset(new ocs2::RelaxedBarrierPenalty(ddpSettings.inequalityConstraintMu_, ddpSettings.inequalityConstraintDelta_));
}

void AnymalCrocPyBindings::visualizeTrajectory(const ocs2::scalar_array_t& t, const ocs2::vector_array_t& x, const ocs2::vector_array_t& u,
                                               ocs2::scalar_t speed) {
  if (!visualizer_) {
    auto anymalCrocInterface = getAnymalCrocInterface(taskName_);
    int fake_argc = 1;
    char arg0[] = "no_name";
    char* fake_argv[] = {arg0};
    ros::init(fake_argc, fake_argv, "anymal_visualization_node");
    ros::NodeHandle n;
    visualizer_.reset(
        new switched_model::QuadrupedVisualizer(anymalCrocInterface->getKinematicModel(), anymalCrocInterface->getComModel(), n));
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
