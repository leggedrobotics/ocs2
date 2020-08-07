#include <ocs2_anymal_croc/AnymalCrocInterface.h>
#include <ocs2_anymal_croc/AnymalCrocPyBindings.h>

#include <ocs2_quadruped_interface/QuadrupedSlqMpc.h>

namespace anymal {

AnymalCrocPyBindings::AnymalCrocPyBindings(std::string taskName, bool async) : Base(async), taskName_(std::move(taskName)) {
  auto anymalCrocInterface = getAnymalCrocInterface(getTaskFileFolderCroc(taskName_));
  ocs2::mpc::Settings mpcSettings = ocs2::mpc::loadSettings(getTaskFilePathCroc(taskName_));
  ocs2::ddp::Settings ddpSettings = ocs2::ddp::loadSettings(getTaskFilePathCroc(taskName_));

  init(*anymalCrocInterface, switched_model::getMpc(*anymalCrocInterface, mpcSettings, ddpSettings));

  penalty_.reset(new ocs2::RelaxedBarrierPenalty<switched_model::STATE_DIM, switched_model::INPUT_DIM>(
      ddpSettings.ddpSettings_.inequalityConstraintMu_, ddpSettings.ddpSettings_.inequalityConstraintDelta_));
}

void AnymalCrocPyBindings::visualizeTrajectory(const scalar_array_t& t, const state_vector_array_t& x, const input_vector_array_t& u,
                                               double speed) {
  if (!visualizer_) {
    auto anymalCrocInterface = getAnymalCrocInterface(taskName_);
    int fake_argc = 1;
    char arg0[] = "no_name";
    char* fake_argv[] = {arg0};
    ros::init(fake_argc, fake_argv, "anymal_visualization_node");
    ros::NodeHandle n;
    visualizer_.reset(new visualizer_t(anymalCrocInterface->getKinematicModel(), anymalCrocInterface->getComModel(), n));
  }

  assert(t.size() == x.size());
  visualizer_t::system_observation_array_t observations(t.size());

  const bool inputProvided = !u.empty();

  for (int i = 0; i < t.size(); i++) {
    observations[i].time() = t[i];
    observations[i].state() = x[i];
    if (inputProvided) {
      observations[i].input() = u[i];
    }
  }

  visualizer_->publishTrajectory(observations, speed);
}

}  // namespace anymal
