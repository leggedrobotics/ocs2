#include <ocs2_anymal_bear/AnymalBearInterface.h>
#include <ocs2_anymal_bear/AnymalBearPyBindings.h>

namespace anymal {

AnymalBearPyBindings::AnymalBearPyBindings(std::string taskFileFolder, bool async) : Base(async), taskFileFolder_(std::move(taskFileFolder)) {
  auto anymalBearInterface = getAnymalBearInterface(taskFileFolder_);
  init(*anymalBearInterface, anymalBearInterface->getMpc());

  const auto slqSettings = anymalBearInterface->slqSettings();
  penalty_.reset(new ocs2::RelaxedBarrierPenalty<switched_model::STATE_DIM, switched_model::INPUT_DIM>(
      slqSettings.ddpSettings_.inequalityConstraintMu_, slqSettings.ddpSettings_.inequalityConstraintDelta_));
}

void AnymalBearPyBindings::visualizeTrajectory(const scalar_array_t& t, const state_vector_array_t& x, const input_vector_array_t& u,
                                               double speed) {
  if (!visualizer_) {
    auto anymalBearInterface = getAnymalBearInterface(taskFileFolder_);
    int fake_argc = 1;
    auto* fake_argv = const_cast<char*>("no_name");
    ros::init(fake_argc, &fake_argv, "anymal_visualization_node");
    ros::NodeHandle n;
    visualizer_.reset(new visualizer_t(anymalBearInterface->getKinematicModel(), anymalBearInterface->getComModel(), "anymal", n));
  }

  assert(t.size() == x.size());
  visualizer_t::system_observation_array_t observations(t.size());

  const bool inputProvided = u.size() > 0;

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
