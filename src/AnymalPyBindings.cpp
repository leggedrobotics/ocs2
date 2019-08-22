#include <ocs2_anymal_interface/AnymalPyBindings.h>

namespace anymal {

AnymalPyBindings::AnymalPyBindings(const std::string& taskFileFolder, bool async) : Base(async) { init(taskFileFolder); }

void AnymalPyBindings::initRobotInterface(const std::string& taskFileFolder) {
  robotInterface_.reset(new OCS2AnymalInterface(taskFileFolder));

  std::shared_ptr<OCS2AnymalInterface> interface_for_visualizer(new OCS2AnymalInterface(taskFileFolder));
  visualizer_.reset(new visualizer_t(interface_for_visualizer, "anymal", false));

  int fake_argc = 1;
  auto* fake_argv = const_cast<char*>("no_name");
  visualizer_->launchVisualizerNode(fake_argc, &fake_argv);
}

void AnymalPyBindings::visualizeTrajectory(const scalar_array_t& t, const state_vector_array_t& x, const input_vector_array_t& u,
                                           double speed) {
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
