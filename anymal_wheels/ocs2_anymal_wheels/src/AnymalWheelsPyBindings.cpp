/*
 * AnymalWheelsPyBindings.cpp
 *
 *  Created on: Nov 27, 2019
 *      Author: Marko Bjelonic
 */

#include <ocs2_anymal_wheels/AnymalWheelsPyBindings.h>

namespace anymal {

AnymalWheelsPyBindings::AnymalWheelsPyBindings(const std::string& taskFileFolder, bool async) : Base(async), taskFileFolder_(taskFileFolder) {
  init(taskFileFolder);
}

void AnymalWheelsPyBindings::initRobotInterface(const std::string& taskFileFolder) {
  robotInterface_.reset(new AnymalWheelsInterface(taskFileFolder));

  const auto& slqSettings = dynamic_cast<AnymalWheelsInterface*>(robotInterface_.get())->slqSettings();

  penalty_.reset(new ocs2::RelaxedBarrierPenalty<28, 28>(slqSettings.ddpSettings_.inequalityConstraintMu_,
                                                         slqSettings.ddpSettings_.inequalityConstraintDelta_));
}

void AnymalWheelsPyBindings::visualizeTrajectory(const scalar_array_t& t, const state_vector_array_t& x, const input_vector_array_t& u,
                                           double speed) {
  if (!visualizer_) {
    std::shared_ptr<AnymalWheelsInterface> interface_for_visualizer(new AnymalWheelsInterface(taskFileFolder_));
    visualizer_.reset(new visualizer_t(interface_for_visualizer, "anymal", false));

    int fake_argc = 1;
    auto* fake_argv = const_cast<char*>("no_name");
    visualizer_->launchVisualizerNode(fake_argc, &fake_argv);
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
