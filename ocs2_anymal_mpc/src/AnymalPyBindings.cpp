#include "ocs2_anymal_mpc/AnymalPyBindings.h"

#include <ocs2_core/constraint/RelaxedBarrierPenalty.h>

#include <ocs2_quadruped_interface/QuadrupedSlqMpc.h>

#include "ocs2_anymal_mpc/AnymalInterface.h"

namespace anymal {

AnymalPyBindings::AnymalPyBindings(std::string varargs) {
  // TODO (jcarius) : Derive this from varargs
  robotName_ = "croc";
  configName_ = "c_series";

  auto anymalInterface = getAnymalInterface(stringToAnymalModel(robotName_), getConfigFolder(configName_));
  const auto mpcSettings = ocs2::mpc::loadSettings(getTaskFilePath(configName_));
  const auto ddpSettings = ocs2::ddp::loadSettings(getTaskFilePath(configName_));

  init(*anymalInterface, switched_model::getMpc(*anymalInterface, mpcSettings, ddpSettings));

  penalty_.reset(new ocs2::RelaxedBarrierPenalty(ddpSettings.inequalityConstraintMu_, ddpSettings.inequalityConstraintDelta_));
}

void AnymalPyBindings::visualizeTrajectory(const ocs2::scalar_array_t& t, const ocs2::vector_array_t& x, const ocs2::vector_array_t& u,
                                           ocs2::scalar_t speed) {
  if (!visualizer_) {
    auto anymalInterface = getAnymalInterface(stringToAnymalModel(robotName_), getConfigFolder(configName_));
    int fake_argc = 1;
    char arg0[] = "no_name";
    char* fake_argv[] = {arg0};
    ros::init(fake_argc, fake_argv, "anymal_visualization_node");
    ros::NodeHandle n;
    visualizer_.reset(new switched_model::QuadrupedVisualizer(anymalInterface->getKinematicModel(), anymalInterface->getComModel(), n));
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
