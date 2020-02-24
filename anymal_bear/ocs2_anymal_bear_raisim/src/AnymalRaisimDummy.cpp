#include <ros/package.h>
#include <ros/param.h>

#include <ocs2_mpc/MpcSettings.h>

#include <ocs2_quadruped_interface/QuadrupedDummyNode.h>

#include <ocs2_anymal_bear/AnymalBearInterface.h>

#include <ocs2_raisim/RaisimRollout.h>

#include <ocs2_anymal_bear_raisim/AnymalRaisimConversions.h>

int main(int argc, char* argv[]) {
  if (argc <= 1) {
    throw std::runtime_error("No task file specified. Aborting.");
  }
  const std::string taskName(argv[1]);  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic))

  // Initialize ros node
  ros::init(argc, argv, "anymal_bear_mrt");
  ros::NodeHandle nodeHandle;

  // Read URDF
  const std::string urdf_param = "/ocs2_anymal_description";
  std::string urdf;
  if (!ros::param::get(urdf_param, urdf)) {
    throw ros::Exception("Error reading ros parameter: " + urdf_param);
  }

  auto anymalInterface = anymal::getAnymalBearInterface(taskName);

  // Set up Raisim rollout
  anymal::AnymalRaisimConversions conversions(anymalInterface->getComModel(), anymalInterface->getKinematicModel());
  const auto raisimRolloutSettings =
      ocs2::loadRaisimRolloutSettings(ros::package::getPath("ocs2_anymal_bear_raisim") + "/config/raisim_rollout.info", "rollout");

  ocs2::RaisimRollout<switched_model::STATE_DIM, switched_model::INPUT_DIM> simRollout(
      urdf,
      std::bind(&anymal::AnymalRaisimConversions::stateToRaisimGenCoordGenVel, &conversions, std::placeholders::_1, std::placeholders::_2),
      std::bind(&anymal::AnymalRaisimConversions::raisimGenCoordGenVelToState, &conversions, std::placeholders::_1, std::placeholders::_2),
      std::bind(&anymal::AnymalRaisimConversions::inputToRaisimGeneralizedForce, &conversions, std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3, std::placeholders::_4, std::placeholders::_5),
      std::bind(&anymal::AnymalRaisimConversions::extractModelData, &conversions, std::placeholders::_1, std::placeholders::_2),
      raisimRolloutSettings,
      std::bind(&anymal::AnymalRaisimConversions::inputToRaisimPdTargets, std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3, std::placeholders::_4, std::placeholders::_5));

  // Run dummy loop
  const auto mpcSettings = ocs2::loadMpcSettings(anymal::getTaskFilePathBear(taskName));
  quadrupedDummyNode(nodeHandle, *anymalInterface, &simRollout, mpcSettings.mrtDesiredFrequency_, mpcSettings.mpcDesiredFrequency_);

  return 0;
}
