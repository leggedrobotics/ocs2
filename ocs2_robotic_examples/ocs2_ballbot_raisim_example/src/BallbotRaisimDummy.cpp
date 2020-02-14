#include <ros/package.h>

#include <ocs2_ballbot_example/BallbotInterface.h>
#include <ocs2_ballbot_example/definitions.h>
#include <ocs2_ballbot_example/ros_comm/MRT_ROS_Dummy_Ballbot.h>
#include <ocs2_ballbot_raisim_example/BallbotRaisimConversions.h>
#include <ocs2_comm_interfaces/ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>
#include <ocs2_raisim/RaisimRollout.h>
#include <ocs2_raisim_ros/RaisimHeightmapRosConverter.h>

int main(int argc, char* argv[]) {
  // task file
  if (argc <= 1) {
    throw std::runtime_error("No task tile specified.");
  }
  const std::string taskFileFolderName(argv[1]);  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)

  ros::init(argc, argv, "ballbot_raisim_dummy");

  // read urdf file
  const std::string urdfParamName = "/ocs2_ballbot_raisim_description";
  std::string urdf;
  if (!ros::param::get(urdfParamName, urdf)) {
    throw ros::Exception("Error reading urdf from parameter server: " + urdfParamName);
  }

  // setup raisim rollout
  ocs2::RaisimRolloutSettings raisimRolloutSettings(ros::package::getPath("ocs2_ballbot_raisim_example") + "/config/raisim_rollout.info",
                                                    "rollout", true);
  ocs2::ballbot::BallbotRaisimConversions conversions;
  ocs2::RaisimRollout<ocs2::ballbot::STATE_DIM_, ocs2::ballbot::INPUT_DIM_> simRollout(
      urdf,
      std::bind(&ocs2::ballbot::BallbotRaisimConversions::stateToRaisimGenCoordGenVel, &conversions, std::placeholders::_1,
                std::placeholders::_2),
      std::bind(&ocs2::ballbot::BallbotRaisimConversions::raisimGenCoordGenVelToState, &conversions, std::placeholders::_1,
                std::placeholders::_2),
      std::bind(&ocs2::ballbot::BallbotRaisimConversions::inputToRaisimGeneralizedForce, &conversions, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5),
      nullptr, raisimRolloutSettings, nullptr);

  ocs2::ballbot::BallbotInterface interface(taskFileFolderName);

  // setup MRT
  ocs2::MRT_ROS_Interface<ocs2::ballbot::STATE_DIM_, ocs2::ballbot::INPUT_DIM_> mrt("ballbot");
  mrt.initRollout(&simRollout);

  ocs2::ballbot::MRT_ROS_Dummy_Ballbot dummyBallbot(mrt, interface.mpcSettings().mrtDesiredFrequency_,
                                                    interface.mpcSettings().mpcDesiredFrequency_);

  dummyBallbot.launchNodes(argc, argv);

  // initial state and command
  ocs2::ballbot::MRT_ROS_Dummy_Ballbot::system_observation_t initObservation;
  initObservation.state() = interface.getInitialState();

  ocs2::CostDesiredTrajectories initCostDesiredTrajectories;
  initCostDesiredTrajectories.desiredTimeTrajectory().push_back(initObservation.time());
  initCostDesiredTrajectories.desiredStateTrajectory().push_back(initObservation.state());
  initCostDesiredTrajectories.desiredInputTrajectory().push_back(initObservation.input());

  // run dummy
  dummyBallbot.run(initObservation, initCostDesiredTrajectories);

  return 0;
}
