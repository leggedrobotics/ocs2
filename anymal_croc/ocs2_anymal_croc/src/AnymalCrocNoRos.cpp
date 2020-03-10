//
// Created by rgrandia on 05.03.20.
//

#include <ros/ros.h>

#include <ocs2_comm_interfaces/ocs2_interfaces/MPC_MRT_Interface.h>

#include <ocs2_quadruped_interface/QuadrupedSlqMpc.h>
#include <ocs2_quadruped_interface/QuadrupedVisualizer.h>

#include "ocs2_anymal_croc/AnymalCrocInterface.h"

// MPC messages
#include <ocs2_msgs/mode_sequence.h>
#include <ocs2_msgs/mpc_flattened_controller.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_msgs/mpc_target_trajectories.h>
#include <ocs2_msgs/reset.h>

#include "ocs2_comm_interfaces/ocs2_ros_interfaces/common/RosMsgConversions.h"

template <typename Functor>
void executeAtRate(Functor f, double frequency) {
  const std::chrono::duration<double> dt(1. / frequency);
  auto start = std::chrono::high_resolution_clock::now();
  f();
  auto end = std::chrono::high_resolution_clock::now();
  std::this_thread::sleep_for(dt - (end - start));
}

int main(int argc, char* argv[]) {
  if (argc <= 1) {
    throw std::runtime_error("No task file specified. Aborting.");
  }
  const std::string taskName(argv[1]);  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic))
  using mpc_mrt_t = ocs2::MPC_MRT_Interface<switched_model::STATE_DIM, switched_model::INPUT_DIM>;

  // Initialize ros node
  ros::init(argc, argv, "anymal_croc_mpc_no_ros");
  ros::NodeHandle nodeHandle;

  auto anymalInterface = anymal::getAnymalCrocInterface(taskName);
  ocs2::MPC_Settings mpcSettings;
  mpcSettings.loadSettings(anymal::getTaskFilePathCroc(taskName));
  ocs2::SLQ_Settings slqSettings;
  slqSettings.loadSettings(anymal::getTaskFilePathCroc(taskName));

  mpcSettings.debugPrint_ = true;
  auto mpcPtr = getMpc(*anymalInterface, mpcSettings, slqSettings);
  mpc_mrt_t mpcInterface(*mpcPtr);

  // initial state
  mpc_mrt_t::system_observation_t initObservation;
  initObservation.state() = anymalInterface->getInitialState();
  initObservation.subsystem() = switched_model::ModeNumber::STANCE;

  // initial command
  ocs2::CostDesiredTrajectories initCostDesiredTrajectories({0.0}, {initObservation.state()}, {initObservation.input()});

  // Visualization
  using vis_t = switched_model::QuadrupedVisualizer;
  auto visualizer = std::make_shared<vis_t>(anymalInterface->getKinematicModel(), anymalInterface->getComModel(), nodeHandle);

  mpcInterface.setCurrentObservation(initObservation);
  mpcInterface.setTargetTrajectories(initCostDesiredTrajectories);

  //  std::atomic_bool running(true);
  //  double trackingIncrement = 1.0 / 10.0;

  //  // Mpc loop
  //  auto mpcTask = [&]() {
  //    while (running) {
  //      {
  //        mpcInterface.advanceMpc();
  //      }
  //      usleep(uint(trackingIncrement * 1e6));
  //    }
  //    std::cout << "Mpc thread exit " << std::endl;
  //  };
  //  std::thread mpcThread(mpcTask);

  // Gait subsriber
  std::atomic_bool everReceived(false);
  auto interfacePtr = &mpcInterface;  // can be passed by value to the callback
  boost::function<void(const ocs2_msgs::mode_sequence::ConstPtr&)> modeCallback = [&](const ocs2_msgs::mode_sequence::ConstPtr& msg) {
    std::cout << " ========== Mode sequence received ================ " << std::endl;
    ocs2::ModeSequenceTemplate<double> modeSequenceTemplate;
    ocs2::RosMsgConversions<switched_model::STATE_DIM, switched_model::INPUT_DIM>::readModeSequenceTemplateMsg(*msg, modeSequenceTemplate);
    interfacePtr->setModeSequence(modeSequenceTemplate);
    everReceived = true;
  };
  std::string robotName_ = "anymal";
  ros::Subscriber mpcModeSequenceSubscriber_ = nodeHandle.subscribe(robotName_ + "_mpc_mode_sequence", 1, modeCallback);

  double MpcFrequency = 20.0;
  double MrtFrequency = 100.0;
  int ratio = MrtFrequency / MpcFrequency;

  // Tracking loop
  auto observation = initObservation;
  auto trackingLoop = [&]() {
    if (mpcInterface.initialPolicyReceived()) {
      mpcInterface.updatePolicy();
      observation.time() += 1. / MrtFrequency;
      auto currentState = observation.state();
      mpcInterface.evaluatePolicy(observation.time(), currentState, observation.state(), observation.input(), observation.subsystem());
      visualizer->update(observation, mpcInterface.getPolicy(), mpcInterface.getCommand());
      mpcInterface.setCurrentObservation(observation);
      ros::spinOnce();
    }
  };

  while (ros::ok()) {
    executeAtRate(
        [&]() {
          mpcInterface.advanceMpc();
          for (int i = 0; i < ratio; i++) {
            trackingLoop();
          }
        },
        MpcFrequency);
  }

  //  ros::WallRate rate(30.0);
  //  while (ros::ok()) {
  //    if (!everReceived) {
  //      std::cout << " ========== never received ================ " << std::endl;
  //    }
  //
  //    if (mpcInterface.initialPolicyReceived()) {
  //      initObservation.time() += 0.01;
  //      mpcInterface.setCurrentObservation(initObservation);
  //      mpcInterface.updatePolicy();
  //      visualizer->update(initObservation, mpcInterface.getPolicy(), mpcInterface.getCommand());
  //    }
  //    rate.sleep();
  //    ros::spinOnce();
  //  }
  //
  //  running = false;
  //  mpcThread.join();

  return 0;
}