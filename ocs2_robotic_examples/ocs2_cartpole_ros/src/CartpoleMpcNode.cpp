/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#include <ocs2_cartpole/CartPoleInterface.h>
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>
#include <ocs2_ros_interfaces/synchronized_module/SolverObserverRosCallbacks.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <memory>

#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
  const std::string robotName = "cartpole";

  // task file
  std::vector<std::string> programArgs =
      rclcpp::remove_ros_arguments(argc, argv);

  if (programArgs.size() <= 1) {
    throw std::runtime_error("No task file specified. Aborting.");
  }
  std::string taskFileFolderName(programArgs[1]);

  // Initialize ros node
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared(robotName + "_mpc");

  // Robot interface
  const std::string taskFile =
      ament_index_cpp::get_package_share_directory("ocs2_cartpole") +
      "/config/" + taskFileFolderName + "/task.info";
  const std::string libFolder =
      ament_index_cpp::get_package_share_directory("ocs2_cartpole") +
      "/auto_generated";
  ocs2::cartpole::CartPoleInterface cartPoleInterface(taskFile, libFolder,
                                                      true /*verbose*/);

  // MPC
  ocs2::GaussNewtonDDP_MPC mpc(cartPoleInterface.mpcSettings(),
                               cartPoleInterface.ddpSettings(),
                               cartPoleInterface.getRollout(),
                               cartPoleInterface.getOptimalControlProblem(),
                               cartPoleInterface.getInitializer());

  // observer for the input limits constraints
  auto createStateInputBoundsObserver = [&]() {
    const std::string observingLagrangianTerm = "InputLimits";
    const ocs2::scalar_array_t observingTimePoints{0.0, 0.5};
    std::vector<std::string> metricsTopicNames;
    std::vector<std::string> multiplierTopicNames;
    for (const auto& t : observingTimePoints) {
      const int timeMs = static_cast<int>(t * 1000.0);
      metricsTopicNames.push_back("metrics/" + observingLagrangianTerm + "/" +
                                  "MsLookAhead" + std::to_string(timeMs));
      multiplierTopicNames.push_back("multipliers/" + observingLagrangianTerm +
                                     "/" + "MsLookAhead" +
                                     std::to_string(timeMs));
    }
    auto lagrangianCallback = ocs2::ros::createLagrangianCallback(
        node, observingTimePoints, metricsTopicNames,
        ocs2::ros::CallbackInterpolationStrategy::linear_interpolation);
    auto multiplierCallback = ocs2::ros::createMultiplierCallback(
        node, observingTimePoints, multiplierTopicNames,
        ocs2::ros::CallbackInterpolationStrategy::linear_interpolation);
    return ocs2::SolverObserver::LagrangianTermObserver(
        ocs2::SolverObserver::Type::Intermediate, observingLagrangianTerm,
        std::move(lagrangianCallback), std::move(multiplierCallback));
  };
  mpc.getSolverPtr()->addSolverObserver(createStateInputBoundsObserver());

  // Launch MPC ROS node
  ocs2::MPC_ROS_Interface mpcNode(mpc, robotName);
  mpcNode.launchNodes(node);

  return 0;
}
