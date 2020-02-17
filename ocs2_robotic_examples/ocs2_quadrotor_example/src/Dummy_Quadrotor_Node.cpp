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

#include <ocs2_comm_interfaces/ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h>
#include <ocs2_comm_interfaces/ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

#include "ocs2_quadrotor_example/QuadrotorInterface.h"
#include "ocs2_quadrotor_example/definitions.h"
#include "ocs2_quadrotor_example/ros_comm/QuadrotorDummyVisualization.h"

using namespace ocs2;

int main(int argc, char** argv) {
  const std::string robotName = "quadrotor";
  using interface_t = ocs2::quadrotor::QuadrotorInterface;
  using vis_t = ocs2::quadrotor::QuadrotorDummyVisualization;
  using mrt_t = ocs2::MRT_ROS_Interface<ocs2::quadrotor::STATE_DIM_, ocs2::quadrotor::INPUT_DIM_>;
  using dummy_t = ocs2::MRT_ROS_Dummy_Loop<ocs2::quadrotor::STATE_DIM_, ocs2::quadrotor::INPUT_DIM_>;

  // task file
  if (argc <= 1) {
    throw std::runtime_error("No task file specified. Aborting.");
  }
  std::string taskFileFolderName = std::string(argv[1]);  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)

  // Initialize ros node
  ros::init(argc, argv, robotName + "_mrt");
  ros::NodeHandle nodeHandle;

  // Robot interface
  interface_t quadrotorInterface(taskFileFolderName);

  // MRT
  mrt_t mrt(robotName);
  mrt.initRollout(&quadrotorInterface.getRollout());
  mrt.launchNodes(nodeHandle);

  // Visualization
  std::shared_ptr<vis_t> quadrotorDummyVisualization(new vis_t());

  // Dummy loop
  dummy_t dummyQuadrotor(mrt, quadrotorInterface.mpcSettings().mrtDesiredFrequency_, quadrotorInterface.mpcSettings().mpcDesiredFrequency_);
  dummyQuadrotor.subscribeObservers({quadrotorDummyVisualization});

  // initial state
  mrt_t::system_observation_t initObservation;
  initObservation.state() = quadrotorInterface.getInitialState();

  // initial command
  ocs2::CostDesiredTrajectories initCostDesiredTrajectories({initObservation.time()}, {initObservation.state()}, {initObservation.input()});

  // Run dummy (loops while ros is ok)
  dummyQuadrotor.run(initObservation, initCostDesiredTrajectories);

  return 0;
}
