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

#include <ocs2_comm_interfaces/ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

#include "ocs2_cart_pole_example/CartPoleInterface.h"
#include "ocs2_cart_pole_example/definitions.h"
#include "ocs2_cart_pole_example/ros_comm/MRT_ROS_Dummy_Cartpole.h"

using namespace ocs2;

int main(int argc, char** argv) {
  // task file
  if (argc <= 1) {
    throw std::runtime_error("No task file specified. Aborting.");
  }
  std::string taskFileFolderName = std::string(argv[1]);  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)

  // CartPoleInterface
  cartpole::CartPoleInterface cartPoleInterface(taskFileFolderName);

  using mrt_base_ptr_t = cartpole::MRT_ROS_Dummy_Cartpole::mrt_ptr_t;
  mrt_base_ptr_t mrtPtr(new MRT_ROS_Interface<cartpole::STATE_DIM_, cartpole::INPUT_DIM_>("cartpole"));

  // Dummy cartpole
  cartpole::MRT_ROS_Dummy_Cartpole dummyCartpole(mrtPtr, cartPoleInterface.mpcSettings().mrtDesiredFrequency_,
                                                 cartPoleInterface.mpcSettings().mpcDesiredFrequency_);

  dummyCartpole.launchNodes(argc, argv);

  // initial state
  cartpole::MRT_ROS_Dummy_Cartpole::system_observation_t initObservation;
  cartPoleInterface.getInitialState(initObservation.state());

  // initial command
  cartpole::MRT_ROS_Dummy_Cartpole::cost_desired_trajectories_t initCostDesiredTrajectories;
  initCostDesiredTrajectories.desiredTimeTrajectory().resize(1);
  initCostDesiredTrajectories.desiredTimeTrajectory().front() = 0.0;
  initCostDesiredTrajectories.desiredStateTrajectory().resize(1);
  initCostDesiredTrajectories.desiredStateTrajectory().front().setZero(cartpole::STATE_DIM_);
  initCostDesiredTrajectories.desiredInputTrajectory().resize(1);
  initCostDesiredTrajectories.desiredInputTrajectory().front().setZero(cartpole::INPUT_DIM_);

  // run dummy
  dummyCartpole.run(initObservation, initCostDesiredTrajectories);

  // Successful exit
  return 0;
}
