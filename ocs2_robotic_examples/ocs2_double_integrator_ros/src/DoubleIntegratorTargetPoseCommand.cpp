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

#include <ocs2_double_integrator/definitions.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesKeyboardPublisher.h>

using namespace ocs2;
using namespace double_integrator;

/**
 * Converts command line to TargetTrajectories.
 * @param [in] commadLineTarget : [deltaX]
 * @param [in] observation : the current observation
 */
TargetTrajectories commandLineToTargetTrajectories(const vector_t& commadLineTarget, const SystemObservation& observation) {
  const vector_t displacement = (vector_t(STATE_DIM) << observation.state(0) + commadLineTarget(0), 0.0).finished();
  return TargetTrajectories({observation.time}, {displacement}, {vector_t::Zero(INPUT_DIM)});
}

int main(int argc, char* argv[]) {
  const std::string robotName = "double_integrator";
  ::ros::init(argc, argv, robotName + "_target");
  ::ros::NodeHandle nodeHandle;

  const scalar_array_t goalLimit{10.0};  // [deltaX]
  TargetTrajectoriesKeyboardPublisher targetPoseCommand(nodeHandle, robotName, goalLimit, &commandLineToTargetTrajectories);

  const std::string commadMsg = "Enter the desired displacement for the double-integrator";
  targetPoseCommand.publishKeyboardCommand(commadMsg);

  // Successful exit
  return 0;
}
