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

#include "ocs2_double_slit_example/DoubleSlitInterface.h"
#include "ocs2_double_slit_example/definitions.h"
#include "ocs2_double_slit_example/ros_comm/MRT_ROS_Double_Slit.h"
#include "ocs2_double_slit_example/ros_comm/MRT_ROS_Dummy_Double_Slit.h"

using namespace ocs2;

int main(int argc, char* argv[]) {
  // task file
  if (argc <= 1) {
    throw std::runtime_error("No task file specified. Aborting.");
  }
  std::string taskFileFolderName(argv[1]);  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)

  // doubleSlitInterface
  double_slit::DoubleSlitInterface doubleSlitInterface(taskFileFolderName);

  using mrt_base_ptr_t = double_slit::MrtRosDummyDoubleSlit::mrt_ptr_t;

  double rollout_dt;
  loadData::loadCppDataType(doubleSlitInterface.taskFile_, "pathIntegral.rollout_settings.minTimeStep", rollout_dt);

  // Dummy double_slit
  double_slit::MrtRosDoubleSlit mrt("double_slit");
  double_slit::MrtRosDummyDoubleSlit dummyDoubleSlit(
      mrt, doubleSlitInterface.mpcSettings().mrtDesiredFrequency_, doubleSlitInterface.mpcSettings().mpcDesiredFrequency_,
      &doubleSlitInterface.getDynamics(), Rollout_Settings(1e-9, 1e-6, 5000, rollout_dt, IntegratorType::EULER, false, true));

  dummyDoubleSlit.launchNodes(argc, argv);

  // Run dummy
  double_slit::MrtRosDummyDoubleSlit::system_observation_t initObservation;
  doubleSlitInterface.getInitialState(initObservation.state());

  double_slit::MrtRosDummyDoubleSlit::cost_desired_trajectories_t initCostDesiredTraj;
  initCostDesiredTraj.desiredTimeTrajectory().push_back(0.0);
  initCostDesiredTraj.desiredInputTrajectory().push_back(double_slit::DoubleSlitInterface::input_vector_t::Zero());
  initCostDesiredTraj.desiredStateTrajectory().push_back(double_slit::DoubleSlitInterface::state_vector_t::Zero());

  dummyDoubleSlit.run(initObservation, initCostDesiredTraj);

  // Successful exit
  return 0;
}
