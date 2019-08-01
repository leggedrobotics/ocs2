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

#include <unistd.h>
#include <thread>
#include "ocs2_comm_interfaces/ocs2_interfaces/MPC_Interface.h"
#include "ocs2_double_integrator_example/DoubleIntegratorInterface.h"

using namespace ocs2;
using namespace double_integrator;
using dim_t = ocs2::Dimensions<STATE_DIM_, INPUT_DIM_>;
typedef MPC_Interface<dim_t::STATE_DIM_, dim_t::INPUT_DIM_> mpc_t;

int main(int argc, char** argv) {
  // task file
  if (argc <= 1) throw std::runtime_error("No task file specified. Aborting.");
  std::string taskFileFolderName = std::string(argv[1]);

  DoubleIntegratorInterface doubleIntegratorInterface(taskFileFolderName);
  mpc_t mpcInterface(*doubleIntegratorInterface.getMPCPtr());

  double time = 0;

  mpc_t::state_vector_t initialState;
  doubleIntegratorInterface.getInitialState(initialState);

  // initialize reference:
  mpc_t::cost_desired_trajectories_t costDesiredTrajectories;
  costDesiredTrajectories.desiredTimeTrajectory().push_back(time);
  costDesiredTrajectories.desiredTimeTrajectory().push_back(time + 1);
  mpc_t::state_vector_t goalState = doubleIntegratorInterface.getXFinal();
  costDesiredTrajectories.desiredStateTrajectory().push_back(initialState);
  costDesiredTrajectories.desiredStateTrajectory().push_back(goalState);
  mpc_t::input_vector_t desiredInput;
  costDesiredTrajectories.desiredInputTrajectory().push_back(desiredInput);
  costDesiredTrajectories.desiredInputTrajectory().push_back(desiredInput);
  mpcInterface.setTargetTrajectories(costDesiredTrajectories);

  double f_mpc = 10;
  double mpcIncrement = 1.0 / f_mpc;
  double f_tracking = 100;
  double trackingIncrement = 1.0 / f_tracking;
  // double f_control = doubleIntegratorInterface.mpcSettings().mpcDesiredFrequency_;
  double T = 3;

  mpc_t::system_observation_t observation;
  mpc_t::state_vector_t optimalState = initialState;
  mpc_t::input_vector_t optimalInput;
  size_t subsystem;
  std::atomic_bool trackerRunning(true);

  std::mutex timeStateMutex;

  auto tracker = [&]() {
    while (trackerRunning) {
      {
        std::lock_guard<std::mutex> lock(timeStateMutex);
        time += trackingIncrement;
        if (mpcInterface.policyReceived()) {
          // TODO(johannes) Hacky, we call evaluatePolicy twice to retrieve the optimal state
          mpcInterface.evaluatePolicy(time, mpc_t::state_vector_t::Zero(), optimalState, optimalInput, subsystem);
          mpcInterface.evaluatePolicy(time, optimalState, optimalState, optimalInput, subsystem);
          std::cout << std::endl
                    << "time:" << time << "  state:" << optimalState.transpose() << "  input:" << optimalInput.transpose() << std::endl
                    << std::endl;
        }
      }
      usleep(uint(trackingIncrement * 1e6));
    }
  };

  std::thread trackerThread(tracker);

  // run MPC for N iterations
  int N = int(f_mpc * T);
  for (int i = 0; i < N; i++) {
    {
      std::lock_guard<std::mutex> lock(timeStateMutex);
      // use optimal state for the next observation:
      observation.state() = optimalState;
      observation.time() = time;
    }

    mpcInterface.setCurrentObservation(observation);

    std::cout << std::endl << "Start MPC iteration..." << std::endl;
    mpcInterface.advanceMpc();
    std::cout << "... finish MPC iteration." << std::endl;

    usleep(uint(mpcIncrement * 1e6));
  }
  trackerRunning = false;
  trackerThread.join();
  // Successful exit
  return 0;
}

/* EOF */
