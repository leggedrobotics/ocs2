/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#include <algorithm>
#include <stdexcept>

#include <ocs2_mpc/MPC_BASE.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MPC_BASE::MPC_BASE(mpc::Settings mpcSettings) : mpcSettings_(std::move(mpcSettings)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_BASE::reset() {
  getSolverPtr()->reset();
  initRun_ = true;
  solverHealthy_ = true;
  lastRunTime_ = 0.0;
  mpcTimer_.reset();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool MPC_BASE::run(scalar_t currentTime, const vector_t& currentState) {
  if (!solverHealthy_) {
    throw std::runtime_error("The previous MPC solve failed. Reset the MPC before running it again.");
  }

  if (!initRun_) {
    const scalar_t previousFinalTime = getSolverPtr()->getFinalTime();
    if (currentTime >= previousFinalTime || currentTime < lastRunTime_) {
      std::cerr << "WARNING: MPC observation time discontinuity detected. Resetting the solver.\n";
      std::cerr << "currentTime: " << currentTime << "\t previousRunTime: " << lastRunTime_
                << "\t previousFinalTime: " << previousFinalTime << '\n';
      reset();
    }
  }

  const scalar_t finalTime = currentTime + mpcSettings_.timeHorizon_;

  // display
  if (mpcSettings_.debugPrint_) {
    std::cerr << "\n#####################################################";
    std::cerr << "\n#####################################################";
    std::cerr << "\n#####################################################";
    std::cerr << "\n### MPC is called at time:  " << currentTime << " [s].";
    std::cerr << "\n### MPC final Time:         " << finalTime << " [s].";
    std::cerr << "\n### MPC time horizon:       " << mpcSettings_.timeHorizon_ << " [s].\n";
    mpcTimer_.startTimer();
  }

  // calculate the MPC policy
  try {
    calculateController(currentTime, currentState, finalTime);
  } catch (...) {
    solverHealthy_ = false;
    throw;
  }

  // set initRun flag to false
  initRun_ = false;
  lastRunTime_ = currentTime;

  // display
  if (mpcSettings_.debugPrint_) {
    mpcTimer_.endTimer();
    std::cerr << "\n### MPC Benchmarking";
    std::cerr << "\n###   Maximum : " << mpcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
    std::cerr << "\n###   Average : " << mpcTimer_.getAverageInMilliseconds() << "[ms].";
    std::cerr << "\n###   Latest  : " << mpcTimer_.getLastIntervalInMilliseconds() << "[ms]." << std::endl;
  }

  return true;
}

}  // namespace ocs2
