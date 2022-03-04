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

#pragma once

#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_python_interface/PythonInterface.h>

#include "ocs2_ballbot/BallbotInterface.h"
#include "ocs2_ballbot/definitions.h"

namespace ocs2 {
namespace ballbot {

class BallbotPyBindings final : public PythonInterface {
 public:
  /**
   * Constructor
   *
   * @note Creates directory for generated library into if it does not exist.
   * @throw Invalid argument error if input task file does not exist.
   *
   * @param [in] taskFile: The absolute path to the configuration file for the MPC.
   * @param [in] libraryFolder: The absolute path to the directory to generate CppAD library into.
   * @param [in] urdfFile: The absolute path to the URDF of the robot. This is not used for ballbot.
   */
  BallbotPyBindings(const std::string& taskFile, const std::string& libraryFolder, const std::string urdfFile = "") {
    // System dimensions
    stateDim_ = static_cast<int>(STATE_DIM);
    inputDim_ = static_cast<int>(INPUT_DIM);

    // Robot interface
    BallbotInterface ballbotInterface(taskFile, libraryFolder);

    // MPC
    std::unique_ptr<GaussNewtonDDP_MPC> mpcPtr(
        new GaussNewtonDDP_MPC(ballbotInterface.mpcSettings(), ballbotInterface.ddpSettings(), ballbotInterface.getRollout(),
                               ballbotInterface.getOptimalControlProblem(), ballbotInterface.getInitializer()));
    mpcPtr->getSolverPtr()->setReferenceManager(ballbotInterface.getReferenceManagerPtr());

    // Python interface
    PythonInterface::init(ballbotInterface, std::move(mpcPtr));
  }
};

}  // namespace ballbot
}  // namespace ocs2
