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

#include <gtest/gtest.h>

#include <ocs2_ballbot/BallbotPyBindings.h>
#include <ocs2_ballbot/package_path.h>

TEST(Ballbot, PyBindings) {
  // create binding interface
  const std::string taskFile = ocs2::ballbot::getPath() + "/config/mpc/task.info";
  const std::string libFolder = ocs2::ballbot::getPath() + "/auto_generated";
  ocs2::ballbot::BallbotPyBindings bindings(taskFile, libFolder);

  ocs2::vector_t initState = ocs2::vector_t::Zero(ocs2::ballbot::STATE_DIM);
  initState(0) = 0.0;
  initState(2) = 0.0;
  const ocs2::vector_t zeroInput = ocs2::vector_t::Zero(ocs2::ballbot::INPUT_DIM);

  ocs2::TargetTrajectories targetTrajectories;
  targetTrajectories.timeTrajectory.resize(2, 0.0);
  targetTrajectories.timeTrajectory[1] = 2.0;
  targetTrajectories.inputTrajectory.resize(2, ocs2::vector_t::Zero(ocs2::ballbot::INPUT_DIM));
  targetTrajectories.stateTrajectory.resize(2, ocs2::vector_t::Zero(ocs2::ballbot::STATE_DIM));
  targetTrajectories.stateTrajectory[0] = initState;

  bindings.reset(targetTrajectories);
  bindings.setObservation(0.0, initState, zeroInput);
  bindings.advanceMpc();

  ocs2::scalar_array_t t_arr;
  ocs2::vector_array_t x_arr;
  ocs2::vector_array_t u_arr;

  bindings.getMpcSolution(t_arr, x_arr, u_arr);

  EXPECT_EQ(t_arr.size(), x_arr.size());
  EXPECT_EQ(t_arr.size(), u_arr.size());

  std::cout << "t\t\tx\t\tu" << std::endl;
  for (size_t i = 0; i < t_arr.size(); i++) {
    std::cout << std::setprecision(4);
    std::cout << t_arr[i] << "\t\t" << x_arr[i].transpose() << "\t\t" << u_arr[i].transpose() << std::endl;
  }

  const auto dxdt = bindings.flowMap(t_arr[0], x_arr[0], u_arr[0]);
  std::cout << "dxdt: " << dxdt.transpose() << std::endl;

  const auto flowMap = bindings.flowMapLinearApproximation(t_arr[0], x_arr[0], u_arr[0]);
  std::cout << "A\n" << flowMap.dfdx << "\nB\n" << flowMap.dfdu << std::endl;

  const auto L = bindings.costQuadraticApproximation(t_arr[0], x_arr[0], u_arr[0]);
  std::cout << "L: " << L.f << "\ndLdx: " << L.dfdx.transpose() << "\ndLdu: " << L.dfdu.transpose() << std::endl;

  bindings.reset(targetTrajectories);
  bindings.setObservation(0.0, initState, zeroInput);
  bindings.advanceMpc();
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
