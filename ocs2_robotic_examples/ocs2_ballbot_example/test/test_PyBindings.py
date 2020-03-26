"""
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
"""

import unittest
import numpy as np

from ocs2_ballbot_example import mpc_interface
from ocs2_ballbot_example import scalar_array, state_vector_array, state_matrix_array, input_vector_array, dynamic_vector_array, cost_desired_trajectories

class ballbot_python_tests(unittest.TestCase):
  def setUp(self):
    print("Instantiating MPC interface")
    self.mpc = mpc_interface("mpc", False)

  def test_run_mpc(self):
    print("Setting up goal")
    desiredTimeTraj = scalar_array()
    desiredTimeTraj.resize(1)
    desiredTimeTraj[0] = 2.0

    desiredInputTraj = dynamic_vector_array()
    desiredInputTraj.resize(1)
    desiredInputTraj[0] = np.zeros((self.mpc.INPUT_DIM, 1))

    desiredStateTraj = dynamic_vector_array()
    desiredStateTraj.resize(1)
    desiredStateTraj[0] = np.zeros((self.mpc.STATE_DIM, 1))

    targetTrajectories = cost_desired_trajectories(desiredTimeTraj, desiredStateTraj, desiredInputTraj)
    self.mpc.reset(targetTrajectories)


    time = 0.0
    x = np.zeros((self.mpc.STATE_DIM, 1))
    x[0] = 0.3
    x[1] = 0.5

    self.mpc.setObservation(time, x)
    self.mpc.advanceMpc()

    # instantiate c++ std arrays to store results
    t_result = scalar_array()
    x_result = state_vector_array()
    u_result = input_vector_array()

    self.mpc.getMpcSolution(t_result, x_result, u_result)

    print("MPC solution has", t_result.__len__(), "time steps")
    print("t\t\tx\t\tu")

    for t,x,u in zip(t_result, x_result, u_result):
      print(t, "\t\t", x, "\t\t", u)


    print("\n### Testing flow map and its derivative")
    dxdt = self.mpc.computeFlowMap(t_result[0], x_result[0], u_result[0])
    print("dxdt", dxdt)
    self.mpc.setFlowMapDerivativeStateAndControl(t_result[0], x_result[0], u_result[0])
    A = self.mpc.computeFlowMapDerivativeState()
    print("A", A)
    B = self.mpc.computeFlowMapDerivativeInput()
    print("B", B)

    print("\n### Testing cost and its derivative")
    L = self.mpc.getIntermediateCost(t_result[0], x_result[0], u_result[0])
    print("L", L)
    dLdx = self.mpc.getIntermediateCostDerivativeState(t_result[0], x_result[0], u_result[0])
    print("dLdx", dLdx)
    dLdu = self.mpc.getIntermediateCostDerivativeInput(t_result[0], x_result[0], u_result[0])
    print("dLdu", dLdu)

if __name__ == '__main__':
    unittest.main()
