"""
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
"""

import unittest
import numpy as np
import os

import rospkg

from ocs2_ballbot import mpc_interface
from ocs2_ballbot import (
    scalar_array,
    vector_array,
    matrix_array,
    TargetTrajectories,
)


class ballbot_python_tests(unittest.TestCase):
    def setUp(self):
        packageDir = rospkg.RosPack().get_path('ocs2_ballbot')
        taskFile = os.path.join(packageDir, 'config/mpc/task.info')
        libFolder = os.path.join(packageDir, 'auto_generated')
        print("Instantiating MPC interface")
        self.mpc = mpc_interface(taskFile, libFolder)
        self.stateDim = 10
        self.inputDim = 3

    def test_run_mpc(self):
        print("Setting up goal")
        desiredTimeTraj = scalar_array()
        desiredTimeTraj.push_back(2.0)

        desiredInputTraj = vector_array()
        desiredInputTraj.push_back(np.zeros(self.inputDim))

        desiredStateTraj = vector_array()
        desiredStateTraj.push_back(np.zeros(self.stateDim))

        targetTrajectories = TargetTrajectories(
            desiredTimeTraj, desiredStateTraj, desiredInputTraj
        )
        self.mpc.reset(targetTrajectories)

        time = 0.0
        x = np.zeros(self.stateDim)
        x[0] = 0.3
        x[1] = 0.5
        u = np.zeros(self.inputDim)

        self.mpc.setObservation(time, x, u)
        self.mpc.advanceMpc()

        # instantiate c++ std arrays to store results
        t_result = scalar_array()
        x_result = vector_array()
        u_result = vector_array()

        self.mpc.getMpcSolution(t_result, x_result, u_result)

        print("MPC solution has", t_result.__len__(), "time steps")
        print("t\t\tx\t\tu")

        for t, x, u in zip(t_result, x_result, u_result):
            print("{:.4f}, \t\t{}, \t\t {}".format(t, x, u))

        print("\n### Testing flow map and its derivative")
        flowMap = self.mpc.flowMapLinearApproximation(
            t_result[0], x_result[0], u_result[0]
        )
        print("dxdt", flowMap.f)
        print("A", flowMap.dfdx)
        print("B", flowMap.dfdu)

        print("\n### Testing cost and its derivative")
        L = self.mpc.costQuadraticApproximation(t_result[0], x_result[0], u_result[0])
        print("L", L.f)
        print("dLdx", L.dfdx)
        print("dLdu", L.dfdu)


if __name__ == "__main__":
    unittest.main()
