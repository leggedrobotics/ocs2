import unittest
import numpy as np
import os

import rospkg

from ocs2_double_integrator import mpc_interface
from ocs2_double_integrator import (
    scalar_array,
    vector_array,
    matrix_array,
    TargetTrajectories,
)


class double_integrator_python_tests(unittest.TestCase):
    def setUp(self):
        packageDir = rospkg.RosPack().get_path('ocs2_double_integrator')
        taskFile = os.path.join(packageDir, 'config/mpc/task.info')
        libFolder = os.path.join(packageDir, 'auto_generated')
        print("Instantiating MPC interface")
        self.mpc = mpc_interface(taskFile, libFolder)
        self.stateDim = 2
        self.inputDim = 1

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
        x = np.zeros(2)
        x[0] = 0.3
        x[1] = 0.5
        u = np.zeros(1)

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
