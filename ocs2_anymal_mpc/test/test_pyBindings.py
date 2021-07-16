import unittest
import numpy as np

from ocs2_anymal_mpc import mpc_interface
from ocs2_anymal_mpc import (
    scalar_array,
    vector_array,
    matrix_array,
    TargetTrajectories,
)


class anymal_mpc_python_tests(unittest.TestCase):
    def setUp(self):
        print("Instantiating MPC interface")
        self.mpc = mpc_interface("--name croc --config c_series")
        self.STATE_DIM = 24
        self.INPUT_DIM = 24

    def test_run_mpc(self):
        print("Setting up goal")

        desiredTimeTraj = scalar_array()
        desiredTimeTraj.push_back(2.0)

        desiredInputTraj = vector_array()
        desiredInputTraj.push_back(np.zeros(self.INPUT_DIM))

        desiredStateTraj = vector_array()
        desiredStateTraj.push_back(np.zeros(self.STATE_DIM))

        targetTrajectories = TargetTrajectories(
            desiredTimeTraj, desiredStateTraj, desiredInputTraj
        )
        self.mpc.reset(targetTrajectories)

        time = 0.0
        x = np.zeros(self.STATE_DIM)
        x[3] = 0.1  # base x
        x[4] = 0.1  # base y
        x[5] = 0.5163  # base z
        x[12] = -0.25  # LF_HAA
        x[13] = 0.6
        x[14] = -0.85
        x[15] = 0.25
        x[16] = 0.6
        x[17] = -0.85
        x[18] = -0.25
        x[19] = -0.6
        x[20] = 0.85
        x[21] = 0.25
        x[22] = -0.6
        x[23] = 0.85  # RH_KFE
        self.mpc.setObservation(time, x, np.zeros(self.INPUT_DIM))

        self.mpc.advanceMpc()

        # instantiate c++ std arrays to store results
        t_result = scalar_array()
        x_result = vector_array()
        u_result = vector_array()

        self.mpc.getMpcSolution(t_result, x_result, u_result)

        print("MPC solution has", t_result.__len__(), "time steps")
        print("t\t\tx\t\t\tu")

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
