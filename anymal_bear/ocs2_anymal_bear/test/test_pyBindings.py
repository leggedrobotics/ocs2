import unittest
import numpy as np

from ocs2_anymal_bear import mpc_interface
from ocs2_anymal_bear import scalar_array, state_vector_array, state_matrix_array, input_vector_array, dynamic_vector_array, cost_desired_trajectories

class anymal_bear_python_tests(unittest.TestCase):

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
    x[3] = 0.3     # base x
    x[4] = 0.2     # base y
    x[5] = 0.495   # base z
    x[12] = -0.1   # LF_HAA
    x[13] = 0.7
    x[14] = -1.0
    x[15] = 0.1
    x[16] = 0.7
    x[17] = -1.0
    x[18] = -0.1
    x[19] = -0.7
    x[20] = 1.0
    x[21] = 0.1
    x[22] = -0.7
    x[23] = 1.0    # RH_KFE
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
