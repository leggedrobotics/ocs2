import unittest
import numpy as np

from ocs2_double_integrator_example import mpc_interface
from ocs2_double_integrator_example import scalar_array, state_vector_array, state_matrix_array, input_vector_array, dynamic_vector_array, cost_desired_trajectories

"""
# Python 3 import workaround (while ROS is still using python 2.7)
import sys
sys.path.append("/path/to/catkin_ws/devel/lib/python3.6/dist-packages/ocs2_double_integrator_example")
from DoubleIntegratorPyBindings import mpc_interface
from DoubleIntegratorPyBindings import scalar_array, state_vector_array, state_matrix_array, input_vector_array, dynamic_vector_array, cost_desired_trajectories
"""

class double_integrator_python_tests(unittest.TestCase):
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
    x = np.ndarray([2, 1])
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

    print("\n### Testing cost its derivative")
    L = self.mpc.getIntermediateCost(t_result[0], x_result[0], u_result[0])
    print("L", L)
    dLdx = self.mpc.getIntermediateCostDerivativeState(t_result[0], x_result[0], u_result[0])
    print("dLdx", dLdx)
    dLdu = self.mpc.getIntermediateCostDerivativeInput(t_result[0], x_result[0], u_result[0])
    print("dLdu", dLdu)

if __name__ == '__main__':
    unittest.main()
