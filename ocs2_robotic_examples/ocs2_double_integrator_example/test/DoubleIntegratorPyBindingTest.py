# import ocs2_double_integrator_example # works with python2.7, should be working with python3, too

# python 3 workaround
import sys
sys.path.append("/home/jcarius/catkin_ws/devel/lib/python3.6/dist-packages/ocs2_double_integrator_example")
from DoubleIntegratorPyBindings import mpc_interface
from DoubleIntegratorPyBindings import scalar_array, state_vector_array, state_matrix_array, input_vector_array, dynamic_vector_array, cost_desired_trajectories

import numpy as np

print("Instantiating MPC interface")
mpc = mpc_interface("mpc", False)

print("Setting up goal point")
desiredTimeTraj = scalar_array()
desiredTimeTraj.resize(1)
desiredTimeTraj[0] = 2.0

desiredInputTraj = dynamic_vector_array()
desiredInputTraj.resize(1)
desiredInputTraj[0] = np.zeros((mpc.INPUT_DIM, 1))

desiredStateTraj = dynamic_vector_array()
desiredStateTraj.resize(1)
desiredStateTraj[0] = np.zeros((mpc.STATE_DIM, 1))

targetTrajectories = cost_desired_trajectories(desiredTimeTraj, desiredStateTraj, desiredInputTraj)
mpc.reset(targetTrajectories)


time = 0.0
x = np.ndarray([2, 1])
x[0] = 0.3
x[1] = 0.5

mpc.setObservation(time, x)
mpc.advanceMpc()

# instantiate c++ std arrays to store results
t_result = scalar_array()
x_result = state_vector_array()
u_result = input_vector_array()

mpc.getMpcSolution(t_result, x_result, u_result)

print("MPC solution has", t_result.__len__(), "time steps")
print("t\t\tx\t\tu")

for t,x,u in zip(t_result, x_result, u_result):
  print(t, "\t\t", x, "\t\t", u)


print("\n### Testing flow map and its derivative")
dxdt = mpc.computeFlowMap(t_result[0], x_result[0], u_result[0])
print("dxdt", dxdt)
mpc.setFlowMapDerivativeStateAndControl(t_result[0], x_result[0], u_result[0])
A = mpc.computeFlowMapDerivativeState()
print("A", A)
B = mpc.computeFlowMapDerivativeInput()
print("B", B)

print("\n### Testing cost its derivative")
L = mpc.getRunningCost(t_result[0], x_result[0], u_result[0])
print("L", L)
dLdx = mpc.getRunningCostDerivativeState(t_result[0], x_result[0], u_result[0])
print("dLdx", dLdx)
dLdu = mpc.getRunningCostDerivativeInput(t_result[0], x_result[0], u_result[0])
print("dLdu", dLdu)
