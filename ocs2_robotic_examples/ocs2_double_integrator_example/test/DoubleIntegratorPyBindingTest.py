# import ocs2_double_integrator_example # works with python2.7, should be working with python3, too

# python 3 workaround
import sys
sys.path.append("/home/jcarius/catkin_ws/devel/lib/python3.6/dist-packages/ocs2_double_integrator_example")
from DoubleIntegratorPyBindings import mpc_interface, scalar_array, state_vector_array, input_vector_array_t

import numpy as np

mpc = mpc_interface("mpc")


time = 0.0
x = np.ndarray([2, 1])
x[0] = 0.3
x[1] = 0.5

mpc.setObservation(time, x)

mpc.advanceMpc()

# instantiate c++ std arrays to store results
t_result = scalar_array()
x_result = state_vector_array()
u_result = input_vector_array_t()
Vx_result = state_vector_array()

mpc.getMpcSolution(t_result, x_result, u_result, Vx_result)

print("MPC solution has", t_result.__len__(), "time steps")
print("t\t\tx\t\tu")

for t,x,u,Vx in zip(t_result, x_result, u_result,Vx_result):
  print(t, "\t\t", x, "\t\t", u, "\t\t", Vx)


print("testing flow map and its derivative")
dxdt = mpc.computeFlowMap(t_result[0], x_result[0], u_result[0])
print("dxdt", dxdt)
mpc.setFlowMapDerivativeStateAndControl(t_result[0], x_result[0], u_result[0])
A = mpc.computeFlowMapDerivativeState()
print("A", A)
B = mpc.computeFlowMapDerivativeInput()
print("B", B)


L = mpc.getRunningCost(t_result[0], x_result[0], u_result[0])
print("L", L)
dLdx = mpc.getRunningCostDerivativeState(t_result[0], x_result[0], u_result[0])
print("dLdx", dLdx)
dLdu = mpc.getRunningCostDerivativeInput(t_result[0], x_result[0], u_result[0])
print("dLdu", dLdu)
