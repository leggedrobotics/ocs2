#include <gtest/gtest.h>

#include <ocs2_anymal_interface/AnymalPyBindings.h>

TEST(Anymal, PyBindings) {
  using bindings_t = anymal::AnymalPyBindings;
  using state_vector_t = bindings_t::state_vector_t;
  using input_vector_t = bindings_t::input_vector_t;
  using state_matrix_array_t = bindings_t::state_matrix_array_t;
  using scalar_array_t = bindings_t::scalar_array_t;
  using state_vector_array_t = bindings_t::state_vector_array_t;
  using input_vector_array_t = bindings_t::input_vector_array_t;
  using input_state_matrix_array_t = bindings_t::input_state_matrix_array_t;
  using cost_desired_trajectories_t = bindings_t::cost_desired_trajectories_t;

  bindings_t bindings("/home/jcarius/catkin_ws/src/ocs2_anymal_interface/config/mpc", false);

  state_vector_t initState = state_vector_t::Zero();
  initState(5) = 0.495;  // base z
  initState(12) = -0.1;   // LF_HAA
  initState(13) = 0.7;
  initState(14) = -1.0;
  initState(15) = 0.1;
  initState(16) = 0.7;
  initState(17) = -1.0;
  initState(18) = -0.1;
  initState(19) = -0.7;
  initState(20) = 1.0;
  initState(21) = 0.1;
  initState(22) = -0.7;
  initState(23) = 1.0;  // RH_KFE
  auto costDesiredTraj = cost_desired_trajectories_t();
  costDesiredTraj.desiredTimeTrajectory().resize(1, 0.0);

  costDesiredTraj.desiredInputTrajectory().resize(1, input_vector_t::Zero());
  costDesiredTraj.desiredInputTrajectory()[0](2) = 79.36;
  costDesiredTraj.desiredInputTrajectory()[0](5) = 77.85;
  costDesiredTraj.desiredInputTrajectory()[0](8) = 81.44;
  costDesiredTraj.desiredInputTrajectory()[0](11) = 79.93;

  costDesiredTraj.desiredStateTrajectory().resize(1, state_vector_t::Zero());
  costDesiredTraj.desiredStateTrajectory()[0] = initState;

  bindings.reset(costDesiredTraj);
  //initState(3) = 0.5; // x position
  bindings.setObservation(0.0, initState);
  bindings.advanceMpc();

  auto t_arr = scalar_array_t();
  auto x_arr = state_vector_array_t();
  auto u_arr = input_vector_array_t();
  auto sigmaX_arr = state_matrix_array_t();

  bindings.getMpcSolution(t_arr, x_arr, u_arr, sigmaX_arr);

  EXPECT_EQ(t_arr.size(), x_arr.size());
  EXPECT_EQ(t_arr.size(), u_arr.size());
  EXPECT_EQ(t_arr.size(), sigmaX_arr.size());

  std::cout << "t\t\tx(3)\tx(14)" << std::endl;
  for (size_t i = 0; i < t_arr.size(); i++) {
    std::cout << std::setprecision(4);
    std::cout << t_arr[i] << "\t\t" << x_arr[i](3) << "\t" << x_arr[i](14) << std::endl;
  }

  double t = 0.0;
  for (int i = 0; i < 201; i++) {
    bindings.setObservation(t, initState);
    bindings.advanceMpc();
    bindings.getMpcSolution(t_arr, x_arr, u_arr, sigmaX_arr);
    t += 0.01;
  }

  std::cout << "computing FlowMap: " << std::endl;
  auto dxdt = bindings.computeFlowMap(t_arr[0], x_arr[0], u_arr[0]);
  std::cout << "dxdt: " << dxdt.transpose() << std::endl;

  std::cout << "computing FlowMap derivative: " << std::endl;
  bindings.setFlowMapDerivativeStateAndControl(t_arr[0], x_arr[0], u_arr[0]);
  auto A = bindings.computeFlowMapDerivativeState();
  auto B = bindings.computeFlowMapDerivativeInput();

  std::cout << "A\n" << A << "\nB\n" << B << std::endl;

  auto L = bindings.getRunningCost(t_arr[0], x_arr[0], u_arr[0]);
  auto dLdx = bindings.getRunningCostDerivativeState(t_arr[0], x_arr[0], u_arr[0]);
  auto dLdu = bindings.getRunningCostDerivativeInput(t_arr[0], x_arr[0], u_arr[0]);

  std::cout << "L: " << L << "\ndLdx: " << dLdx.transpose() << "\ndLdu: " << dLdu.transpose() << std::endl;

  auto Vx = bindings.getValueFunctionStateDerivative(t_arr[0], x_arr[0]);
  std::cout << "Vx: " << Vx.transpose() << std::endl;

//  auto e = bindings.getStateInputConstraint(t_arr[0], x_arr[0], u_arr[0]);

  bindings.reset(costDesiredTraj);
  bindings.setObservation(0.0, initState);
  bindings.advanceMpc();
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
