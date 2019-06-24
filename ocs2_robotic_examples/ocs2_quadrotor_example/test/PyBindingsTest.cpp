#include <gtest/gtest.h>

#include <ocs2_quadrotor_example/QuadrotorPyBindings.hpp>

TEST(QuadrotorTest, PyBindings) {
  using bindings_t = ocs2::quadrotor::QuadrotorPyBindings;
  using state_vector_t = bindings_t::state_vector_t;
  using state_matrix_array_t = bindings_t::state_matrix_array_t;
  using scalar_array_t = bindings_t::scalar_array_t;
  using state_vector_array_t = bindings_t::state_vector_array_t;
  using input_vector_array_t = bindings_t::input_vector_array_t;
  using input_state_matrix_array_t = bindings_t::input_state_matrix_array_t;
  using cost_desired_trajectories_t = bindings_t::cost_desired_trajectories_t;

  bindings_t bindings("mpc", false);

  auto costDesiredTraj = cost_desired_trajectories_t();
  bindings.reset(costDesiredTraj);

  auto state = state_vector_t::Zero();
  bindings.setObservation(0.0, state);

  bindings.advanceMpc();

  auto t_arr = scalar_array_t();
  auto x_arr = state_vector_array_t();
  auto u_arr = input_vector_array_t();
  auto sigmaX_arr = state_matrix_array_t();


  bindings.getMpcSolution(t_arr, x_arr, u_arr, sigmaX_arr);

  EXPECT_EQ(t_arr.size(), x_arr.size());
  EXPECT_EQ(t_arr.size(), u_arr.size());
  EXPECT_EQ(t_arr.size(), sigmaX_arr.size());

  std::cout << "t\t\tx\t\tu\t\tsigmaX" << std::endl;
  for (size_t i = 0; i < t_arr.size(); i++) {
    std::cout << std::setprecision(4);
    std::cout << t_arr[i] << "\t\t" << x_arr[i].transpose() << "\t\t" << u_arr[i].transpose() << "\t\t" << sigmaX_arr[i] << std::endl;
  }

  auto dxdt = bindings.computeFlowMap(t_arr[0], x_arr[0], u_arr[0]);
  std::cout << "dxdt: " << dxdt.transpose() << std::endl;

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

  bindings.reset(costDesiredTraj);
  bindings.setObservation(0.0, state);
  bindings.advanceMpc();
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
