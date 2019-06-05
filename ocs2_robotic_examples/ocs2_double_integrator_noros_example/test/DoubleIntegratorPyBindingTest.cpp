#include <gtest/gtest.h>

#include <ocs2_double_integrator_noros_example/DoubleIntegratorPyBindings.hpp>

TEST(DoubleIntegratorTest, pyBindings) {
  using bindings_t = ocs2::double_integrator::DoubleIntegratorPyBindings;
  using state_vector_t = bindings_t::state_vector_t;
  using scalar_array_t = bindings_t::scalar_array_t;
  using state_vector_array_t = bindings_t::state_vector_array_t;
  using input_vector_array_t = bindings_t::input_vector_array_t;

  auto bindings = bindings_t("mpc");

  auto state = state_vector_t::Zero();
  bindings.setObservation(0.0, state);

  bindings.advanceMpc();

  auto t_arr = scalar_array_t();
  auto x_arr = state_vector_array_t();
  auto u_arr = input_vector_array_t();
  auto Vx_arr = state_vector_array_t();

  bindings.getMpcSolution(t_arr, x_arr, u_arr, Vx_arr);

  EXPECT_EQ(t_arr.size(), x_arr.size());
  EXPECT_EQ(t_arr.size(), u_arr.size());
  EXPECT_EQ(t_arr.size(), Vx_arr.size());

  std::cout << "t\t\tx\t\tu\t\tVx" << std::endl;
  for (size_t i = 0; i < t_arr.size(); i++) {
    std::cout << std::setprecision(4);
    std::cout << t_arr[i] << "\t\t" << x_arr[i].transpose() << "\t\t" << u_arr[i].transpose() << "\t\t" << Vx_arr[i].transpose() << std::endl;
  }

  auto dxdt = bindings.computeFlowMap(t_arr[0], x_arr[0], u_arr[0]);
  std::cout << "dxdt: " << dxdt.transpose() << std::endl;

  bindings.setFlowMapDerivativeStateAndControl(t_arr[0], x_arr[0], u_arr[0]);
  auto A = bindings.computeFlowMapDerivativeState();
  auto B = bindings.computeFlowMapDerivativeInput();

  std::cout << "A\n" << A << "\nB\n" << B << std::endl;
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
