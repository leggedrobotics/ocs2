#include <gtest/gtest.h>

#include <ocs2_double_integrator_example/DoubleIntegratorPyBindings.h>

TEST(DoubleIntegratorTest, pyBindings) {
  using bindings_t = ocs2::double_integrator::DoubleIntegratorPyBindings;

  bindings_t bindings("mpc");

  const ocs2::vector_t state = ocs2::vector_t::Zero(ocs2::double_integrator::STATE_DIM);
  const ocs2::vector_t zeroInput = ocs2::vector_t::Zero(ocs2::double_integrator::INPUT_DIM);
  bindings.setObservation(0.0, state, zeroInput);

  bindings.setTargetTrajectories(ocs2::CostDesiredTrajectories({0.0}, {state}, {zeroInput}));

  bindings.advanceMpc();

  ocs2::scalar_array_t t_arr;
  ocs2::vector_array_t x_arr;
  ocs2::vector_array_t u_arr;

  bindings.getMpcSolution(t_arr, x_arr, u_arr);

  EXPECT_EQ(t_arr.size(), x_arr.size());
  EXPECT_EQ(t_arr.size(), u_arr.size());

  std::cout << "t\t\tx\t\tu" << std::endl;
  for (size_t i = 0; i < t_arr.size(); i++) {
    std::cout << std::setprecision(4);
    std::cout << t_arr[i] << "\t\t" << x_arr[i].transpose() << "\t\t" << u_arr[i].transpose() << std::endl;
  }

  const auto dxdt = bindings.flowMap(t_arr[0], x_arr[0], u_arr[0]);
  std::cout << "dxdt: " << dxdt.transpose() << std::endl;

  const auto flowMap = bindings.flowMapLinearApproximation(t_arr[0], x_arr[0], u_arr[0]);
  std::cout << "A\n" << flowMap.dfdx << "\nB\n" << flowMap.dfdu << std::endl;

  const auto L = bindings.costQuadraticApproximation(t_arr[0], x_arr[0], u_arr[0]);
  std::cout << "L: " << L.f << "\ndLdx: " << L.dfdx.transpose() << "\ndLdu: " << L.dfdu.transpose() << std::endl;

  //  auto Vx = bindings.getValueFunctionStateDerivative(t_arr[0], x_arr[0]);
  //  std::cout << "Vx: " << Vx.transpose() << std::endl;

  auto K = bindings.getLinearFeedbackGain(t_arr[0]);
  std::cout << "K: " << K << std::endl;
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
