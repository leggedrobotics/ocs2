#ifdef NDEBUG
#undef NDEBUG
#endif

#include <gtest/gtest.h>
#include <ros/package.h>

#include <ocs2_anymal_bear/AnymalBearPyBindings.h>

TEST(Anymal, PyBindings) {
  const std::string taskFile = "mpc";
  anymal::AnymalBearPyBindings bindings(taskFile);

  ocs2::vector_t initState = ocs2::vector_t::Zero(switched_model::STATE_DIM);
  initState(5) = 0.495;  // base z
  initState(12) = -0.1;  // LF_HAA
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

  ocs2::vector_t initInput = ocs2::vector_t::Zero(switched_model::INPUT_DIM);
  initInput(2) = 79.36;
  initInput(5) = 77.85;
  initInput(8) = 81.44;
  initInput(11) = 79.93;

  auto costDesiredTraj = ocs2::CostDesiredTrajectories();
  costDesiredTraj.desiredTimeTrajectory().push_back(0.0);
  costDesiredTraj.desiredInputTrajectory().push_back(initInput);
  costDesiredTraj.desiredStateTrajectory().push_back(initState);

  bindings.reset(costDesiredTraj);
  // initState(3) = 0.5; // x position
  bindings.setObservation(0.0, initState, initInput);
  bindings.advanceMpc();

  ocs2::scalar_array_t t_arr;
  ocs2::vector_array_t x_arr;
  ocs2::vector_array_t u_arr;

  bindings.getMpcSolution(t_arr, x_arr, u_arr);

  EXPECT_EQ(t_arr.size(), x_arr.size());
  EXPECT_EQ(t_arr.size(), u_arr.size());

  std::cerr << "t\t\tx(3)\tx(14)" << std::endl;
  for (size_t i = 0; i < t_arr.size(); i++) {
    std::cerr << std::setprecision(4);
    std::cerr << t_arr[i] << "\t\t" << x_arr[i](3) << "\t" << x_arr[i](14) << std::endl;
  }

  ocs2::scalar_t t = 0.0;
  for (int i = 0; i < 301; i++) {
    bindings.setObservation(t, initState, initInput);
    bindings.advanceMpc();
    bindings.getMpcSolution(t_arr, x_arr, u_arr);
    t += 0.01;
  }

  std::cerr << "computing FlowMap: " << std::endl;
  const auto dxdt = bindings.flowMap(t_arr[0], x_arr[0], u_arr[0]);
  std::cerr << "dxdt: " << dxdt.transpose() << std::endl;

  const auto flowMap = bindings.flowMapLinearApproximation(t_arr[0], x_arr[0], u_arr[0]);
  std::cerr << "A\n" << flowMap.dfdx << "\nB\n" << flowMap.dfdu << std::endl;

  const auto L = bindings.costQuadraticApproximation(t_arr[0], x_arr[0], u_arr[0]);
  std::cerr << "L: " << L.f << "\ndLdx: " << L.dfdx.transpose() << "\ndLdu: " << L.dfdu.transpose() << std::endl;

  auto V = bindings.valueFunction(t_arr[0], x_arr[0]);
  std::cerr << "V: " << V << std::endl;

  auto Vx = bindings.valueFunctionStateDerivative(t_arr[0], x_arr[0]);
  std::cerr << "Vx: " << Vx.transpose() << std::endl;

  auto e = bindings.stateInputEqualityConstraint(t_arr[0], x_arr[0], u_arr[0]);
  std::cerr << "e: " << e.transpose() << std::endl;

  auto D = bindings.stateInputEqualityConstraintLinearApproximation(t_arr[0], x_arr[0], u_arr[0]).dfdu;
  std::cerr << "D:\n" << D << std::endl;

  auto nu = bindings.stateInputEqualityConstraintLagrangian(t_arr[0], x_arr[0], u_arr[0]);
  std::cerr << "nu " << nu.transpose() << std::endl;

  bindings.reset(costDesiredTraj);

  t = 0.0;
  for (int i = 0; i < 301; i++) {
    bindings.setObservation(t, initState, initInput);
    bindings.advanceMpc();
    bindings.getMpcSolution(t_arr, x_arr, u_arr);
    t += 0.01;
  }

  bindings.reset(costDesiredTraj);
  bindings.setObservation(t, initState, initInput);
  bindings.advanceMpc();
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
