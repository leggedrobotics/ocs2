#include <gtest/gtest.h>

#include <ocs2_quadrotor/QuadrotorPyBindings.h>
#include <ocs2_quadrotor/package_path.h>

TEST(QuadrotorTest, PyBindings) {
  const std::string taskFile = ocs2::quadrotor::getPath() + "/config/mpc/task.info";
  const std::string libFolder = ocs2::quadrotor::getPath() + "/auto_generated";
  ocs2::quadrotor::QuadrotorPyBindings bindings(taskFile, libFolder);

  ocs2::vector_t initState = ocs2::vector_t::Zero(ocs2::quadrotor::STATE_DIM);
  initState(0) = 0.0;
  initState(2) = 0.0;
  const ocs2::vector_t zeroInput = ocs2::vector_t::Zero(ocs2::quadrotor::INPUT_DIM);

  ocs2::TargetTrajectories targetTrajectories;
  targetTrajectories.timeTrajectory.resize(2, 0.0);
  targetTrajectories.timeTrajectory[1] = 2.0;
  targetTrajectories.inputTrajectory.resize(2, ocs2::vector_t::Zero(ocs2::quadrotor::INPUT_DIM));
  targetTrajectories.stateTrajectory.resize(2, ocs2::vector_t::Zero(ocs2::quadrotor::STATE_DIM));
  targetTrajectories.stateTrajectory[0] = initState;

  bindings.reset(targetTrajectories);
  bindings.setObservation(0.0, initState, zeroInput);
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

  bindings.reset(targetTrajectories);
  bindings.setObservation(0.0, initState, zeroInput);
  bindings.advanceMpc();
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
