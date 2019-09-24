#include <gtest/gtest.h>
#include <ros/package.h>

#include <ocs2_cart_pole_example/CartPoleInterface.h>
#include <ocs2_cart_pole_example/definitions.h>
#include <ocs2_cart_pole_raisim_example/CartpoleRaisimConversions.h>
#include <ocs2_core/control/FeedforwardController.h>
#include <ocs2_raisim/RaisimRollout.h>

using ocs2::cartpole::INPUT_DIM_;
using ocs2::cartpole::STATE_DIM_;
using state_vector_t = ocs2::Dimensions<STATE_DIM_, INPUT_DIM_>::state_vector_t;
using input_vector_t = ocs2::Dimensions<STATE_DIM_, INPUT_DIM_>::input_vector_t;
using scalar_array_t = ocs2::Dimensions<STATE_DIM_, INPUT_DIM_>::scalar_array_t;
using size_array_t = ocs2::Dimensions<STATE_DIM_, INPUT_DIM_>::size_array_t;
using state_vector_array_t = ocs2::Dimensions<STATE_DIM_, INPUT_DIM_>::state_vector_array_t;
using input_vector_array_t = ocs2::Dimensions<STATE_DIM_, INPUT_DIM_>::input_vector_array_t;

TEST(ocs2_cart_pole_raisim_example, ConversionsConsistency) {
  state_vector_t ocs2_state;
  ocs2_state.setRandom();

  input_vector_t ocs2_input;
  ocs2_input.setConstant(NAN);

  Eigen::VectorXd raisim_q, raisim_dq;
  std::tie(raisim_q, raisim_dq) = ocs2::cartpole::stateToRaisimGenCoordGenVel(ocs2_state, ocs2_input);

  state_vector_t ocs2_state2 = ocs2::cartpole::raisimGenCoordGenVelToState(raisim_q, raisim_dq);
  bool test = ocs2_state.isApprox(ocs2_state2);
  ASSERT_TRUE(test);

  Eigen::VectorXd raisim_q2, raisim_dq2;
  std::tie(raisim_q2, raisim_dq2) = ocs2::cartpole::stateToRaisimGenCoordGenVel(ocs2_state2, ocs2_input);
  test = raisim_q2.isApprox(raisim_q);
  ASSERT_TRUE(test);
  test = raisim_dq2.isApprox(raisim_dq);
  ASSERT_TRUE(test);
}

TEST(ocs2_cart_pole_raisim_example, RolloutTest) {
  scalar_array_t controllerTime{0.0, 1.0};
  input_vector_array_t controllerInput(2, input_vector_t::Zero());

  ocs2::FeedforwardController<STATE_DIM_, INPUT_DIM_> ctrl(controllerTime, controllerInput);

  size_t partitionIndex = 0;
  const double t0 = 0.0;
  const double tf = 2.0;
  state_vector_t x_init;
  x_init.setZero();
  x_init(0) = M_PI;  // pole position (0 corresponds to up)
  x_init(1) = 0.3;   // cart position
  x_init(3) = 5.0;   // initial cart velocity

  state_vector_t finalState;
  {
    ocs2::HybridLogicRulesMachine logicRulesMachine;
    scalar_array_t timeTrajectory;
    size_array_t eventsPastTheEndIndeces;
    state_vector_array_t stateTrajectory;
    input_vector_array_t inputTrajectory;

    ocs2::RaisimRollout<STATE_DIM_, INPUT_DIM_> rollout(
        ros::package::getPath("ocs2_cart_pole_example") + "/urdf/cartpole.urdf", &ocs2::cartpole::stateToRaisimGenCoordGenVel,
        &ocs2::cartpole::raisimGenCoordGenVelToState, &ocs2::cartpole::inputToRaisimGeneralizedForce);

    finalState = rollout.run(partitionIndex, t0, x_init, tf, &ctrl, logicRulesMachine, timeTrajectory, eventsPastTheEndIndeces,
                             stateTrajectory, inputTrajectory);
  }

  state_vector_t finalState_expected = x_init;
  finalState_expected(1) += (tf - t0) * x_init(3);

  bool test = finalState.isApprox(finalState_expected);
  ASSERT_TRUE(test);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
