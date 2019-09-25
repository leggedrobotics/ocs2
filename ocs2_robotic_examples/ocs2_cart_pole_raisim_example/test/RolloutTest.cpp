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
  EXPECT_TRUE(test);

  Eigen::VectorXd raisim_q2, raisim_dq2;
  std::tie(raisim_q2, raisim_dq2) = ocs2::cartpole::stateToRaisimGenCoordGenVel(ocs2_state2, ocs2_input);
  test = raisim_q2.isApprox(raisim_q);
  EXPECT_TRUE(test);
  test = raisim_dq2.isApprox(raisim_dq);
  EXPECT_TRUE(test);
}

TEST(ocs2_cart_pole_raisim_example, RolloutTest) {
  // define zero-input controller
  const scalar_array_t controllerTime{0.0, 1.0};
  const input_vector_array_t controllerInput(2, input_vector_t::Zero());
  ocs2::FeedforwardController<STATE_DIM_, INPUT_DIM_> ctrl(controllerTime, controllerInput);

  const double t0 = 0.2;
  const double tf = 2.2;
  const scalar_array_t eventTimes{0.1, 0.5, 1.0, 1.0, 1.5, 2.5};
  state_vector_t x_init;
  x_init.setZero();
  x_init(0) = M_PI;  // pole position (0 corresponds to up)
  x_init(1) = 0.3;   // cart position
  x_init(3) = 5.0;   // initial cart velocity

  scalar_array_t timeTrajectory;
  size_array_t eventsPastTheEndIndeces;
  state_vector_array_t stateTrajectory;
  input_vector_array_t inputTrajectory;

  ocs2::RaisimRollout<STATE_DIM_, INPUT_DIM_> rollout(
      ros::package::getPath("ocs2_cart_pole_example") + "/urdf/cartpole.urdf", &ocs2::cartpole::stateToRaisimGenCoordGenVel,
      &ocs2::cartpole::raisimGenCoordGenVelToState, &ocs2::cartpole::inputToRaisimGeneralizedForce);
  rollout.settings().minTimeStep_ = 0.2;

  state_vector_t finalState =
      rollout.run(t0, x_init, tf, &ctrl, eventTimes, timeTrajectory, eventsPastTheEndIndeces, stateTrajectory, inputTrajectory);

  // check if initial state is what we expected
  {
    bool test = x_init.isApprox(stateTrajectory.front());
    EXPECT_TRUE(test);
  }

  // check if final state is consistent with return value
  {
    bool test = finalState.isApprox(stateTrajectory.back());
    EXPECT_TRUE(test);
  }

  // check if final state is what we expected
  {
    state_vector_t finalState_expected = x_init;
    finalState_expected(1) += (tf - t0) * x_init(3);
    bool test = finalState.isApprox(finalState_expected, 1e-7);
    EXPECT_TRUE(test);
  }

  // check if event times have been treated correctly
  {
    const auto eventTimesBetweent0tf =
        std::upper_bound(eventTimes.begin(), eventTimes.end(), tf) - std::upper_bound(eventTimes.begin(), eventTimes.end(), t0);
    EXPECT_EQ(eventTimesBetweent0tf, eventsPastTheEndIndeces.size());

    for (size_t i = 0; i < eventsPastTheEndIndeces.size(); i++) {
      auto timeJustBeforeEvent = timeTrajectory[eventsPastTheEndIndeces[i] - 1];
      auto eventTime = *(std::upper_bound(eventTimes.begin(), eventTimes.end(), t0) + i);
      EXPECT_DOUBLE_EQ(eventTime, timeJustBeforeEvent);
    }
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
