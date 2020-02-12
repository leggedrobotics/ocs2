/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <gtest/gtest.h>

#include <fstream>
#include <memory>

#include <ocs2_core/control/LinearController.h>
#include <ocs2_core/dynamics/LinearSystemDynamics.h>
#include <ocs2_core/integration/Integrator.h>

using namespace ocs2;

template <class Integrator>
void testSecondOrderSystem() {
  Eigen::Matrix2d A;
  A << -2, -1,  // clang-format off
        1,  0;  // clang-format on
  Eigen::Vector2d B;
  B << 1, 0;

  typedef LinearSystemDynamics<2, 1> SecondOrderSystem;
  ControlledSystemBase<2, 1>::Ptr sys = ControlledSystemBase<2, 1>::Ptr(new SecondOrderSystem(A, B));

  const double t0 = 0.0;
  const double t1 = 10.0;
  SecondOrderSystem::scalar_array_t cntTimeStamp{0, 10};
  SecondOrderSystem::input_vector_array_t uff(2, SecondOrderSystem::input_vector_t::Ones());
  SecondOrderSystem::input_state_matrix_array_t k(2, SecondOrderSystem::input_state_matrix_t::Zero());

  using controller_t = ocs2::LinearController<2, 1>;
  auto controller = std::unique_ptr<controller_t>(new controller_t(cntTimeStamp, uff, k));

  sys->setController(controller.get());

  ControlledSystemBase<2, 1>::Ptr sysClone1(sys->clone());
  ASSERT_TRUE(sysClone1.unique());

  std::vector<double> timeTrajectory1, timeTrajectory2;
  std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> stateTrajectory1, stateTrajectory2, stateTrajectory3;

  Eigen::Vector2d x0;
  x0.setZero();
  const double dt = 0.05;

  // Adaptive time integrator
  Integrator odeAdaptive;
  Observer<2> observer1(&stateTrajectory1, &timeTrajectory1);
  odeAdaptive.integrate_adaptive(*sys, observer1, x0, t0, t1);

  EXPECT_NEAR(timeTrajectory1.front(), t0, 1e-6);
  EXPECT_NEAR(timeTrajectory1.back(), t1, 1e-6);
  EXPECT_TRUE(stateTrajectory1.front().isApprox(x0, 1e-6));
  EXPECT_NEAR(stateTrajectory1.back()(1), 1.0, 1e-3);

  // Equidistant time integrator
  Integrator odeConst;
  Observer<2> observer2(&stateTrajectory2, &timeTrajectory2);
  odeConst.integrate_const(*sys, observer2, x0, t0, t1, dt);

  EXPECT_NEAR(timeTrajectory2.front(), t0, 1e-6);
  EXPECT_NEAR(timeTrajectory2.back(), t1, 1e-6);
  EXPECT_TRUE(stateTrajectory2.front().isApprox(x0, 1e-6));
  EXPECT_NEAR(stateTrajectory2.back()(1), 1.0, 1e-3);

  // Integrator with given time trajectory
  Integrator odeTime;
  Observer<2> observer3(&stateTrajectory3);

  // integrate with given time trajectory
  odeTime.integrate_times(*sys, observer3, x0, timeTrajectory1.begin(), timeTrajectory1.end());

  EXPECT_NEAR(stateTrajectory3.back()(1), 1.0, 1e-3);
}

TEST(IntegrationTest, SecondOrderSystem_ODE45) {
  testSecondOrderSystem<ODE45<2>>();
}

TEST(IntegrationTest, SecondOrderSystem_AdamsBashfort) {
  const size_t order = 5;
  using AdamBashforth = IntegratorAdamsBashforth<2, order>;
  testSecondOrderSystem<AdamBashforth>();
}

#if (BOOST_VERSION / 100000 == 1 && BOOST_VERSION / 100 % 1000 > 55)

TEST(IntegrationTest, SecondOrderSystem_AdamsBashfortMoulton) {
  const size_t order = 5;
  using AdamsBashforthMoulton = IntegratorAdamsBashforthMoulton<2, order>;
  testSecondOrderSystem<AdamsBashforthMoulton>();
}

#endif

TEST(IntegrationTest, integratorType_from_string) {
  IntegratorType type = integrator_type::fromString("ODE45");
  EXPECT_EQ(type, IntegratorType::ODE45);
}

TEST(IntegrationTest, integratorType_to_string) {
  std::string name = integrator_type::toString(IntegratorType::ODE45);
  EXPECT_EQ(name, "ODE45");
}
