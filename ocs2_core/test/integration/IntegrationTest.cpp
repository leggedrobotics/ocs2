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

void testSecondOrderSystem(IntegratorType integrator_type) {
  const size_t stateDim = 2;
  const size_t inputDim = 1;

  matrix_t A(2, 2);
  A << -2, -1,  // clang-format off
        1,  0;  // clang-format on
  matrix_t B(2, 1);
  B << 1, 0;

  auto sys = std::shared_ptr<ControlledSystemBase>(new LinearSystemDynamics(A, B));

  const scalar_t t0 = 0.0;
  const scalar_t t1 = 10.0;
  const scalar_t dt = 0.05;
  vector_t x0 = vector_t::Zero(2);
  scalar_array_t cntTimeStamp{0, 10};
  vector_array_t uff(2, vector_t::Ones(1));
  matrix_array_t k(2, matrix_t::Zero(1, 2));

  auto controller = std::unique_ptr<LinearController>(new LinearController(stateDim, inputDim, cntTimeStamp, uff, k));

  sys->setController(controller.get());

  std::shared_ptr<ControlledSystemBase> sysClone1(sys->clone());
  ASSERT_TRUE(sysClone1.unique());

  scalar_array_t timeTrajectory1, timeTrajectory2;
  vector_array_t stateTrajectory1, stateTrajectory2, stateTrajectory3;

  std::unique_ptr<IntegratorBase> integrator = newIntegrator(integrator_type);

  // Adaptive time integrator
  Observer observer1(&stateTrajectory1, &timeTrajectory1);
  integrator->integrateAdaptive(*sys, observer1, x0, t0, t1);

  EXPECT_NEAR(timeTrajectory1.front(), t0, 1e-6);
  EXPECT_NEAR(timeTrajectory1.back(), t1, 1e-6);
  EXPECT_TRUE(stateTrajectory1.front().isApprox(x0, 1e-6));
  EXPECT_NEAR(stateTrajectory1.back()(1), 1.0, 1e-3);

  // Equidistant time integrator
  Observer observer2(&stateTrajectory2, &timeTrajectory2);
  integrator->integrateConst(*sys, observer2, x0, t0, t1, dt);

  EXPECT_NEAR(timeTrajectory2.front(), t0, 1e-6);
  EXPECT_NEAR(timeTrajectory2.back(), t1, 1e-6);
  EXPECT_TRUE(stateTrajectory2.front().isApprox(x0, 1e-6));
  EXPECT_NEAR(stateTrajectory2.back()(1), 1.0, 1e-3);

  // Integrator with given time trajectory
  Observer observer3(&stateTrajectory3);

  // integrate with given time trajectory
  integrator->integrateTimes(*sys, observer3, x0, timeTrajectory1.begin(), timeTrajectory1.end());

  EXPECT_NEAR(stateTrajectory3.back()(1), 1.0, 1e-3);
}

TEST(IntegrationTest, SecondOrderSystem_ODE45) {
  testSecondOrderSystem(IntegratorType::ODE45);
}

TEST(IntegrationTest, SecondOrderSystem_AdamsBashfort) {
  testSecondOrderSystem(IntegratorType::ADAMS_BASHFORTH);
}

#if (BOOST_VERSION / 100000 == 1 && BOOST_VERSION / 100 % 1000 > 55)

TEST(IntegrationTest, SecondOrderSystem_AdamsBashfortMoulton) {
  testSecondOrderSystem(IntegratorType::ADAMS_BASHFORTH_MOULTON);
}

#endif

TEST(IntegrationTest, model_data_test) {
  const size_t stateDim = 2;
  const size_t inputDim = 1;

  matrix_t A(2, 2);
  A << -2, -1,  // clang-format off
        1,  0;  // clang-format on
  matrix_t B(2, 1);
  B << 1, 0;

  auto sys = std::shared_ptr<ControlledSystemBase>(new LinearSystemDynamics(A, B));

  const scalar_t t0 = 0.0;
  const scalar_t t1 = 10.0;
  vector_t x0 = vector_t::Zero(2);
  scalar_array_t cntTimeStamp{0, 10};
  vector_array_t uff(2, vector_t::Ones(1));
  matrix_array_t k(2, matrix_t::Zero(1, 2));

  auto controller = std::unique_ptr<LinearController>(new LinearController(stateDim, inputDim, cntTimeStamp, uff, k));
  sys->setController(controller.get());

  auto integrator = newIntegrator(IntegratorType::ODE45);

  scalar_array_t timeTrajectory;
  vector_array_t stateTrajectory;
  std::vector<ModelDataBase> modelDataTrajectory;

  // integrate adaptive
  sys->resetNumFunctionCalls();

  Observer observer(&stateTrajectory, &timeTrajectory, &modelDataTrajectory);
  integrator->integrateAdaptive(*sys, observer, x0, 0.0, 10.0);

  vector_t dynamics;
  sys->systemFunction()(x0, dynamics, timeTrajectory.front());
  EXPECT_TRUE(modelDataTrajectory.front().dynamics_.isApprox(dynamics, 1e-3));

  EXPECT_EQ(modelDataTrajectory.size(), stateTrajectory.size())
      << "MESSAGE: ModelData trajectory size is not equal to state trajectory size!";

  for (int i = 0; i < stateTrajectory.size(); i++) {
    ASSERT_FLOAT_EQ(modelDataTrajectory[i].time_, timeTrajectory[i])
        << "MESSAGE: ModelData trajectory time does not match the time trajectory!";
  }
}

TEST(IntegrationTest, integratorType_from_string) {
  IntegratorType type = integrator_type::fromString("ODE45");
  EXPECT_EQ(type, IntegratorType::ODE45);
}

TEST(IntegrationTest, integratorType_to_string) {
  std::string name = integrator_type::toString(IntegratorType::ODE45);
  EXPECT_EQ(name, "ODE45");
}
