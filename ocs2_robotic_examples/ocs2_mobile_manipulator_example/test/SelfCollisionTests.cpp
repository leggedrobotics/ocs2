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

#pragma once

#include <gtest/gtest.h>
#include <ros/package.h>

#include <ocs2_mobile_manipulator_example/MobileManipulatorInterface.h>
// #include <ocs2_self_collision/cost/SelfCollisionCost.h>
// #include <ocs2_self_collision/cost/SelfCollisionCostCppAd.h>

// Copied from ocs2_core/test
// TODO(perry) make not random, move hessian check elsewhere (we don't use the true hessian)
// In our functions, the jacobians may be discontinuous. So if you're using a finite differences approximation or something, your mileage
// may vary
inline void checkCostFunction(size_t numTests, ocs2::CostFunctionBase* cost1, ocs2::CostFunctionBase* cost2, bool& success, size_t stateDim,
                              size_t inputDim) {
  using ocs2::matrix_t;
  using ocs2::scalar_t;
  using ocs2::ScalarFunctionQuadraticApproximation;
  using ocs2::vector_t;

  success = true;

  const scalar_t precision = 1e-9;

  const scalar_t t = 0;
  vector_t x;
  vector_t u;

  for (size_t it = 0; it < numTests && success; it++) {
    x.setRandom(stateDim);
    u.setRandom(inputDim);

    scalar_t L1_cost = cost1->cost(t, x, u);
    scalar_t L2_cost = cost2->cost(t, x, u);
    ScalarFunctionQuadraticApproximation L1 = cost1->costQuadraticApproximation(t, x, u);
    ScalarFunctionQuadraticApproximation L2 = cost2->costQuadraticApproximation(t, x, u);

    if (std::abs(L1.f - L1_cost) > precision) {
      std::cout << "[1] L: " << L1.f << " (cost)" << std::endl;
      std::cout << "[1] L: " << L1_cost << " (costQuadraticApproximation)" << std::endl;
      success = false;
    }

    if (std::abs(L2.f - L2_cost) > precision) {
      std::cout << "[2] L: " << L2.f << " (cost)" << std::endl;
      std::cout << "[2] L: " << L2_cost << " (costQuadraticApproximation)" << std::endl;
      success = false;
    }

    if (std::abs(L1.f - L2.f) > precision) {
      std::cout << "[1] L: " << L1.f << std::endl;
      std::cout << "[2] L: " << L2.f << std::endl;
      success = false;
    }

    if (!L1.dfdx.isApprox(L2.dfdx, precision)) {
      std::cout << "[1] dLdx: " << L1.dfdx.transpose() << std::endl;
      std::cout << "[2] dLdx: " << L2.dfdx.transpose() << std::endl;
      success = false;
    }

    if (!L1.dfdxx.isApprox(L2.dfdxx, precision)) {
      std::cout << "[1] dLdxx: \n" << L1.dfdxx << std::endl;
      std::cout << "[2] dLdxx: \n" << L2.dfdxx << std::endl;
      success = false;
    }

    if (!L1.dfdu.isApprox(L2.dfdu, precision)) {
      std::cout << "[1] dLdu:    " << L1.dfdu.transpose() << std::endl;
      std::cout << "[2] dLdu: " << L2.dfdu.transpose() << std::endl;
      success = false;
    }

    if (!L1.dfduu.isApprox(L2.dfduu, precision)) {
      std::cout << "[1] dLduu: \n" << L1.dfduu << std::endl;
      std::cout << "[2] dLduu: \n" << L2.dfduu << std::endl;
      success = false;
    }

    if (!L1.dfdux.isApprox(L2.dfdux, precision)) {
      std::cout << "[1] dLdux: \n" << L1.dfdux << std::endl;
      std::cout << "[2] dLdux: \n" << L2.dfdux << std::endl;
      success = false;
    }

    scalar_t Phi1_cost = cost1->finalCost(t, x);
    scalar_t Phi2_cost = cost2->finalCost(t, x);
    ScalarFunctionQuadraticApproximation Phi1 = cost1->finalCostQuadraticApproximation(t, x);
    ScalarFunctionQuadraticApproximation Phi2 = cost2->finalCostQuadraticApproximation(t, x);

    if (std::abs(Phi1.f - Phi1_cost) > precision) {
      std::cout << "[1] L: " << Phi1.f << " (cost)" << std::endl;
      std::cout << "[1] L: " << Phi1_cost << " (costQuadraticApproximation)" << std::endl;
      success = false;
    }

    if (std::abs(Phi2.f - Phi2_cost) > precision) {
      std::cout << "[2] L: " << Phi2.f << " (cost)" << std::endl;
      std::cout << "[2] L: " << Phi2_cost << " (costQuadraticApproximation)" << std::endl;
      success = false;
    }

    if (std::abs(Phi1.f - Phi2.f) > precision) {
      std::cout << "[1] Phi: " << Phi1.f << std::endl;
      std::cout << "[2] Phi: " << Phi2.f << std::endl;
      success = false;
    }

    if (!Phi1.dfdx.isApprox(Phi2.dfdx, precision)) {
      std::cout << "[1] dPhidx: " << Phi1.dfdx.transpose() << std::endl;
      std::cout << "[2] dPhidx: " << Phi2.dfdx.transpose() << std::endl;
      success = false;
    }

    if (!Phi1.dfdxx.isApprox(Phi2.dfdxx, precision)) {
      std::cout << "[1] dPhidxx: \n" << Phi1.dfdxx << std::endl;
      std::cout << "[2] dPhidxx: \n" << Phi2.dfdxx << std::endl;
      success = false;
    }
  }  // end of for loop
}

TEST(testSelfCollision, AnalyticalVsAutoDiff) {
  const std::string urdfPath = ros::package::getPath("ocs2_mobile_manipulator_example") + "/urdf/mobile_manipulator.urdf";

  ocs2::PinocchioInterface pInterface(mobile_manipulator::MobileManipulatorInterface::buildPinocchioInterface(urdfPath));
  // ocs2::PinocchioGeometryInterface gInterface(urdfPath, pInterface, {{1, 4}, {1, 6}, {1, 9}});
  // TODO(perry) get the collision pairs from the task.info file to match the current mpc setup
  // gInterface->getGeometryModel().addAllCollisionPairs();

  const ocs2::scalar_t minDistance = 0.1;
  const ocs2::scalar_t mu = 0.01;
  const ocs2::scalar_t delta = 1e-3;

  // ocs2::SelfCollisionCost selfCollisionCost(pInterface, gInterface, minDistance, mu, delta);
  // ocs2::SelfCollisionCostCppAd selfCollisionCostCppAd(pInterface, gInterface, minDistance, mu, delta);

  auto libraryFolder = ros::package::getPath("ocs2_mobile_manipulator_example") + "/auto_generated";
  // selfCollisionCostCppAd.initialize("ColCost", libraryFolder, true, false);

  bool success = true;
  // checkCostFunction(100, &selfCollisionCost, &selfCollisionCostCppAd, success, 9, 8);
  ASSERT_TRUE(success);
}
