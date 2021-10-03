/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

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
#include <iostream>

#include <ocs2_centroidal_model/AccessHelperFunctions.h>

#include "ocs2_legged_robot/common/ModelSettings.h"
#include "ocs2_legged_robot/common/Types.h"
#include "ocs2_legged_robot/constraint/ZeroForceConstraint.h"
#include "ocs2_legged_robot/test/AnymalFactoryFunctions.h"

using namespace ocs2;
using namespace legged_robot;

class TestZeroForceConstraint : public ::testing::Test {
 public:
  TestZeroForceConstraint() {}

  const ModelSettings modelSettings;  // default constructor just to get contactNames3DoF
  const CentroidalModelType centroidalModelType = CentroidalModelType::SingleRigidBodyDynamics;
  std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr = createAnymalPinocchioInterface();
  const CentroidalModelInfo centroidalModelInfo = createAnymalCentroidalModelInfo(*pinocchioInterfacePtr, centroidalModelType);
  const std::shared_ptr<SwitchedModelReferenceManager> referenceManagerPtr =
      createReferenceManager(centroidalModelInfo.numThreeDofContacts);
  PreComputation preComputation;
};

TEST_F(TestZeroForceConstraint, evaluate) {
  for (size_t i = 0; i < centroidalModelInfo.numThreeDofContacts; i++) {
    ZeroForceConstraint zeroForceConstraint(*referenceManagerPtr, i, centroidalModelInfo);

    // evaluation point
    const scalar_t t = 0.0;
    const vector_t u = vector_t::Random(centroidalModelInfo.inputDim);
    const vector_t x = vector_t::Random(centroidalModelInfo.stateDim);
    const vector3_t eeForce = centroidal_model::getContactForces(u, i, centroidalModelInfo);

    const auto value = zeroForceConstraint.getValue(t, x, u, preComputation);
    const auto approx = zeroForceConstraint.getLinearApproximation(t, x, u, preComputation);

    std::cerr << "Contact: " << modelSettings.contactNames3DoF[i] << "\n";
    std::cerr << "Value\n" << value.transpose() << "\n";
    std::cerr << "LinearApproximation\n" << approx << "\n";

    EXPECT_TRUE(value.isApprox(approx.f));
    EXPECT_TRUE(approx.dfdx.isApproxToConstant(0.0));
    EXPECT_TRUE(eeForce.isApprox(approx.dfdu * u));
  }
}

TEST_F(TestZeroForceConstraint, clone) {
  constexpr size_t eeIndex = 0;
  ZeroForceConstraint zeroForceConstraint(*referenceManagerPtr, eeIndex, centroidalModelInfo);
  std::unique_ptr<ZeroForceConstraint> zeroForceConstraintPtr(zeroForceConstraint.clone());

  // evaluation point
  const scalar_t t = 0.0;
  const vector_t u = vector_t::Random(centroidalModelInfo.inputDim);
  const vector_t x = vector_t::Random(centroidalModelInfo.stateDim);

  const auto value = zeroForceConstraint.getValue(t, x, u, preComputation);
  const auto cloneValue = zeroForceConstraintPtr->getValue(t, x, u, preComputation);

  const auto approx = zeroForceConstraint.getLinearApproximation(t, x, u, preComputation);
  const auto cloneApprox = zeroForceConstraintPtr->getLinearApproximation(t, x, u, preComputation);

  EXPECT_TRUE(value.isApprox(cloneValue));
  EXPECT_TRUE(approx.f.isApprox(cloneApprox.f));
  EXPECT_TRUE(approx.dfdx.isApprox(cloneApprox.dfdx));
  EXPECT_TRUE(approx.dfdu.isApprox(cloneApprox.dfdu));
}
