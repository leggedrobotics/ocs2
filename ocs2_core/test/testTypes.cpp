/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#include <ocs2_core/Types.h>

namespace {
// Tests if the convention for state only quadratic approximation works with the defined operations (addition and scalar multiplication)

void stateOnlyLinearOperationsTest(const ocs2::ScalarFunctionLinearApproximation& value, size_t nx) {
  // Define state only 'other' value
  const ocs2::ScalarFunctionLinearApproximation other = [=] {
    ocs2::ScalarFunctionLinearApproximation other;
    other.f = 1.5;
    other.dfdx.setConstant(nx, 2.3);
    return other;
  }();

  {  // test value + other
    auto result = value;
    result += other;
    EXPECT_DOUBLE_EQ(result.f, value.f + other.f);
    EXPECT_TRUE(result.dfdx.isApprox(value.dfdx + other.dfdx));
  }

  {  // test other + value
    auto result = other;
    result += value;
    EXPECT_DOUBLE_EQ(result.f, value.f + other.f);
    EXPECT_TRUE(result.dfdx.isApprox(value.dfdx + other.dfdx));
  }

  {  // test  value *= scalar
    const ocs2::scalar_t s{2.0};
    auto result = value;
    result *= s;
    EXPECT_DOUBLE_EQ(result.f, s * value.f);
    EXPECT_TRUE(result.dfdx.isApprox(s * value.dfdx));
  }
}

void stateOnlyQuadraticOperationsTest(const ocs2::ScalarFunctionQuadraticApproximation& value, size_t nx) {
  // Define state only 'other' value
  const ocs2::ScalarFunctionQuadraticApproximation other = [=] {
    ocs2::ScalarFunctionQuadraticApproximation other;
    other.f = 1.5;
    other.dfdx.setConstant(nx, 2.3);
    other.dfdxx.setConstant(nx, nx, 8.6);
    return other;
  }();

  {  // test value + other
    auto result = value;
    result += other;
    EXPECT_DOUBLE_EQ(result.f, value.f + other.f);
    EXPECT_TRUE(result.dfdx.isApprox(value.dfdx + other.dfdx));
    EXPECT_TRUE(result.dfdxx.isApprox(value.dfdxx + other.dfdxx));
  }

  {  // test other + value
    auto result = other;
    result += value;
    EXPECT_DOUBLE_EQ(result.f, value.f + other.f);
    EXPECT_TRUE(result.dfdx.isApprox(value.dfdx + other.dfdx));
    EXPECT_TRUE(result.dfdxx.isApprox(value.dfdxx + other.dfdxx));
  }

  {  // test  value *= scalar
    const ocs2::scalar_t s{2.0};
    auto result = value;
    result *= s;
    EXPECT_DOUBLE_EQ(result.f, s * value.f);
    EXPECT_TRUE(result.dfdx.isApprox(s * value.dfdx));
    EXPECT_TRUE(result.dfdxx.isApprox(s * value.dfdxx));
  }
}

}  // namespace

TEST(testTypes, stateOnlyLinearOperations_constructor) {
  const int nx = 2;
  auto constructed = ocs2::ScalarFunctionLinearApproximation(nx);
  // Need to set values after resize.
  constructed.f = 0.5;
  constructed.dfdx.setConstant(1.0);
  stateOnlyLinearOperationsTest(constructed, nx);
}

TEST(testTypes, stateOnlyLinearOperations_zero) {
  const int nx = 2;
  stateOnlyLinearOperationsTest(ocs2::ScalarFunctionLinearApproximation::Zero(nx), nx);
}

TEST(testTypes, stateOnlyLinearOperations_setZero) {
  const int nx = 2;
  ocs2::ScalarFunctionLinearApproximation zero;
  zero.setZero(nx);
  stateOnlyLinearOperationsTest(zero, nx);
}

TEST(testTypes, stateOnlyLinearOperations_resize) {
  const int nx = 2;
  ocs2::ScalarFunctionLinearApproximation resized(nx + 1, 10);
  resized.resize(nx);
  // Need to set values after resize.
  resized.f = 0.5;
  resized.dfdx.setConstant(1.0);
  stateOnlyLinearOperationsTest(resized, nx);
}

TEST(testTypes, stateOnlyQuadraticOperations_constructor) {
  const int nx = 2;
  auto constructed = ocs2::ScalarFunctionQuadraticApproximation(nx);
  // Need to set values after resize.
  constructed.f = 0.5;
  constructed.dfdx.setConstant(1.0);
  constructed.dfdxx.setConstant(2.0);
  stateOnlyQuadraticOperationsTest(constructed, nx);
}

TEST(testTypes, stateOnlyQuadraticOperations_zero) {
  const int nx = 2;
  stateOnlyQuadraticOperationsTest(ocs2::ScalarFunctionQuadraticApproximation::Zero(nx), nx);
}

TEST(testTypes, stateOnlyQuadraticOperations_setZero) {
  const int nx = 2;
  ocs2::ScalarFunctionQuadraticApproximation zero;
  zero.setZero(nx);
  stateOnlyQuadraticOperationsTest(zero, nx);
}

TEST(testTypes, stateOnlyQuadraticOperations_resize) {
  const int nx = 2;
  ocs2::ScalarFunctionQuadraticApproximation resized(nx + 1, 10);
  resized.resize(nx);
  // Need to set values after resize.
  resized.f = 0.5;
  resized.dfdx.setConstant(1.0);
  resized.dfdxx.setConstant(2.0);
  stateOnlyQuadraticOperationsTest(resized, nx);
}
