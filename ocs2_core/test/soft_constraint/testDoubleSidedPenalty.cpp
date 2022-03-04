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

#include <ocs2_core/penalties/penalties/DoubleSidedPenalty.h>
#include <ocs2_core/penalties/penalties/SquaredHingePenalty.h>

TEST(testDoubleSidedPenalty, value) {
  const ocs2::scalar_t eps = 1e-9;
  const ocs2::scalar_t t = 0.0;
  const ocs2::scalar_t l = -1.0;
  const ocs2::scalar_t u = 1.0;
  ocs2::SquaredHingePenalty penalty(ocs2::SquaredHingePenalty::Config(1.0, 0.0));
  ocs2::DoubleSidedPenalty bounds(l, u, std::unique_ptr<ocs2::SquaredHingePenalty>(penalty.clone()));

  EXPECT_NEAR(bounds.getValue(t, 0.0), 0.0, eps);
  EXPECT_NEAR(bounds.getValue(t, 2.0), penalty.getValue(t, u - 2.0), eps);
  EXPECT_NEAR(bounds.getValue(t, -3.0), penalty.getValue(t, -3.0 - l), eps);
}

TEST(testDoubleSidedPenalty, derivative) {
  const ocs2::scalar_t eps = 1e-9;
  const ocs2::scalar_t t = 0.0;
  const ocs2::scalar_t l = -1.0;
  const ocs2::scalar_t u = 1.0;
  ocs2::SquaredHingePenalty penalty(ocs2::SquaredHingePenalty::Config(1.0, 0.0));
  ocs2::DoubleSidedPenalty bounds(l, u, std::unique_ptr<ocs2::SquaredHingePenalty>(penalty.clone()));

  EXPECT_NEAR(bounds.getDerivative(t, 0.0), 0.0, eps);
  EXPECT_NEAR(bounds.getDerivative(t, 2.0), -penalty.getDerivative(t, u - 2.0), eps);
  EXPECT_NEAR(bounds.getDerivative(t, -3.0), penalty.getDerivative(t, -3.0 - l), eps);
}

TEST(testDoubleSidedPenalty, secondDerivative) {
  const ocs2::scalar_t eps = 1e-9;
  const ocs2::scalar_t t = 0.0;
  const ocs2::scalar_t l = -1.0;
  const ocs2::scalar_t u = 1.0;
  ocs2::SquaredHingePenalty penalty(ocs2::SquaredHingePenalty::Config(1.0, 0.0));
  ocs2::DoubleSidedPenalty bounds(l, u, std::unique_ptr<ocs2::SquaredHingePenalty>(penalty.clone()));

  EXPECT_NEAR(bounds.getSecondDerivative(t, 0.0), 0.0, eps);
  EXPECT_NEAR(bounds.getSecondDerivative(t, 2.0), penalty.getSecondDerivative(t, u - 2.0), eps);
  EXPECT_NEAR(bounds.getSecondDerivative(t, -3.0), penalty.getSecondDerivative(t, -3.0 - l), eps);
}
