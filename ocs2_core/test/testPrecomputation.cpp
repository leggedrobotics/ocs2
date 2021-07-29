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

#include <ocs2_core/ComputationRequest.h>
#include <ocs2_core/PreComputation.h>

using Request = ocs2::Request;

TEST(testPrecomputation, convenientInterface) {
  // Check that this compiles
  ocs2::PreComputation preComputation;
  preComputation.request(Request::Dynamics + Request::Approximation, 0.0, ocs2::vector_t(), ocs2::vector_t());
  preComputation.requestPreJump(Request::Dynamics + Request::Approximation, 0.0, ocs2::vector_t());
  preComputation.requestFinal(Request::Approximation, 0.0, ocs2::vector_t());
}

TEST(testPrecomputation, containts) {
  constexpr auto request = Request::Dynamics + Request::Cost + Request::Constraint + Request::SoftConstraint + Request::Approximation;
  ASSERT_TRUE(request.contains(Request::Dynamics));
  ASSERT_TRUE(request.contains(Request::Cost));
  ASSERT_TRUE(request.contains(Request::Constraint));
  ASSERT_TRUE(request.contains(Request::SoftConstraint));
  ASSERT_TRUE(request.contains(Request::Approximation));

  constexpr ocs2::RequestSet request2 = Request::Dynamics;
  ASSERT_TRUE(request2.contains(Request::Dynamics));
  ASSERT_FALSE(request2.contains(Request::Cost));
  ASSERT_FALSE(request2.contains(Request::Constraint));
  ASSERT_FALSE(request2.contains(Request::SoftConstraint));
  ASSERT_FALSE(request2.contains(Request::Approximation));

  constexpr auto request3 = Request::Cost + Request::Constraint;
  ASSERT_FALSE(request3.contains(Request::Dynamics));
  ASSERT_TRUE(request3.contains(Request::Constraint));
  ASSERT_TRUE(request3.contains(Request::Cost));
  ASSERT_FALSE(request3.contains(Request::SoftConstraint));
  ASSERT_FALSE(request3.contains(Request::Approximation));
}

TEST(testPrecomputation, containtsAnyAll) {
  constexpr auto request1 = Request::Dynamics + Request::Cost;
  constexpr auto request2 = Request::Constraint + Request::Cost;
  ASSERT_TRUE(request1.containsAny(request2));
  ASSERT_FALSE(request1.containsAll(request2));

  constexpr auto request3 = Request::Constraint + Request::Cost + Request::Approximation;
  ASSERT_TRUE(request3.containsAll(request2));
}
