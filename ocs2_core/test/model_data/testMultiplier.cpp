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

#include <numeric>

#include <gtest/gtest.h>

#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_core/model_data/Multiplier.h>

namespace ocs2 {
namespace {

/**
 * Linearly interpolates a trajectory of MultiplierCollections.
 *
 * @param [in] indexAlpha : index and interpolation coefficient (alpha) pair.
 * @param [in] dataArray : A trajectory of MultiplierCollections.
 * @return The interpolated MultiplierCollection at indexAlpha.
 */
inline MultiplierCollection interpolateNew(const LinearInterpolation::index_alpha_t& indexAlpha,
                                           const std::vector<MultiplierCollection>& dataArray) {
  assert(dataArray.size() > 0);
  if (dataArray.size() > 1) {
    // Normal interpolation case
    const int index = indexAlpha.first;
    const scalar_t alpha = indexAlpha.second;
    bool areSameSize = true;

    const auto lhs_stateEq = toVector(dataArray[index].stateEq);
    const auto rhs_stateEq = toVector(dataArray[index + 1].stateEq);
    areSameSize = areSameSize && (lhs_stateEq.size() == rhs_stateEq.size());

    const auto lhs_stateIneq = toVector(dataArray[index].stateIneq);
    const auto rhs_stateIneq = toVector(dataArray[index + 1].stateIneq);
    areSameSize = areSameSize && (lhs_stateIneq.size() == rhs_stateIneq.size());

    const auto lhs_stateInputEq = toVector(dataArray[index].stateInputEq);
    const auto rhs_stateInputEq = toVector(dataArray[index + 1].stateInputEq);
    areSameSize = areSameSize && (lhs_stateInputEq.size() == rhs_stateInputEq.size());

    const auto lhs_stateInputIneq = toVector(dataArray[index].stateInputIneq);
    const auto rhs_stateInputIneq = toVector(dataArray[index + 1].stateInputIneq);
    areSameSize = areSameSize && (lhs_stateInputIneq.size() == rhs_stateInputIneq.size());

    if (areSameSize) {
      const auto f = [alpha](const vector_t& lhs, const vector_t& rhs) -> vector_t { return alpha * lhs + (scalar_t(1.0) - alpha) * rhs; };
      MultiplierCollection out;
      out.stateEq = toMultipliers(getSizes(dataArray[index].stateEq), f(lhs_stateEq, rhs_stateEq));
      out.stateIneq = toMultipliers(getSizes(dataArray[index].stateIneq), f(lhs_stateIneq, rhs_stateIneq));
      out.stateInputEq = toMultipliers(getSizes(dataArray[index].stateInputEq), f(lhs_stateInputEq, rhs_stateInputEq));
      out.stateInputIneq = toMultipliers(getSizes(dataArray[index].stateInputIneq), f(lhs_stateInputIneq, rhs_stateInputIneq));
      return out;

    } else {
      return (alpha > 0.5) ? dataArray[index] : dataArray[index + 1];
    }
  } else {  // dataArray.size() == 1
    // Time vector has only 1 element -> Constant function
    return dataArray[0];
  }
}

void random(const size_array_t& termsSize, vector_t& serializedMultiplier, std::vector<Multiplier>& multiplier) {
  const size_t length = std::accumulate(termsSize.begin(), termsSize.end(), termsSize.size());
  serializedMultiplier = vector_t::Random(length);
  multiplier = toMultipliers(termsSize, serializedMultiplier);
}

bool isApprox(const MultiplierCollection& lhs, const MultiplierCollection& rhs, scalar_t prec) {
  bool flag = toVector(lhs.stateEq).isApprox(toVector(rhs.stateEq), prec);
  flag = flag && toVector(lhs.stateIneq).isApprox(toVector(rhs.stateIneq), prec);
  flag = flag && toVector(lhs.stateInputEq).isApprox(toVector(rhs.stateInputEq), prec);
  flag = flag && toVector(lhs.stateInputIneq).isApprox(toVector(rhs.stateInputIneq), prec);
  return flag;
}

}  // unnamed namespace
}  // namespace ocs2

TEST(TestMultiplier, testSerialization) {
  const ocs2::size_array_t termsSize{0, 2, 0, 0, 3, 5};
  const size_t numConstraint = termsSize.size();
  const size_t length = std::accumulate(termsSize.begin(), termsSize.end(), numConstraint);

  const ocs2::vector_t randomVec = ocs2::vector_t::Random(length);
  const std::vector<ocs2::Multiplier> termsMultiplier = ocs2::toMultipliers(termsSize, randomVec);
  const ocs2::vector_t serializedMultiplier = ocs2::toVector(termsMultiplier);

  EXPECT_TRUE(serializedMultiplier == randomVec);
  EXPECT_TRUE(serializedMultiplier.size() == length);
  EXPECT_TRUE(getSizes(termsMultiplier) == termsSize);
}

TEST(TestMultiplier, testSwap) {
  const ocs2::size_array_t termsSize{0, 2, 0, 0, 3, 5};

  ocs2::vector_t serializedStateEq, serializedStateIneq, serializedStateInputEq, serializedStateInputIneq;

  ocs2::MultiplierCollection termsMultiplier;
  ocs2::random(termsSize, serializedStateEq, termsMultiplier.stateEq);
  ocs2::random(termsSize, serializedStateIneq, termsMultiplier.stateIneq);
  ocs2::random(termsSize, serializedStateInputEq, termsMultiplier.stateInputEq);
  ocs2::random(termsSize, serializedStateInputIneq, termsMultiplier.stateInputIneq);

  auto termsMultiplierRef = termsMultiplier;
  ocs2::MultiplierCollection termsMultiplierNew;
  termsMultiplier.swap(termsMultiplierNew);

  EXPECT_TRUE(ocs2::isApprox(termsMultiplierNew, termsMultiplierRef, 1e-10));

  termsMultiplierRef.clear();
  EXPECT_TRUE(ocs2::isApprox(termsMultiplier, termsMultiplierRef, 1e-10));
}

TEST(TestMultiplier, testInterpolation) {
  // terms size
  const ocs2::size_array_t stateEqTermsSize{0, 0};
  const ocs2::size_array_t stateIneqTermsSize{2};
  const ocs2::size_array_t stateInputEqTermsSize{};
  const ocs2::size_array_t stateInputIneqTermsSize{0, 2, 0, 0, 3, 5};

  // construct trajectories of length N
  const size_t N = 11;
  ocs2::scalar_array_t timeTrajectory(N);
  ocs2::vector_array_t stateEqTrajectory(N);
  ocs2::vector_array_t stateIneqTrajectory(N);
  ocs2::vector_array_t stateInputEqTrajectory(N);
  ocs2::vector_array_t stateInputIneqTrajectory(N);
  std::vector<ocs2::MultiplierCollection> multiplierCollectionTrajectory(N);
  for (size_t i = 0; i < N; i++) {
    timeTrajectory[i] = i * 0.1;
    ocs2::random(stateEqTermsSize, stateEqTrajectory[i], multiplierCollectionTrajectory[i].stateEq);
    ocs2::random(stateIneqTermsSize, stateIneqTrajectory[i], multiplierCollectionTrajectory[i].stateIneq);
    ocs2::random(stateInputEqTermsSize, stateInputEqTrajectory[i], multiplierCollectionTrajectory[i].stateInputEq);
    ocs2::random(stateInputIneqTermsSize, stateInputIneqTrajectory[i], multiplierCollectionTrajectory[i].stateInputIneq);
  }  // end of i loop

  constexpr ocs2::scalar_t prec = 1e-8;
  const ocs2::scalar_array_t timeTrajectoryTest{-1.0, 0.0, 4.0, 4.6, 8.7, 10.0, 11.0, 100.0};
  for (size_t i = 0; i < timeTrajectoryTest.size(); i++) {
    const auto indexAlpha = ocs2::LinearInterpolation::timeSegment(timeTrajectoryTest[i], timeTrajectory);

    const auto stateEq = ocs2::LinearInterpolation::interpolate(indexAlpha, stateEqTrajectory);
    const auto stateIneq = ocs2::LinearInterpolation::interpolate(indexAlpha, stateIneqTrajectory);
    const auto stateInputEq = ocs2::LinearInterpolation::interpolate(indexAlpha, stateInputEqTrajectory);
    const auto stateInputIneq = ocs2::LinearInterpolation::interpolate(indexAlpha, stateInputIneqTrajectory);
    const auto multiplierCollection = ocs2::LinearInterpolation::interpolate(indexAlpha, multiplierCollectionTrajectory);
    const auto multiplierCollectionNew = ocs2::interpolateNew(indexAlpha, multiplierCollectionTrajectory);

    EXPECT_TRUE(stateEq.isApprox(toVector(multiplierCollection.stateEq), prec));
    EXPECT_TRUE(stateIneq.isApprox(toVector(multiplierCollection.stateIneq), prec));
    EXPECT_TRUE(stateInputEq.isApprox(toVector(multiplierCollection.stateInputEq), prec));
    EXPECT_TRUE(stateInputIneq.isApprox(toVector(multiplierCollection.stateInputIneq), prec));
    EXPECT_TRUE(ocs2::isApprox(multiplierCollection, multiplierCollectionNew, prec));
  }  // end of i loop
}
