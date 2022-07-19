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

#include <cmath>
#include <numeric>

#include <gtest/gtest.h>

#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_core/model_data/Metrics.h>

namespace ocs2 {
namespace {

/**
 * Linearly interpolates a trajectory of MetricsCollections.
 *
 * @param [in] indexAlpha : index and interpolation coefficient (alpha) pair.
 * @param [in] dataArray : A trajectory of MetricsCollections.
 * @return The interpolated MetricsCollection at indexAlpha.
 */
inline MetricsCollection interpolateNew(const LinearInterpolation::index_alpha_t& indexAlpha,
                                        const std::vector<MetricsCollection>& dataArray) {
  assert(dataArray.size() > 0);
  if (dataArray.size() > 1) {
    // Normal interpolation case
    const int index = indexAlpha.first;
    const scalar_t alpha = indexAlpha.second;
    bool areSameSize = true;

    const auto lhs_stateEq = toVector(dataArray[index].stateEqLagrangian);
    const auto rhs_stateEq = toVector(dataArray[index + 1].stateEqLagrangian);
    areSameSize = areSameSize && (lhs_stateEq.size() == rhs_stateEq.size());

    const auto lhs_stateIneq = toVector(dataArray[index].stateIneqLagrangian);
    const auto rhs_stateIneq = toVector(dataArray[index + 1].stateIneqLagrangian);
    areSameSize = areSameSize && (lhs_stateIneq.size() == rhs_stateIneq.size());

    const auto lhs_stateInputEq = toVector(dataArray[index].stateInputEqLagrangian);
    const auto rhs_stateInputEq = toVector(dataArray[index + 1].stateInputEqLagrangian);
    areSameSize = areSameSize && (lhs_stateInputEq.size() == rhs_stateInputEq.size());

    const auto lhs_stateInputIneq = toVector(dataArray[index].stateInputIneqLagrangian);
    const auto rhs_stateInputIneq = toVector(dataArray[index + 1].stateInputIneqLagrangian);
    areSameSize = areSameSize && (lhs_stateInputIneq.size() == rhs_stateInputIneq.size());

    if (areSameSize) {
      const auto f = [alpha](const vector_t& lhs, const vector_t& rhs) -> vector_t { return alpha * lhs + (scalar_t(1.0) - alpha) * rhs; };
      MetricsCollection out;
      // cost
      out.cost = LinearInterpolation::interpolate(
          indexAlpha, dataArray, [](const std::vector<MetricsCollection>& array, size_t t) -> const scalar_t& { return array[t].cost; });
      // constraints
      out.stateEqConstraint = LinearInterpolation::interpolate(
          indexAlpha, dataArray,
          [](const std::vector<MetricsCollection>& array, size_t t) -> const vector_t& { return array[t].stateEqConstraint; });
      out.stateInputEqConstraint = LinearInterpolation::interpolate(
          indexAlpha, dataArray,
          [](const std::vector<MetricsCollection>& array, size_t t) -> const vector_t& { return array[t].stateInputEqConstraint; });
      out.stateEqLagrangian = toLagrangianMetrics(getSizes(dataArray[index].stateEqLagrangian), f(lhs_stateEq, rhs_stateEq));
      out.stateIneqLagrangian = toLagrangianMetrics(getSizes(dataArray[index].stateIneqLagrangian), f(lhs_stateIneq, rhs_stateIneq));
      out.stateInputEqLagrangian =
          toLagrangianMetrics(getSizes(dataArray[index].stateInputEqLagrangian), f(lhs_stateInputEq, rhs_stateInputEq));
      out.stateInputIneqLagrangian =
          toLagrangianMetrics(getSizes(dataArray[index].stateInputIneqLagrangian), f(lhs_stateInputIneq, rhs_stateInputIneq));
      return out;

    } else {
      return (alpha > 0.5) ? dataArray[index] : dataArray[index + 1];
    }
  } else {  // dataArray.size() == 1
    // Time vector has only 1 element -> Constant function
    return dataArray[0];
  }
}

void random(const size_array_t& termsSize, std::vector<LagrangianMetrics>& lagrangianMetrics) {
  const size_t length = std::accumulate(termsSize.begin(), termsSize.end(), termsSize.size());
  const vector_t serializedLagrangianMetrics = vector_t::Random(length);
  lagrangianMetrics = toLagrangianMetrics(termsSize, serializedLagrangianMetrics);
}

bool isApprox(const MetricsCollection& lhs, const MetricsCollection& rhs, scalar_t prec) {
  bool flag = std::abs(lhs.cost - rhs.cost) < prec;
  flag = flag && lhs.stateEqConstraint.isApprox(rhs.stateEqConstraint, prec);
  flag = flag && lhs.stateInputEqConstraint.isApprox(rhs.stateInputEqConstraint, prec);
  flag = flag && toVector(lhs.stateEqLagrangian).isApprox(toVector(rhs.stateEqLagrangian), prec);
  flag = flag && toVector(lhs.stateIneqLagrangian).isApprox(toVector(rhs.stateIneqLagrangian), prec);
  flag = flag && toVector(lhs.stateInputEqLagrangian).isApprox(toVector(rhs.stateInputEqLagrangian), prec);
  flag = flag && toVector(lhs.stateInputIneqLagrangian).isApprox(toVector(rhs.stateInputIneqLagrangian), prec);
  return flag;
}

}  // unnamed namespace
}  // namespace ocs2

TEST(TestMetrics, testSerialization) {
  const ocs2::size_array_t termsSize{0, 2, 0, 0, 3, 5};
  const size_t numConstraint = termsSize.size();
  const size_t length = std::accumulate(termsSize.begin(), termsSize.end(), numConstraint);

  const ocs2::vector_t randomVec = ocs2::vector_t::Random(length);
  const std::vector<ocs2::LagrangianMetrics> termsMetricsCollection = ocs2::toLagrangianMetrics(termsSize, randomVec);
  const ocs2::vector_t serializedMetricsCollection = ocs2::toVector(termsMetricsCollection);

  EXPECT_TRUE(serializedMetricsCollection == randomVec);
  EXPECT_TRUE(serializedMetricsCollection.size() == length);
  EXPECT_TRUE(getSizes(termsMetricsCollection) == termsSize);
}

TEST(TestMetrics, testSwap) {
  const ocs2::size_array_t termsSize{0, 2, 0, 0, 3, 5};

  ocs2::MetricsCollection termsMetricsCollection;
  termsMetricsCollection.cost = ocs2::vector_t::Random(1)(0);
  termsMetricsCollection.stateEqConstraint = ocs2::vector_t::Random(2);
  termsMetricsCollection.stateInputEqConstraint = ocs2::vector_t::Random(3);
  ocs2::random(termsSize, termsMetricsCollection.stateEqLagrangian);
  ocs2::random(termsSize, termsMetricsCollection.stateIneqLagrangian);
  ocs2::random(termsSize, termsMetricsCollection.stateInputEqLagrangian);
  ocs2::random(termsSize, termsMetricsCollection.stateInputIneqLagrangian);

  auto termsMetricsCollectionRef = termsMetricsCollection;
  ocs2::MetricsCollection termsMultiplierNew;
  termsMetricsCollection.swap(termsMultiplierNew);

  EXPECT_TRUE(ocs2::isApprox(termsMultiplierNew, termsMetricsCollectionRef, 1e-10));

  termsMetricsCollectionRef.clear();
  EXPECT_TRUE(ocs2::isApprox(termsMetricsCollection, termsMetricsCollectionRef, 1e-10));
}

TEST(TestMetrics, testInterpolation) {
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
  std::vector<ocs2::MetricsCollection> metricsCollectionTrajectory(N);
  for (size_t i = 0; i < N; i++) {
    timeTrajectory[i] = i * 0.1;
    metricsCollectionTrajectory[i].cost = ocs2::vector_t::Random(1)(0);
    metricsCollectionTrajectory[i].stateEqConstraint = ocs2::vector_t::Random(2);
    metricsCollectionTrajectory[i].stateInputEqConstraint = ocs2::vector_t::Random(3);
    ocs2::random(stateEqTermsSize, metricsCollectionTrajectory[i].stateEqLagrangian);
    ocs2::random(stateIneqTermsSize, metricsCollectionTrajectory[i].stateIneqLagrangian);
    ocs2::random(stateInputEqTermsSize, metricsCollectionTrajectory[i].stateInputEqLagrangian);
    ocs2::random(stateInputIneqTermsSize, metricsCollectionTrajectory[i].stateInputIneqLagrangian);
  }  // end of i loop

  constexpr ocs2::scalar_t prec = 1e-8;
  const ocs2::scalar_array_t timeTrajectoryTest{-1.0, 0.0, 4.0, 4.6, 8.7, 10.0, 11.0, 100.0};
  for (size_t i = 0; i < timeTrajectoryTest.size(); i++) {
    const auto indexAlpha = ocs2::LinearInterpolation::timeSegment(timeTrajectoryTest[i], timeTrajectory);

    const auto metricsCollection = ocs2::LinearInterpolation::interpolate(indexAlpha, metricsCollectionTrajectory);
    const auto metricsCollectionNew = ocs2::interpolateNew(indexAlpha, metricsCollectionTrajectory);

    EXPECT_TRUE(ocs2::isApprox(metricsCollection, metricsCollectionNew, prec));
  }  // end of i loop
}
