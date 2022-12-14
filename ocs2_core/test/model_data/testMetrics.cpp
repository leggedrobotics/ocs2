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
 * Linearly interpolates a trajectory of Metricss.
 *
 * @param [in] indexAlpha : index and interpolation coefficient (alpha) pair.
 * @param [in] dataArray : A trajectory of Metricss.
 * @return The interpolated Metrics at indexAlpha.
 */
inline Metrics interpolateNew(const LinearInterpolation::index_alpha_t& indexAlpha,
                                        const std::vector<Metrics>& dataArray) {
  assert(dataArray.size() > 0);
  if (dataArray.size() > 1) {
    // Normal interpolation case
    const int index = indexAlpha.first;
    const scalar_t alpha = indexAlpha.second;
    bool areSameSize = true;

    const auto lhs_stateEqConst = toVector(dataArray[index].stateEqConstraint);
    const auto rhs_stateEqConst = toVector(dataArray[index + 1].stateEqConstraint);
    areSameSize = areSameSize && (lhs_stateEqConst.size() == rhs_stateEqConst.size());

    const auto lhs_stateInputEqConst = toVector(dataArray[index].stateInputEqConstraint);
    const auto rhs_stateInputEqConst = toVector(dataArray[index + 1].stateInputEqConstraint);
    areSameSize = areSameSize && (lhs_stateInputEqConst.size() == rhs_stateInputEqConst.size());

    const auto lhs_stateIneqConst = toVector(dataArray[index].stateIneqConstraint);
    const auto rhs_stateIneqConst = toVector(dataArray[index + 1].stateIneqConstraint);
    areSameSize = areSameSize && (lhs_stateIneqConst.size() == rhs_stateIneqConst.size());

    const auto lhs_stateInputIneqConst = toVector(dataArray[index].stateInputIneqConstraint);
    const auto rhs_stateInputIneqConst = toVector(dataArray[index + 1].stateInputIneqConstraint);
    areSameSize = areSameSize && (lhs_stateInputIneqConst.size() == rhs_stateInputIneqConst.size());

    const auto lhs_stateEqLag = toVector(dataArray[index].stateEqLagrangian);
    const auto rhs_stateEqLag = toVector(dataArray[index + 1].stateEqLagrangian);
    areSameSize = areSameSize && (lhs_stateEqLag.size() == rhs_stateEqLag.size());

    const auto lhs_stateIneqLag = toVector(dataArray[index].stateIneqLagrangian);
    const auto rhs_stateIneqLag = toVector(dataArray[index + 1].stateIneqLagrangian);
    areSameSize = areSameSize && (lhs_stateIneqLag.size() == rhs_stateIneqLag.size());

    const auto lhs_stateInputEqLag = toVector(dataArray[index].stateInputEqLagrangian);
    const auto rhs_stateInputEqLag = toVector(dataArray[index + 1].stateInputEqLagrangian);
    areSameSize = areSameSize && (lhs_stateInputEqLag.size() == rhs_stateInputEqLag.size());

    const auto lhs_stateInputIneqLag = toVector(dataArray[index].stateInputIneqLagrangian);
    const auto rhs_stateInputIneqLag = toVector(dataArray[index + 1].stateInputIneqLagrangian);
    areSameSize = areSameSize && (lhs_stateInputIneqLag.size() == rhs_stateInputIneqLag.size());

    if (areSameSize) {
      const auto f = [alpha](const vector_t& lhs, const vector_t& rhs) -> vector_t { return alpha * lhs + (scalar_t(1.0) - alpha) * rhs; };
      Metrics out;
      // cost
      out.cost = LinearInterpolation::interpolate(
          indexAlpha, dataArray, [](const std::vector<Metrics>& array, size_t t) -> const scalar_t& { return array[t].cost; });
      // dynamics violation
      out.dynamicsViolation = LinearInterpolation::interpolate(
          indexAlpha, dataArray, [](const std::vector<Metrics>& array, size_t t) -> const vector_t& { return array[t].dynamicsViolation; });
      // equality constraints
      out.stateEqConstraint = toConstraintArray(getSizes(dataArray[index].stateEqConstraint), f(lhs_stateEqConst, rhs_stateEqConst));
      out.stateInputEqConstraint = toConstraintArray(getSizes(dataArray[index].stateInputEqConstraint), f(lhs_stateInputEqConst, rhs_stateInputEqConst));
      // inequality constraints
      out.stateIneqConstraint = toConstraintArray(getSizes(dataArray[index].stateIneqConstraint), f(lhs_stateIneqConst, rhs_stateIneqConst));
      out.stateInputIneqConstraint = toConstraintArray(getSizes(dataArray[index].stateInputIneqConstraint), f(lhs_stateInputIneqConst, rhs_stateInputIneqConst));
      // lagrangian
      out.stateEqLagrangian = toLagrangianMetrics(getSizes(dataArray[index].stateEqLagrangian), f(lhs_stateEqLag, rhs_stateEqLag));
      out.stateIneqLagrangian = toLagrangianMetrics(getSizes(dataArray[index].stateIneqLagrangian), f(lhs_stateIneqLag, rhs_stateIneqLag));
      out.stateInputEqLagrangian =
          toLagrangianMetrics(getSizes(dataArray[index].stateInputEqLagrangian), f(lhs_stateInputEqLag, rhs_stateInputEqLag));
      out.stateInputIneqLagrangian =
          toLagrangianMetrics(getSizes(dataArray[index].stateInputIneqLagrangian), f(lhs_stateInputIneqLag, rhs_stateInputIneqLag));
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

void random(const size_array_t& termsSize, vector_array_t& constraintArray) {
  const size_t length = std::accumulate(termsSize.begin(), termsSize.end(), termsSize.size());
  const vector_t serializedConstraintArray = vector_t::Random(length);
  constraintArray = toConstraintArray(termsSize, serializedConstraintArray);
}

}  // unnamed namespace
}  // namespace ocs2

TEST(TestMetrics, testSerialization) {
  const ocs2::size_array_t termsSize{0, 2, 0, 0, 3, 5};
  const size_t numConstraint = termsSize.size();
  const size_t length = std::accumulate(termsSize.begin(), termsSize.end(), numConstraint);

  const ocs2::vector_t randomVec = ocs2::vector_t::Random(length);
  const std::vector<ocs2::LagrangianMetrics> l = ocs2::toLagrangianMetrics(termsSize, randomVec);
  const ocs2::vector_t serialized_l = ocs2::toVector(l);

  EXPECT_TRUE(serialized_l == randomVec);
  EXPECT_TRUE(serialized_l.size() == length);
  EXPECT_TRUE(getSizes(l) == termsSize);
}

TEST(TestMetrics, testSwap) {
  const ocs2::size_array_t termsSize{0, 2, 0, 0, 3, 5};

  ocs2::Metrics termsMetrics;
  termsMetrics.cost = ocs2::vector_t::Random(1)(0);
  termsMetrics.dynamicsViolation.setRandom(2);
  ocs2::random(termsSize, termsMetrics.stateEqConstraint);
  ocs2::random(termsSize, termsMetrics.stateInputEqConstraint);
  ocs2::random(termsSize, termsMetrics.stateIneqConstraint);
  ocs2::random(termsSize, termsMetrics.stateInputIneqConstraint);
  ocs2::random(termsSize, termsMetrics.stateEqLagrangian);
  ocs2::random(termsSize, termsMetrics.stateIneqLagrangian);
  ocs2::random(termsSize, termsMetrics.stateInputEqLagrangian);
  ocs2::random(termsSize, termsMetrics.stateInputIneqLagrangian);

  auto termsMetricsRef = termsMetrics;
  ocs2::Metrics termsMetricsNew;
  termsMetrics.swap(termsMetricsNew);

  EXPECT_TRUE(termsMetricsNew.isApprox(termsMetricsRef, 1e-10));

  termsMetricsRef.clear();
  EXPECT_TRUE(termsMetrics.isApprox(termsMetricsRef, 1e-10));
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
  std::vector<ocs2::Metrics> metricsTrajectory(N);
  for (size_t i = 0; i < N; i++) {
    timeTrajectory[i] = i * 0.1;
    metricsTrajectory[i].cost = ocs2::vector_t::Random(1)(0);
    metricsTrajectory[i].dynamicsViolation.setRandom(2);
    ocs2::random(stateEqTermsSize, metricsTrajectory[i].stateEqConstraint);
    ocs2::random(stateInputEqTermsSize, metricsTrajectory[i].stateInputEqConstraint);
    ocs2::random(stateIneqTermsSize, metricsTrajectory[i].stateIneqConstraint);
    ocs2::random(stateInputIneqTermsSize, metricsTrajectory[i].stateInputIneqConstraint);
    ocs2::random(stateEqTermsSize, metricsTrajectory[i].stateEqLagrangian);
    ocs2::random(stateIneqTermsSize, metricsTrajectory[i].stateIneqLagrangian);
    ocs2::random(stateInputEqTermsSize, metricsTrajectory[i].stateInputEqLagrangian);
    ocs2::random(stateInputIneqTermsSize, metricsTrajectory[i].stateInputIneqLagrangian);
  }  // end of i loop

  constexpr ocs2::scalar_t prec = 1e-8;
  const ocs2::scalar_array_t timeTrajectoryTest{-1.0, 0.0, 4.0, 4.6, 8.7, 10.0, 11.0, 100.0};
  for (size_t i = 0; i < timeTrajectoryTest.size(); i++) {
    const auto indexAlpha = ocs2::LinearInterpolation::timeSegment(timeTrajectoryTest[i], timeTrajectory);

    const auto metrics = ocs2::LinearInterpolation::interpolate(indexAlpha, metricsTrajectory);
    const auto metricsNew = ocs2::interpolateNew(indexAlpha, metricsTrajectory);

    EXPECT_TRUE(metrics.isApprox(metricsNew, prec));
  }  // end of i loop
}
