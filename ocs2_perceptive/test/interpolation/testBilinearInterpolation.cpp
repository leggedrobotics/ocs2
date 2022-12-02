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

#include <array>
#include <cmath>
#include <utility>

#include <gtest/gtest.h>

#include <ocs2_core/automatic_differentiation/CppAdInterface.h>
#include <ocs2_core/misc/Numerics.h>

#include "ocs2_perceptive/interpolation/BilinearInterpolation.h"

namespace ocs2 {
namespace bilinear_interpolation {

class TestBilinearInterpolation : public ::testing::Test {
 protected:
  using array4_t = std::array<scalar_t, 4>;
  using vector2_t = Eigen::Matrix<scalar_t, 2, 1>;

  TestBilinearInterpolation() {
    auto adBilinearInterpolation = [](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& out) {
      const ad_scalar_t resolution = p(0);
      const ad_vector_t referenceCorner = (ad_vector_t(2) << p(1), p(2)).finished();
      const std::array<ad_scalar_t, 4> cornerValues = {p(3), p(4), p(5), p(6)};

      const ad_vector_t positionRed = (x.head(2) - referenceCorner) / resolution;
      const ad_vector_t positionRedFlip = ad_vector_t::Ones(2) - positionRed;

      out.resize(1);
      out(0) = cornerValues[0] * positionRedFlip(0) * positionRedFlip(1) + cornerValues[1] * positionRed(0) * positionRedFlip(1) +
               cornerValues[2] * positionRedFlip(0) * positionRed(1) + cornerValues[3] * positionRed(0) * positionRed(1);
      out(0) *= x(2);
    };

    cppadInterfacePtr.reset(new CppAdInterface(adBilinearInterpolation, variableDim, parameterDim, "TestBilinearInterpolation"));
    cppadInterfacePtr->createModels(CppAdInterface::ApproximationOrder::First, verbose);
  }

  scalar_t bilinearInterpolation(scalar_t resolution, const vector2_t& referenceCorner, const array4_t& cornerValues,
                                 const vector2_t& position) {
    const vector2_t positionRed = (position - referenceCorner) / resolution;
    const vector2_t positionRedFlip = vector2_t::Ones() - positionRed;
    const scalar_t value = cornerValues[0] * positionRedFlip.x() * positionRedFlip.y() +
                           cornerValues[1] * positionRed.x() * positionRedFlip.y() +
                           cornerValues[2] * positionRedFlip.x() * positionRed.y() + cornerValues[3] * positionRed.x() * positionRed.y();
    return value;
  }

  /** parpam = (resolution, referenceCornerXY, cornerValues[0:3]) */
  vector_t toParameter(scalar_t resolution, const vector2_t& referenceCorner, const array4_t& cornerValues) {
    vector_t p(7);
    p(0) = resolution;
    p(1) = referenceCorner.x();
    p(2) = referenceCorner.y();
    p(3) = cornerValues[0];
    p(4) = cornerValues[1];
    p(5) = cornerValues[2];
    p(6) = cornerValues[3];
    return p;
  }

  static constexpr bool verbose = true;
  static constexpr size_t numSamples = 100;
  static constexpr size_t variableDim = 3;   // (x, y, 1.0)
  static constexpr size_t parameterDim = 7;  // (resolution, referenceCornerXY, cornerValues[0:3])
  static constexpr scalar_t precision = 1e-6;
  static constexpr scalar_t resolution = 0.05;

  std::unique_ptr<CppAdInterface> cppadInterfacePtr;
};

constexpr bool TestBilinearInterpolation::verbose;
constexpr size_t TestBilinearInterpolation::numSamples;
constexpr size_t TestBilinearInterpolation::variableDim;
constexpr size_t TestBilinearInterpolation::parameterDim;
constexpr scalar_t TestBilinearInterpolation::precision;
constexpr scalar_t TestBilinearInterpolation::resolution;

TEST_F(TestBilinearInterpolation, testAdFunction) {
  for (size_t i = 0; i < numSamples; i++) {
    const vector_t randValues = vector_t::Random(parameterDim - 1 + variableDim - 1);
    const vector2_t position(randValues(0), randValues(1));
    const vector2_t referenceCorner(randValues(2), randValues(3));
    const array4_t cornerValues = {randValues(4), randValues(5), randValues(6), randValues(7)};

    const scalar_t trueValue = bilinearInterpolation(resolution, referenceCorner, cornerValues, position);

    const vector_t input = (vector_t(3) << position, 1.0).finished();
    const vector_t param = toParameter(resolution, referenceCorner, cornerValues);
    // true value
    const scalar_t value = cppadInterfacePtr->getFunctionValue(input, param)(0);
    // true linear approximation
    const matrix_t jacAug = cppadInterfacePtr->getJacobian(input, param);
    const std::pair<scalar_t, vector2_t> linApprox = {jacAug(0, 2), vector2_t(jacAug(0, 0), jacAug(0, 1))};

    EXPECT_TRUE(std::abs(trueValue - value) < precision) << "the true value is " << trueValue << " while the value is " << value;
    EXPECT_TRUE(std::abs(trueValue - linApprox.first) < precision)
        << "the true value is " << trueValue << " while the linApprox.first is " << linApprox.first;
  }  // end of i loop
}

TEST_F(TestBilinearInterpolation, testBilinearInterpolation) {
  for (size_t i = 0; i < numSamples; i++) {
    const vector_t randValues = vector_t::Random(parameterDim - 1 + variableDim - 1);
    const vector2_t position(randValues(0), randValues(1));
    const vector2_t referenceCorner(randValues(2), randValues(3));
    const array4_t cornerValues = {randValues(4), randValues(5), randValues(6), randValues(7)};

    const vector_t input = (vector_t(3) << position, 1.0).finished();
    const vector_t param = toParameter(resolution, referenceCorner, cornerValues);
    // true value
    const scalar_t trueValue = cppadInterfacePtr->getFunctionValue(input, param)(0);
    // true linear approximation
    const matrix_t jacAug = cppadInterfacePtr->getJacobian(input, param);
    const std::pair<scalar_t, vector2_t> trueLinApprox = {jacAug(0, 2), vector2_t(jacAug(0, 0), jacAug(0, 1))};

    // value and linear approximation
    const auto value = bilinear_interpolation::getValue(resolution, referenceCorner, cornerValues, position);
    const auto linApprox = bilinear_interpolation::getLinearApproximation(resolution, referenceCorner, cornerValues, position);

    EXPECT_TRUE(std::abs(trueValue - value) < precision) << "the true value is " << trueValue << " while the value is " << value;
    EXPECT_TRUE(std::abs(trueValue - linApprox.first) < precision)
        << "the true value is " << trueValue << " while linApprox.first is " << linApprox.first;
    EXPECT_TRUE(trueLinApprox.second.isApprox(linApprox.second, precision));
  }  // end of i loop
}

}  // namespace bilinear_interpolation
}  // namespace ocs2
