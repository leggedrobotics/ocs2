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

#include "ocs2_perceptive/interpolation/TrilinearInterpolation.h"

namespace ocs2 {
namespace trilinear_interpolation {

class TestTrilinearInterpolation : public ::testing::Test {
 protected:
  using array8_t = std::array<scalar_t, 8>;
  using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;

  TestTrilinearInterpolation() {
    auto adTrilinearInterpolation = [](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& out) {
      const ad_scalar_t resolution = p(0);
      const ad_vector_t referenceCorner = (ad_vector_t(3) << p(1), p(2), p(3)).finished();
      const std::array<ad_scalar_t, 8> cornerValues = {p(4), p(5), p(6), p(7), p(8), p(9), p(10), p(11)};

      const ad_vector_t positionRed = (x.head(3) - referenceCorner) / resolution;
      const ad_vector_t positionRedFlip = ad_vector_t::Ones(3) - positionRed;

      const ad_scalar_t v00 = positionRedFlip(0) * cornerValues[0] + positionRed(0) * cornerValues[1];
      const ad_scalar_t v10 = positionRedFlip(0) * cornerValues[2] + positionRed(0) * cornerValues[3];
      const ad_scalar_t v01 = positionRedFlip(0) * cornerValues[4] + positionRed(0) * cornerValues[5];
      const ad_scalar_t v11 = positionRedFlip(0) * cornerValues[6] + positionRed(0) * cornerValues[7];
      const ad_scalar_t v0 = positionRedFlip(1) * v00 + positionRed(1) * v10;
      const ad_scalar_t v1 = positionRedFlip(1) * v01 + positionRed(1) * v11;

      out.resize(1);
      out(0) = positionRedFlip(2) * v0 + positionRed(2) * v1;
      out(0) *= x(3);
    };

    cppadInterfacePtr.reset(new CppAdInterface(adTrilinearInterpolation, variableDim, parameterDim, "TestTrilinearInterpolation"));
    cppadInterfacePtr->createModels(CppAdInterface::ApproximationOrder::First, verbose);
  }

  scalar_t trilinearInterpolation(scalar_t resolution, const vector3_t& referenceCorner, const array8_t& cornerValues,
                                  const vector3_t& position) {
    const vector3_t positionRed = (position - referenceCorner) / resolution;
    const vector3_t positionRedFlip = vector3_t::Ones() - positionRed;

    const scalar_t v00 = positionRedFlip(0) * cornerValues[0] + positionRed(0) * cornerValues[1];
    const scalar_t v10 = positionRedFlip(0) * cornerValues[2] + positionRed(0) * cornerValues[3];
    const scalar_t v01 = positionRedFlip(0) * cornerValues[4] + positionRed(0) * cornerValues[5];
    const scalar_t v11 = positionRedFlip(0) * cornerValues[6] + positionRed(0) * cornerValues[7];
    const scalar_t v0 = positionRedFlip(1) * v00 + positionRed(1) * v10;
    const scalar_t v1 = positionRedFlip(1) * v01 + positionRed(1) * v11;

    const scalar_t value = positionRedFlip(2) * v0 + positionRed(2) * v1;

    return value;
  }

  /** param = (resolution, referenceCornerXYZ, cornerValues[0:7]) */
  vector_t toParameter(scalar_t resolution, const vector3_t& referenceCorner, const array8_t& cornerValues) {
    vector_t p(12);
    p(0) = resolution;
    p(1) = referenceCorner.x();
    p(2) = referenceCorner.y();
    p(3) = referenceCorner.z();
    p(4) = cornerValues[0];   // f_000
    p(5) = cornerValues[1];   // f_100
    p(6) = cornerValues[2];   // f_010
    p(7) = cornerValues[3];   // f_110
    p(8) = cornerValues[4];   // f_001
    p(9) = cornerValues[5];   // f_101
    p(10) = cornerValues[6];  // f_011
    p(11) = cornerValues[7];  // f_111
    return p;
  }

  static constexpr bool verbose = true;
  static constexpr size_t numSamples = 100;
  static constexpr size_t variableDim = 4;    // (x, y, z, 1.0)
  static constexpr size_t parameterDim = 12;  // (resolution, referenceCornerXYZ, cornerValues[0:7])
  static constexpr scalar_t precision = 1e-6;
  static constexpr scalar_t resolution = 0.05;

  std::unique_ptr<CppAdInterface> cppadInterfacePtr;
};

constexpr bool TestTrilinearInterpolation::verbose;
constexpr size_t TestTrilinearInterpolation::numSamples;
constexpr size_t TestTrilinearInterpolation::variableDim;
constexpr size_t TestTrilinearInterpolation::parameterDim;
constexpr scalar_t TestTrilinearInterpolation::precision;
constexpr scalar_t TestTrilinearInterpolation::resolution;

TEST_F(TestTrilinearInterpolation, testAdFunction) {
  for (size_t i = 0; i < numSamples; i++) {
    const vector_t randValues = vector_t::Random(parameterDim - 1 + variableDim - 1);
    const vector3_t position(randValues(0), randValues(1), randValues(2));
    const vector3_t referenceCorner(randValues(3), randValues(4), randValues(5));
    const array8_t cornerValues = {randValues(6),  randValues(7),  randValues(8),  randValues(9),
                                   randValues(10), randValues(11), randValues(12), randValues(13)};

    const scalar_t trueValue = trilinearInterpolation(resolution, referenceCorner, cornerValues, position);

    const vector_t input = (vector_t(4) << position, 1.0).finished();
    const vector_t param = toParameter(resolution, referenceCorner, cornerValues);
    // true value
    const scalar_t value = cppadInterfacePtr->getFunctionValue(input, param)(0);
    // true linear approximation
    const matrix_t jacAug = cppadInterfacePtr->getJacobian(input, param);
    const std::pair<scalar_t, vector3_t> linApprox = {jacAug(0, 3), vector3_t(jacAug(0, 0), jacAug(0, 1), jacAug(0, 2))};

    EXPECT_TRUE(std::abs(trueValue - value) < precision) << "the true value is " << trueValue << " while the value is " << value;
    EXPECT_TRUE(std::abs(trueValue - linApprox.first) < precision)
        << "the true value is " << trueValue << " while the linApprox.first is " << linApprox.first;
  }  // end of i loop
}

TEST_F(TestTrilinearInterpolation, testTrilinearInterpolation) {
  for (size_t i = 0; i < numSamples; i++) {
    const vector_t randValues = vector_t::Random(parameterDim - 1 + variableDim - 1);
    const vector3_t position(randValues(0), randValues(1), randValues(2));
    const vector3_t referenceCorner(randValues(3), randValues(4), randValues(5));
    const array8_t cornerValues = {randValues(6),  randValues(7),  randValues(8),  randValues(9),
                                   randValues(10), randValues(11), randValues(12), randValues(13)};

    const vector_t input = (vector_t(4) << position, 1.0).finished();
    const vector_t param = toParameter(resolution, referenceCorner, cornerValues);
    // true value
    const scalar_t trueValue = cppadInterfacePtr->getFunctionValue(input, param)(0);
    // true linear approximation
    const matrix_t jacAug = cppadInterfacePtr->getJacobian(input, param);
    const std::pair<scalar_t, vector3_t> trueLinApprox = {jacAug(0, 3), vector3_t(jacAug(0, 0), jacAug(0, 1), jacAug(0, 2))};

    // value and linear approximation
    const auto value = trilinear_interpolation::getValue(resolution, referenceCorner, cornerValues, position);
    const auto linApprox = trilinear_interpolation::getLinearApproximation(resolution, referenceCorner, cornerValues, position);

    EXPECT_TRUE(std::abs(trueValue - value) < precision) << "the true value is " << trueValue << " while the value is " << value;
    EXPECT_TRUE(std::abs(trueValue - linApprox.first) < precision)
        << "the true value is " << trueValue << " while linApprox.first is " << linApprox.first;
    EXPECT_TRUE(trueLinApprox.second.isApprox(linApprox.second, precision))
        << "the true value is (" << trueLinApprox.second.transpose() << ") while linApprox.second is (" << linApprox.second.transpose()
        << ") ";
  }  // end of i loop
}

}  // namespace trilinear_interpolation
}  // namespace ocs2
