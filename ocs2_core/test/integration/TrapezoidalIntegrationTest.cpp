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

#include <algorithm>

#include <ocs2_core/Types.h>
#include <ocs2_core/integration/TrapezoidalIntegration.h>

TEST(TrapezoidalIntegrationTest, Rectangle) {
  const ocs2::scalar_t width = 8.0;
  const ocs2::scalar_t height = 2.0;
  const size_t numIntervals = 100;
  const ocs2::scalar_t dx = width / numIntervals;

  ocs2::scalar_array_t xTrj(numIntervals + 1, 0.0);
  auto x = -dx;
  std::generate(xTrj.begin(), xTrj.end(), [&x, dx]() { return (x += dx); });

  ocs2::scalar_array_t yTrj(numIntervals + 1, height);

  const auto area = ocs2::trapezoidalIntegration(xTrj, yTrj);
  ASSERT_NEAR(area, width * height, 1e-12);
}

TEST(TrapezoidalIntegrationTest, RightTriangle) {
  const ocs2::scalar_t width = 8.0;
  const ocs2::scalar_t height = 2.0;
  const size_t numIntervals = 10;
  const ocs2::scalar_t dx = width / numIntervals;
  const ocs2::scalar_t dy = height / numIntervals;

  ocs2::scalar_array_t xTrj(numIntervals + 1, 0.0);
  auto x = -dx;
  std::generate(xTrj.begin(), xTrj.end(), [&x, dx]() { return (x += dx); });

  ocs2::scalar_array_t yTrj(numIntervals + 1, 0.0);
  auto y = -dy;
  std::generate(yTrj.begin(), yTrj.end(), [&y, dy]() { return (y += dy); });

  const auto area = ocs2::trapezoidalIntegration(xTrj, yTrj);
  ASSERT_NEAR(area, width * height / 2.0, 1e-12);
}
