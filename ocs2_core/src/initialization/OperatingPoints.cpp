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

#include <ocs2_core/initialization/OperatingPoints.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_core/misc/Numerics.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
OperatingPoints::OperatingPoints(const vector_t& stateOperatingPoint, const vector_t& inputOperatingPoint)
    : timeTrajectory_(1, 0.0), stateTrajectory_(1, stateOperatingPoint), inputTrajectory_(1, inputOperatingPoint) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
OperatingPoints::OperatingPoints(scalar_array_t timeTrajectory, vector_array_t stateTrajectory, vector_array_t inputTrajectory)
    : timeTrajectory_(std::move(timeTrajectory)),
      stateTrajectory_(std::move(stateTrajectory)),
      inputTrajectory_(std::move(inputTrajectory)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
OperatingPoints* OperatingPoints::clone() const {
  return new OperatingPoints(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void OperatingPoints::getSystemOperatingTrajectories(const vector_t& initialState, scalar_t startTime, scalar_t finalTime,
                                                     scalar_array_t& timeTrajectory, vector_array_t& stateTrajectory,
                                                     vector_array_t& inputTrajectory, bool concatOutput /*= false*/) {
  if (!concatOutput) {
    timeTrajectory.clear();
    stateTrajectory.clear();
    inputTrajectory.clear();
  }

  getSystemOperatingTrajectoriesImpl(startTime, finalTime, timeTrajectory, stateTrajectory, inputTrajectory);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void OperatingPoints::getSystemOperatingTrajectoriesImpl(scalar_t startTime, scalar_t finalTime, scalar_array_t& timeTrajectory,
                                                         vector_array_t& stateTrajectory, vector_array_t& inputTrajectory) const {
  const auto initIndexAlpha = LinearInterpolation::timeSegment(startTime, timeTrajectory_);
  const auto finalindexAlpha = LinearInterpolation::timeSegment(finalTime, timeTrajectory_);

  vector_t x0;
  LinearInterpolation::interpolate(initIndexAlpha, x0, stateTrajectory_);
  vector_t u0;
  LinearInterpolation::interpolate(initIndexAlpha, u0, inputTrajectory_);
  timeTrajectory.push_back(startTime);
  stateTrajectory.push_back(std::move(x0));
  inputTrajectory.push_back(std::move(u0));

  // if the time interval is not empty
  if (!numerics::almost_eq(startTime, finalTime)) {
    const auto beginIndex = initIndexAlpha.first + 1;
    const auto lastIndex = finalindexAlpha.first + 1;
    std::copy(timeTrajectory_.begin() + beginIndex, timeTrajectory_.begin() + lastIndex, std::back_inserter(timeTrajectory));
    std::copy(stateTrajectory_.begin() + beginIndex, stateTrajectory_.begin() + lastIndex, std::back_inserter(stateTrajectory));
    std::copy(inputTrajectory_.begin() + beginIndex, inputTrajectory_.begin() + lastIndex, std::back_inserter(inputTrajectory));

    vector_t xf;
    LinearInterpolation::interpolate(finalindexAlpha, xf, stateTrajectory_);
    vector_t uf;
    LinearInterpolation::interpolate(finalindexAlpha, uf, inputTrajectory_);
    timeTrajectory.push_back(finalTime);
    stateTrajectory.push_back(std::move(xf));
    inputTrajectory.push_back(std::move(uf));
  }
}

}  // namespace ocs2
