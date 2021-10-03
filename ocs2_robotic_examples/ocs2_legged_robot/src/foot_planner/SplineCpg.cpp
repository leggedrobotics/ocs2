/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

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

#include "ocs2_legged_robot/foot_planner/SplineCpg.h"

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SplineCpg::SplineCpg(CubicSpline::Node liftOff, scalar_t midHeight, CubicSpline::Node touchDown)
    : midTime_((liftOff.time + touchDown.time) / 2),
      leftSpline_(liftOff, CubicSpline::Node{midTime_, midHeight, 0.0}),
      rightSpline_(CubicSpline::Node{midTime_, midHeight, 0.0}, touchDown) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SplineCpg::position(scalar_t time) const {
  return (time < midTime_) ? leftSpline_.position(time) : rightSpline_.position(time);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SplineCpg::velocity(scalar_t time) const {
  return (time < midTime_) ? leftSpline_.velocity(time) : rightSpline_.velocity(time);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SplineCpg::acceleration(scalar_t time) const {
  return (time < midTime_) ? leftSpline_.acceleration(time) : rightSpline_.acceleration(time);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SplineCpg::startTimeDerivative(scalar_t time) const {
  if (time <= midTime_) {
    return leftSpline_.startTimeDerivative(time) + 0.5 * leftSpline_.startTimeDerivative(time);
  } else {
    return 0.5 * rightSpline_.startTimeDerivative(time);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SplineCpg::finalTimeDerivative(scalar_t time) const {
  if (time <= midTime_) {
    return 0.5 * leftSpline_.finalTimeDerivative(time);
  } else {
    return rightSpline_.finalTimeDerivative(time) + 0.5 * rightSpline_.finalTimeDerivative(time);
  }
}

}  // namespace legged_robot
}  // namespace ocs2
