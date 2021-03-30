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

#include <ocs2_mobile_manipulator_example/constraint/JointVelocityLimits.h>

namespace ocs2 {
namespace mobile_manipulator {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
JointVelocityLimits::JointVelocityLimits(vector_t lowerBound, vector_t upperBound)
    : lowerBound_(std::move(lowerBound)), upperBound_(std::move(upperBound)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t JointVelocityLimits::getValue(scalar_t time, const vector_t& state, const vector_t& input) const {
  vector_t limits(2 * input.rows());
  limits.head(input.rows()) = input - lowerBound_;  // lowerBound_ < input
  limits.tail(input.rows()) = upperBound_ - input;  // input < upperBound_
  return limits;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation JointVelocityLimits::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                              const vector_t& input) const {
  VectorFunctionLinearApproximation limits(2 * input.rows(), state.rows(), input.rows());
  limits.f = getValue(time, state, input);
  limits.dfdx.setZero();
  limits.dfdu.topRows(input.rows()).setIdentity();
  limits.dfdu.bottomRows(input.rows()) = -matrix_t::Identity(input.rows(), input.rows());
  return limits;
}

}  // namespace mobile_manipulator
}  // namespace ocs2
