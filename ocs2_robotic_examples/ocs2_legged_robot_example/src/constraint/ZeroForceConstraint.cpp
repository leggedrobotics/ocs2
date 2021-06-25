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

#include <ocs2_legged_robot_example/constraint/ZeroForceConstraint.h>

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ZeroForceConstraint::ZeroForceConstraint(int contactPointNumber) : BASE(ConstraintOrder::Linear), contactPointNumber_(contactPointNumber) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ZeroForceConstraint* ZeroForceConstraint::clone() const {
  return new ZeroForceConstraint(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t ZeroForceConstraint::getNumConstraints(scalar_t time) const {
  return 3;
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t ZeroForceConstraint::getValue(scalar_t time, const ocs2::vector_t& state, const ocs2::vector_t& input) const {
  const scalar_t Fx = input(3 * contactPointNumber_ + 0);
  const scalar_t Fy = input(3 * contactPointNumber_ + 1);
  const scalar_t Fz = input(3 * contactPointNumber_ + 2);

  vector_t constraintValues(getNumConstraints(time));
  constraintValues[0] = Fx;
  constraintValues[1] = Fy;
  constraintValues[2] = Fz;
  return constraintValues;
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation ZeroForceConstraint::getLinearApproximation(scalar_t time, const ocs2::vector_t& state,
                                                                              const ocs2::vector_t& input) const {
  VectorFunctionLinearApproximation linearApproximation =
      VectorFunctionLinearApproximation::Zero(getNumConstraints(time), state.size(), input.size());
  for (int i = 0; i < 3; i++) {
    linearApproximation.f[i] = input(3 * contactPointNumber_ + i);
    linearApproximation.dfdu(i, 3 * contactPointNumber_ + i) = 1.0;
  }

  return linearApproximation;
}

}  // namespace legged_robot
}  // namespace ocs2
