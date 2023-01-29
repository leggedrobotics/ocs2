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

#include <ocs2_core/constraint/StateInputConstraintCollection.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateInputConstraintCollection::StateInputConstraintCollection(const StateInputConstraintCollection& other)
    : Collection<StateInputConstraint>(other) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateInputConstraintCollection* StateInputConstraintCollection::clone() const {
  return new StateInputConstraintCollection(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t StateInputConstraintCollection::getNumConstraints(scalar_t time) const {
  size_t numConstraints = 0;
  for (const auto& constraintTerm : this->terms_) {
    if (constraintTerm->isActive(time)) {
      numConstraints += constraintTerm->getNumConstraints(time);
    }
  }
  return numConstraints;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_array_t StateInputConstraintCollection::getTermsSize(scalar_t time) const {
  size_array_t termsSize(this->terms_.size(), 0);
  for (size_t i = 0; i < this->terms_.size(); ++i) {
    if (this->terms_[i]->isActive(time)) {
      termsSize[i] = this->terms_[i]->getNumConstraints(time);
    }
  }
  return termsSize;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_array_t StateInputConstraintCollection::getValue(scalar_t time, const vector_t& state, const vector_t& input,
                                                        const PreComputation& preComp) const {
  vector_array_t constraintValues(this->terms_.size());
  for (size_t i = 0; i < this->terms_.size(); ++i) {
    if (this->terms_[i]->isActive(time)) {
      constraintValues[i] = this->terms_[i]->getValue(time, state, input, preComp);
    }
  }  // end of i loop
  return constraintValues;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation StateInputConstraintCollection::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                                         const vector_t& input,
                                                                                         const PreComputation& preComp) const {
  VectorFunctionLinearApproximation linearApproximation(getNumConstraints(time), state.rows(), input.rows());

  // append linearApproximation of each constraintTerm
  size_t i = 0;
  for (const auto& constraintTerm : this->terms_) {
    if (constraintTerm->isActive(time)) {
      const auto constraintTermApproximation = constraintTerm->getLinearApproximation(time, state, input, preComp);
      const size_t nc = constraintTermApproximation.f.rows();
      linearApproximation.f.segment(i, nc) = constraintTermApproximation.f;
      linearApproximation.dfdx.middleRows(i, nc) = constraintTermApproximation.dfdx;
      linearApproximation.dfdu.middleRows(i, nc) = constraintTermApproximation.dfdu;
      i += nc;
    }
  }

  return linearApproximation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionQuadraticApproximation StateInputConstraintCollection::getQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                                               const vector_t& input,
                                                                                               const PreComputation& preComp) const {
  const auto numConstraints = getNumConstraints(time);

  VectorFunctionQuadraticApproximation quadraticApproximation;
  quadraticApproximation.f.resize(numConstraints);
  quadraticApproximation.dfdx.resize(numConstraints, state.rows());
  quadraticApproximation.dfdu.resize(numConstraints, input.rows());
  quadraticApproximation.dfdxx.reserve(numConstraints);  // Use reserve instead of resize to avoid unnecessary allocations.
  quadraticApproximation.dfdux.reserve(numConstraints);
  quadraticApproximation.dfduu.reserve(numConstraints);

  // append quadraticApproximation of each constraintTerm
  size_t i = 0;
  for (const auto& constraintTerm : this->terms_) {
    if (constraintTerm->isActive(time)) {
      auto constraintTermApproximation = constraintTerm->getQuadraticApproximation(time, state, input, preComp);
      const size_t nc = constraintTermApproximation.f.rows();
      quadraticApproximation.f.segment(i, nc) = constraintTermApproximation.f;
      quadraticApproximation.dfdx.middleRows(i, nc) = constraintTermApproximation.dfdx;
      quadraticApproximation.dfdu.middleRows(i, nc) = constraintTermApproximation.dfdu;
      appendVectorToVectorByMoving(quadraticApproximation.dfdxx, std::move(constraintTermApproximation.dfdxx));
      appendVectorToVectorByMoving(quadraticApproximation.dfdux, std::move(constraintTermApproximation.dfdux));
      appendVectorToVectorByMoving(quadraticApproximation.dfduu, std::move(constraintTermApproximation.dfduu));
      i += nc;
    }
  }

  return quadraticApproximation;
}

}  // namespace ocs2
