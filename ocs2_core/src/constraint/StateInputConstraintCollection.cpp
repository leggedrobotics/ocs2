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
StateInputConstraintCollection::StateInputConstraintCollection(const StateInputConstraintCollection& other) {
  // Loop through all constraints by name and clone into the new object
  constraintTermMap_.clear();
  for (const auto& constraintPair : other.constraintTermMap_) {
    add(constraintPair.first, std::unique_ptr<StateInputConstraint>(constraintPair.second->clone()));
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateInputConstraintCollection* StateInputConstraintCollection::clone() const {
  return new StateInputConstraintCollection(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void StateInputConstraintCollection::add(std::string name, std::unique_ptr<StateInputConstraint> constraintTerm) {
  auto info = constraintTermMap_.emplace(std::move(name), std::move(constraintTerm));
  if (!info.second) {
    throw std::runtime_error(std::string("[StateInputConstraintCollection::add] Constraint term with name \"") + info.first->first +
                             "\" already exists");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t StateInputConstraintCollection::getNumConstraints(scalar_t time) const {
  size_t numConstraints = 0;

  // accumulate number of constraints for each constraintTerm
  for (const auto& constraintPair : constraintTermMap_) {
    if (constraintPair.second->isActive(time)) {
      numConstraints += constraintPair.second->getNumConstraints(time);
    }
  }

  return numConstraints;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t StateInputConstraintCollection::getValue(scalar_t time, const vector_t& state, const vector_t& input,
                                                  const PreComputation& preComp) const {
  vector_t constraintValues;
  constraintValues.resize(getNumConstraints(time));

  // append vectors of constraint values from each constraintTerm
  size_t i = 0;
  for (const auto& constraintPair : constraintTermMap_) {
    if (constraintPair.second->isActive(time)) {
      const auto constraintTermValues = constraintPair.second->getValue(time, state, input, preComp);
      constraintValues.segment(i, constraintTermValues.rows()) = constraintTermValues;
      i += constraintTermValues.rows();
    }
  }

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
  for (const auto& constraintPair : constraintTermMap_) {
    if (constraintPair.second->isActive(time)) {
      const auto constraintTermApproximation = constraintPair.second->getLinearApproximation(time, state, input, preComp);
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
  for (const auto& constraintPair : constraintTermMap_) {
    if (constraintPair.second->isActive(time)) {
      auto constraintTermApproximation = constraintPair.second->getQuadraticApproximation(time, state, input, preComp);
      const size_t nc = constraintTermApproximation.f.rows();
      quadraticApproximation.f.segment(i, nc) = constraintTermApproximation.f;
      quadraticApproximation.dfdx.middleRows(i, nc) = constraintTermApproximation.dfdx;
      quadraticApproximation.dfdu.middleRows(i, nc) = constraintTermApproximation.dfdu;
      appendVectorToVectorByMoving(quadraticApproximation.dfdxx, constraintTermApproximation.dfdxx);
      appendVectorToVectorByMoving(quadraticApproximation.dfdux, constraintTermApproximation.dfdux);
      appendVectorToVectorByMoving(quadraticApproximation.dfduu, constraintTermApproximation.dfduu);
      i += nc;
    }
  }

  return quadraticApproximation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename T, typename Allocator>
void StateInputConstraintCollection::appendVectorToVectorByMoving(std::vector<T, Allocator>& v1, std::vector<T, Allocator>& v2) const {
  v1.insert(v1.end(), std::make_move_iterator(v2.begin()), std::make_move_iterator(v2.end()));
}

}  // namespace ocs2
