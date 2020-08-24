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

#include <ocs2_core/constraint/ConstraintCollection.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename CONSTRAINT>
ConstraintCollection<CONSTRAINT>::ConstraintCollection(const ConstraintCollection<CONSTRAINT>& rhs) {
  // Loop through all constraints by name and clone into the new object
  constraintTermMap_.clear();
  for (auto& constraintPair : rhs.constraintTermMap_) {
    add(constraintPair.first, std::unique_ptr<CONSTRAINT>(constraintPair.second->clone()));
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename CONSTRAINT>
ConstraintCollection<CONSTRAINT>::ConstraintCollection(ConstraintCollection<CONSTRAINT>&& rhs) noexcept
    : constraintTermMap_(std::move(rhs.constraintTermMap_)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename CONSTRAINT>
void ConstraintCollection<CONSTRAINT>::add(std::string name, std::unique_ptr<CONSTRAINT> constraintTerm) {
  auto info = constraintTermMap_.emplace(std::move(name), std::move(constraintTerm));
  if (!info.second) {
    throw std::runtime_error("[ConstraintCollection::add] Constraint name already exists");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename CONSTRAINT>
size_t ConstraintCollection<CONSTRAINT>::getNumConstraints(scalar_t time) const {
  size_t numConstraints = 0;

  // accumulate number of constraints for each constraintTerm
  for (auto& constraintPair : constraintTermMap_) {
    if (constraintPair.second->isActive()) {
      numConstraints += constraintPair.second->getNumConstraints(time);
    }
  }

  return numConstraints;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <>
template <>
vector_t ConstraintCollection<StateInputConstraint>::getValue(scalar_t time, const vector_t& state, const vector_t& input) const {
  vector_t constraintValues;
  constraintValues.resize(getNumConstraints(time));

  // append vectors of constraint values from each constraintTerm
  size_t i = 0;
  for (auto& constraintPair : constraintTermMap_) {
    if (constraintPair.second->isActive()) {
      const auto constraintTermValues = constraintPair.second->getValue(time, state, input);
      constraintValues.segment(i, constraintTermValues.rows()) = constraintTermValues;
      i += constraintTermValues.rows();
    }
  }

  return constraintValues;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <>
template <>
auto ConstraintCollection<StateInputConstraint>::getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input) const
    -> LinearApproximation_t {
  LinearApproximation_t linearApproximation;
  linearApproximation.resize(getNumConstraints(time), state.rows(), input.rows());

  // append linearApproximation each constraintTerm
  size_t i = 0;
  for (auto& constraintPair : constraintTermMap_) {
    if (constraintPair.second->isActive()) {
      const auto constraintTermApproximation = constraintPair.second->getLinearApproximation(time, state, input);
      const size_t nc = constraintTermApproximation.f.rows();
      linearApproximation.f.segment(i, nc) = constraintTermApproximation.f;
      linearApproximation.dfdx.block(i, 0, nc, state.rows()) = constraintTermApproximation.dfdx;
      linearApproximation.dfdu.block(i, 0, nc, input.rows()) = constraintTermApproximation.dfdu;
      i += nc;
    }
  }

  return linearApproximation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <>
template <>
auto ConstraintCollection<StateInputConstraint>::getQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                           const vector_t& input) const -> QuadraticApproximation_t {
  const auto numConstraints = getNumConstraints(time);

  QuadraticApproximation_t quadraticApproximation;
  quadraticApproximation.f.resize(numConstraints);
  quadraticApproximation.dfdx.resize(numConstraints, state.rows());
  quadraticApproximation.dfdu.resize(numConstraints, input.rows());
  quadraticApproximation.dfdxx.reserve(numConstraints);
  quadraticApproximation.dfdux.reserve(numConstraints);
  quadraticApproximation.dfduu.reserve(numConstraints);

  // append quadraticApproximation each constraintTerm
  size_t i = 0;
  for (auto& constraintPair : constraintTermMap_) {
    if (constraintPair.second->isActive()) {
      auto constraintTermApproximation = constraintPair.second->getQuadraticApproximation(time, state, input);
      const size_t nc = constraintTermApproximation.f.rows();
      quadraticApproximation.f.segment(i, nc) = constraintTermApproximation.f;
      quadraticApproximation.dfdx.block(i, 0, nc, state.rows()) = constraintTermApproximation.dfdx;
      quadraticApproximation.dfdu.block(i, 0, nc, input.rows()) = constraintTermApproximation.dfdu;
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
template <>
template <>
vector_t ConstraintCollection<StateConstraint>::getValue(scalar_t time, const vector_t& state) const {
  vector_t constraintValues;
  constraintValues.resize(getNumConstraints(time));

  // append vectors of constraint values from each constraintTerm
  size_t i = 0;
  for (auto& constraintPair : constraintTermMap_) {
    if (constraintPair.second->isActive()) {
      const auto constraintTermValues = constraintPair.second->getValue(time, state);
      constraintValues.segment(i, constraintTermValues.rows()) = constraintTermValues;
      i += constraintTermValues.rows();
    }
  }

  return constraintValues;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <>
template <>
auto ConstraintCollection<StateConstraint>::getLinearApproximation(scalar_t time, const vector_t& state) const -> LinearApproximation_t {
  LinearApproximation_t linearApproximation;
  linearApproximation.resize(getNumConstraints(time), state.rows(), 0);

  // append linearApproximation each constraintTerm
  size_t i = 0;
  for (auto& constraintPair : constraintTermMap_) {
    if (constraintPair.second->isActive()) {
      const auto constraintTermApproximation = constraintPair.second->getLinearApproximation(time, state);
      const size_t nc = constraintTermApproximation.f.rows();
      linearApproximation.f.segment(i, nc) = constraintTermApproximation.f;
      linearApproximation.dfdx.block(i, 0, nc, state.rows()) = constraintTermApproximation.dfdx;
      i += nc;
    }
  }

  return linearApproximation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <>
template <>
auto ConstraintCollection<StateConstraint>::getQuadraticApproximation(scalar_t time, const vector_t& state) const
    -> QuadraticApproximation_t {
  const auto numConstraints = getNumConstraints(time);

  QuadraticApproximation_t quadraticApproximation;
  quadraticApproximation.f.resize(numConstraints);
  quadraticApproximation.dfdx.resize(numConstraints, state.rows());
  quadraticApproximation.dfdxx.reserve(numConstraints);

  // append quadraticApproximation each constraintTerm
  size_t i = 0;
  for (auto& constraintPair : constraintTermMap_) {
    if (constraintPair.second->isActive()) {
      auto constraintTermApproximation = constraintPair.second->getQuadraticApproximation(time, state);
      const size_t nc = constraintTermApproximation.f.rows();
      quadraticApproximation.f.segment(i, nc) = constraintTermApproximation.f;
      quadraticApproximation.dfdx.block(i, 0, nc, state.rows()) = constraintTermApproximation.dfdx;
      appendVectorToVectorByMoving(quadraticApproximation.dfdxx, constraintTermApproximation.dfdxx);
      i += nc;
    }
  }

  return quadraticApproximation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename CONSTRAINT>
template <typename T, typename Allocator>
void ConstraintCollection<CONSTRAINT>::appendVectorToVectorByMoving(std::vector<T, Allocator>& v1, std::vector<T, Allocator>& v2) const {
  v1.insert(v1.end(), std::make_move_iterator(v2.begin()), std::make_move_iterator(v2.end()));
}

// explicit template instantiation
template class ConstraintCollection<StateInputConstraint>;
template class ConstraintCollection<StateConstraint>;

}  // namespace ocs2
