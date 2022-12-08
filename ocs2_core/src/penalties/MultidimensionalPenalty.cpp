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

#include <cassert>

#include <ocs2_core/penalties/MultidimensionalPenalty.h>

namespace ocs2 {

namespace {

/** A wrapper class of PenaltyBase to AugmentedPenaltyBase */
class PenaltyBaseWrapper final : public augmented::AugmentedPenaltyBase {
 public:
  /** Default constructor */
  PenaltyBaseWrapper(std::unique_ptr<PenaltyBase> penaltyPtr) : penaltyPtr_(std::move(penaltyPtr)) {}

  ~PenaltyBaseWrapper() override = default;
  PenaltyBaseWrapper* clone() const override { return new PenaltyBaseWrapper(*this); }
  std::string name() const override { return penaltyPtr_->name(); }

  scalar_t getValue(scalar_t t, scalar_t l, scalar_t h) const override { return penaltyPtr_->getValue(t, h); }
  scalar_t getDerivative(scalar_t t, scalar_t l, scalar_t h) const override { return penaltyPtr_->getDerivative(t, h); }
  scalar_t getSecondDerivative(scalar_t t, scalar_t l, scalar_t h) const override { return penaltyPtr_->getSecondDerivative(t, h); }

  scalar_t updateMultiplier(scalar_t t, scalar_t l, scalar_t h) const override {
    throw std::runtime_error("[" + name() + "] This penalty is only applicable to soft constraints!");
  }
  scalar_t initializeMultiplier() const override {
    throw std::runtime_error("[" + name() + "] This penalty is only applicable to soft constraints!");
  }

 private:
  PenaltyBaseWrapper(const PenaltyBaseWrapper& other) : penaltyPtr_(other.penaltyPtr_->clone()) {}

  std::unique_ptr<PenaltyBase> penaltyPtr_;
};

std::unique_ptr<PenaltyBaseWrapper> createWrapper(std::unique_ptr<PenaltyBase> penaltyPtr) {
  return std::make_unique<PenaltyBaseWrapper>(std::move(penaltyPtr));
}

scalar_t getMultiplier(const vector_t* l, size_t ind) {
  return (l == nullptr) ? 0.0 : (*l)(ind);
}

}  // namespace

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <>
MultidimensionalPenalty::MultidimensionalPenalty<augmented::AugmentedPenaltyBase>(
    std::vector<std::unique_ptr<augmented::AugmentedPenaltyBase>> penaltyPtrArray)
    : penaltyPtrArray_(std::move(penaltyPtrArray)) {
  if (penaltyPtrArray_.empty()) {
    throw std::runtime_error("[MultidimensionalPenalty::MultidimensionalPenalty] The penalty array cannot be empty!");
  }
}

template <>
MultidimensionalPenalty::MultidimensionalPenalty<PenaltyBase>(std::vector<std::unique_ptr<PenaltyBase>> penaltyPtrArray) {
  for (auto& penaltyPtr : penaltyPtrArray) {
    penaltyPtrArray_.push_back(createWrapper(std::move(penaltyPtr)));
  }

  if (penaltyPtrArray_.empty()) {
    throw std::runtime_error("[MultidimensionalPenalty::MultidimensionalPenalty] The penalty array cannot be empty!");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <>
MultidimensionalPenalty::MultidimensionalPenalty<augmented::AugmentedPenaltyBase>(
    std::unique_ptr<augmented::AugmentedPenaltyBase> penaltyPtr) {
  penaltyPtrArray_.push_back(std::move(penaltyPtr));
}

template <>
MultidimensionalPenalty::MultidimensionalPenalty<PenaltyBase>(std::unique_ptr<PenaltyBase> penaltyPtr) {
  penaltyPtrArray_.push_back(createWrapper(std::move(penaltyPtr)));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MultidimensionalPenalty::MultidimensionalPenalty(const MultidimensionalPenalty& other) {
  for (const auto& penalty : other.penaltyPtrArray_) {
    penaltyPtrArray_.emplace_back(penalty->clone());
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t MultidimensionalPenalty::getValue(scalar_t t, const vector_t& h, const vector_t* l) const {
  const auto numConstraints = h.rows();
  assert(penaltyPtrArray_.size() == 1 || penaltyPtrArray_.size() == numConstraints);

  scalar_t penalty = 0;
  for (size_t i = 0; i < numConstraints; i++) {
    const auto& penaltyTerm = (penaltyPtrArray_.size() == 1) ? penaltyPtrArray_[0] : penaltyPtrArray_[i];
    penalty += penaltyTerm->getValue(t, getMultiplier(l, i), h(i));
  }

  return penalty;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation MultidimensionalPenalty::getQuadraticApproximation(scalar_t t,
                                                                                        const VectorFunctionLinearApproximation& h,
                                                                                        const vector_t* l) const {
  const auto stateDim = h.dfdx.cols();
  const auto inputDim = h.dfdu.cols();

  scalar_t penaltyValue = 0.0;
  vector_t penaltyDerivative, penaltySecondDerivative;
  std::tie(penaltyValue, penaltyDerivative, penaltySecondDerivative) = getPenaltyValue1stDev2ndDev(t, h.f, l);
  const matrix_t penaltySecondDev_dhdx = penaltySecondDerivative.asDiagonal() * h.dfdx;

  // to make sure that dfdux in the state-only case has a right size
  ScalarFunctionQuadraticApproximation penaltyApproximation(stateDim, inputDim);

  penaltyApproximation.f = penaltyValue;
  penaltyApproximation.dfdx.noalias() = h.dfdx.transpose() * penaltyDerivative;
  penaltyApproximation.dfdxx.noalias() = h.dfdx.transpose() * penaltySecondDev_dhdx;
  if (inputDim > 0) {
    penaltyApproximation.dfdu.noalias() = h.dfdu.transpose() * penaltyDerivative;
    penaltyApproximation.dfdux.noalias() = h.dfdu.transpose() * penaltySecondDev_dhdx;
    penaltyApproximation.dfduu.noalias() = h.dfdu.transpose() * penaltySecondDerivative.asDiagonal() * h.dfdu;
  }

  return penaltyApproximation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation MultidimensionalPenalty::getQuadraticApproximation(scalar_t t,
                                                                                        const VectorFunctionQuadraticApproximation& h,
                                                                                        const vector_t* l) const {
  const auto stateDim = h.dfdx.cols();
  const auto inputDim = h.dfdu.cols();
  const auto numConstraints = h.f.rows();

  scalar_t penaltyValue = 0.0;
  vector_t penaltyDerivative, penaltySecondDerivative;
  std::tie(penaltyValue, penaltyDerivative, penaltySecondDerivative) = getPenaltyValue1stDev2ndDev(t, h.f, l);
  const matrix_t penaltySecondDev_dhdx = penaltySecondDerivative.asDiagonal() * h.dfdx;

  // to make sure that dfdux in the state-only case has a right size
  ScalarFunctionQuadraticApproximation penaltyApproximation(stateDim, inputDim);

  penaltyApproximation.f = penaltyValue;
  penaltyApproximation.dfdx.noalias() = h.dfdx.transpose() * penaltyDerivative;
  penaltyApproximation.dfdxx.noalias() = h.dfdx.transpose() * penaltySecondDev_dhdx;
  for (size_t i = 0; i < numConstraints; i++) {
    penaltyApproximation.dfdxx.noalias() += penaltyDerivative(i) * h.dfdxx[i];
  }

  if (inputDim > 0) {
    penaltyApproximation.dfdu.noalias() = h.dfdu.transpose() * penaltyDerivative;
    penaltyApproximation.dfdux.noalias() = h.dfdu.transpose() * penaltySecondDev_dhdx;
    penaltyApproximation.dfduu.noalias() = h.dfdu.transpose() * penaltySecondDerivative.asDiagonal() * h.dfdu;
    for (size_t i = 0; i < numConstraints; i++) {
      penaltyApproximation.dfduu.noalias() += penaltyDerivative(i) * h.dfduu[i];
      penaltyApproximation.dfdux.noalias() += penaltyDerivative(i) * h.dfdux[i];
    }
  }

  return penaltyApproximation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::tuple<scalar_t, vector_t, vector_t> MultidimensionalPenalty::getPenaltyValue1stDev2ndDev(scalar_t t, const vector_t& h,
                                                                                              const vector_t* l) const {
  const auto numConstraints = h.rows();
  assert(penaltyPtrArray_.size() == 1 || penaltyPtrArray_.size() == numConstraints);

  scalar_t penaltyValue = 0.0;
  vector_t penaltyDerivative(numConstraints);
  vector_t penaltySecondDerivative(numConstraints);
  for (size_t i = 0; i < numConstraints; i++) {
    const auto& penaltyTerm = (penaltyPtrArray_.size() == 1) ? penaltyPtrArray_[0] : penaltyPtrArray_[i];
    penaltyValue += penaltyTerm->getValue(t, getMultiplier(l, i), h(i));
    penaltyDerivative(i) = penaltyTerm->getDerivative(t, getMultiplier(l, i), h(i));
    penaltySecondDerivative(i) = penaltyTerm->getSecondDerivative(t, getMultiplier(l, i), h(i));
  }  // end of i loop

  return {penaltyValue, penaltyDerivative, penaltySecondDerivative};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t MultidimensionalPenalty::updateMultipliers(scalar_t t, const vector_t& h, const vector_t& l) const {
  const size_t numConstraints = h.size();
  assert(l.size() == numConstraints);
  assert(penaltyPtrArray_.size() == 1 || penaltyPtrArray_.size() == numConstraints);

  vector_t updted_l(numConstraints);
  for (size_t i = 0; i < numConstraints; i++) {
    const auto& penaltyTerm = (penaltyPtrArray_.size() == 1) ? penaltyPtrArray_[0] : penaltyPtrArray_[i];
    updted_l(i) = penaltyTerm->updateMultiplier(t, l(i), h(i));
  }

  return updted_l;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t MultidimensionalPenalty::initializeMultipliers(size_t numConstraints) const {
  assert(penaltyPtrArray_.size() == 1 || penaltyPtrArray_.size() == numConstraints);

  vector_t l(numConstraints);
  for (size_t i = 0; i < numConstraints; i++) {
    const auto& penaltyTerm = (penaltyPtrArray_.size() == 1) ? penaltyPtrArray_[0] : penaltyPtrArray_[i];
    l(i) = penaltyTerm->initializeMultiplier();
  }

  return l;
}

}  // namespace ocs2
