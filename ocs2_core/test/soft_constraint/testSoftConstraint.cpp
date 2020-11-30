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

#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
#include <ocs2_core/soft_constraint/penalties/RelaxedBarrierPenaltyFunction.h>

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class TestStateConstraint : public ocs2::StateConstraint {
 public:
  TestStateConstraint(size_t numConstraints) : numConstraints_(numConstraints) {}
  ~TestStateConstraint() override = default;
  TestStateConstraint* clone() const override { return new TestStateConstraint(*this); }

  size_t getNumConstraints(ocs2::scalar_t time) const override { return numConstraints_; }
  ocs2::vector_t getValue(ocs2::scalar_t time, const ocs2::vector_t& state) const override { return ocs2::vector_t::Zero(numConstraints_); }
  ocs2::VectorFunctionLinearApproximation getLinearApproximation(ocs2::scalar_t time, const ocs2::vector_t& state) const override {
    return ocs2::VectorFunctionLinearApproximation::Zero(numConstraints_, state.size(), 0);
  }
  ocs2::VectorFunctionQuadraticApproximation getQuadraticApproximation(ocs2::scalar_t time, const ocs2::vector_t& state) const override {
    return ocs2::VectorFunctionQuadraticApproximation::Zero(numConstraints_, state.size(), 0);
  }

 private:
  size_t numConstraints_;
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class TestStateInputConstraint : public ocs2::StateInputConstraint {
 public:
  TestStateInputConstraint(size_t numConstraints) : numConstraints_(numConstraints) {}
  ~TestStateInputConstraint() override = default;
  TestStateInputConstraint* clone() const override { return new TestStateInputConstraint(*this); }

  size_t getNumConstraints(ocs2::scalar_t time) const override { return numConstraints_; }
  ocs2::vector_t getValue(ocs2::scalar_t time, const ocs2::vector_t& state, const ocs2::vector_t& input) const override {
    return ocs2::vector_t::Zero(numConstraints_);
  }
  ocs2::VectorFunctionLinearApproximation getLinearApproximation(ocs2::scalar_t time, const ocs2::vector_t& state,
                                                                 const ocs2::vector_t& input) const override {
    return ocs2::VectorFunctionLinearApproximation::Zero(numConstraints_, state.size(), input.size());
  }
  ocs2::VectorFunctionQuadraticApproximation getQuadraticApproximation(ocs2::scalar_t time, const ocs2::vector_t& state,
                                                                       const ocs2::vector_t& input) const override {
    return ocs2::VectorFunctionQuadraticApproximation::Zero(numConstraints_, state.size(), input.size());
  }

 private:
  size_t numConstraints_;
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Constraint, class SoftConstraint>
std::unique_ptr<SoftConstraint> softConstraintFactory(size_t numConstraints, ocs2::ConstraintOrder constraintOrder,
                                                      bool useSimilarPenalty) {
  // constraint
  std::unique_ptr<Constraint> constraintPtr(new Constraint(numConstraints));

  if (useSimilarPenalty) {
    // penalty function
    std::unique_ptr<ocs2::RelaxedBarrierPenaltyFunction> penaltyFunctionPtr(new ocs2::RelaxedBarrierPenaltyFunction({10.0, 1.0}));
    return std::unique_ptr<SoftConstraint>(
        new SoftConstraint(std::move(constraintPtr), numConstraints, std::move(penaltyFunctionPtr), constraintOrder));

  } else {
    std::vector<std::unique_ptr<ocs2::PenaltyFunctionBase>> penaltyFunctionPtrArry;
    for (size_t i = 0; i < numConstraints; i++) {
      penaltyFunctionPtrArry.emplace_back(new ocs2::RelaxedBarrierPenaltyFunction({10.0, 1.0}));
    }  // end of i loop
    return std::unique_ptr<SoftConstraint>(
        new SoftConstraint(std::move(constraintPtr), std::move(penaltyFunctionPtrArry), constraintOrder));
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TEST(testSoftConstraint, clone) {
  constexpr size_t numConstraints = 10;

  auto stateQuadraticConstraintPtr =
      softConstraintFactory<TestStateConstraint, ocs2::StateSoftConstraint>(numConstraints, ocs2::ConstraintOrder::Quadratic, true);
  EXPECT_NO_THROW(std::unique_ptr<ocs2::StateSoftConstraint>(stateQuadraticConstraintPtr->clone()));

  auto stateLinearConstraintPtr =
      softConstraintFactory<TestStateInputConstraint, ocs2::StateInputSoftConstraint>(numConstraints, ocs2::ConstraintOrder::Linear, false);
  EXPECT_NO_THROW(std::unique_ptr<ocs2::StateInputSoftConstraint>(stateLinearConstraintPtr->clone()));
}

TEST(testSoftConstraint, softStateConstraint) {
  constexpr size_t numConstraints = 10;
  const ocs2::vector_t state = ocs2::vector_t::Zero(2);
  const ocs2::vector_t input = ocs2::vector_t::Zero(1);
  const ocs2::CostDesiredTrajectories costDesiredTrajectories;

  auto stateLinearConstraintPtr =
      softConstraintFactory<TestStateConstraint, ocs2::StateSoftConstraint>(numConstraints, ocs2::ConstraintOrder::Linear, false);
  EXPECT_NO_THROW(stateLinearConstraintPtr->getValue(0.0, state, costDesiredTrajectories));
  EXPECT_NO_THROW(stateLinearConstraintPtr->getQuadraticApproximation(0.0, state, costDesiredTrajectories));

  auto stateQuadraticConstraintPtr =
      softConstraintFactory<TestStateConstraint, ocs2::StateSoftConstraint>(numConstraints, ocs2::ConstraintOrder::Quadratic, true);
  EXPECT_NO_THROW(stateQuadraticConstraintPtr->getValue(0.0, state, costDesiredTrajectories));
  EXPECT_NO_THROW(stateQuadraticConstraintPtr->getQuadraticApproximation(0.0, state, costDesiredTrajectories));
}

TEST(testSoftConstraint, softStateInputConstraint) {
  constexpr size_t numConstraints = 10;
  const ocs2::vector_t state = ocs2::vector_t::Zero(2);
  const ocs2::vector_t input = ocs2::vector_t::Zero(1);
  const ocs2::CostDesiredTrajectories costDesiredTrajectories;

  auto stateInputLinearConstraintPtr =
      softConstraintFactory<TestStateInputConstraint, ocs2::StateInputSoftConstraint>(numConstraints, ocs2::ConstraintOrder::Linear, true);
  EXPECT_NO_THROW(stateInputLinearConstraintPtr->getValue(0.0, state, input, costDesiredTrajectories));
  EXPECT_NO_THROW(stateInputLinearConstraintPtr->getQuadraticApproximation(0.0, state, input, costDesiredTrajectories));

  auto stateInputQuadraticConstraintPtr = softConstraintFactory<TestStateInputConstraint, ocs2::StateInputSoftConstraint>(
      numConstraints, ocs2::ConstraintOrder::Quadratic, false);
  EXPECT_NO_THROW(stateInputQuadraticConstraintPtr->getValue(0.0, state, input, costDesiredTrajectories));
  EXPECT_NO_THROW(stateInputQuadraticConstraintPtr->getQuadraticApproximation(0.0, state, input, costDesiredTrajectories));
}
