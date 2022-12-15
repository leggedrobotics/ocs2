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
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <memory>

#include "ocs2_ipm/IpmSolver.h"

#include <ocs2_core/initialization/DefaultInitializer.h>
#include <ocs2_oc/test/EXP1.h>

using namespace ocs2;

class EXP1_StateIneqConstraints final : public StateConstraint {
 public:
  EXP1_StateIneqConstraints(const vector_t& xmin, const vector_t& xmax)
      : StateConstraint(ConstraintOrder::Linear), xmin_(xmin), xmax_(xmax) {}
  ~EXP1_StateIneqConstraints() override = default;

  EXP1_StateIneqConstraints* clone() const override { return new EXP1_StateIneqConstraints(*this); }

  size_t getNumConstraints(scalar_t time) const override { return 4; }

  vector_t getValue(scalar_t t, const vector_t& x, const PreComputation&) const override {
    vector_t e(4);
    e.head(2) = x - xmin_;
    e.tail(2) = xmax_ - x;
    return e;
  }

  VectorFunctionLinearApproximation getLinearApproximation(scalar_t t, const vector_t& x, const PreComputation& preComp) const override {
    VectorFunctionLinearApproximation e;
    e.f = getValue(t, x, preComp);
    e.dfdx = matrix_t::Zero(4, x.size());
    e.dfdx.topLeftCorner(2, 2) = matrix_t::Identity(2, 2);
    e.dfdx.bottomRightCorner(2, 2) = -matrix_t::Identity(2, 2);
    return e;
  }

 private:
  vector_t xmin_, xmax_;
};

class EXP1_StateInputIneqConstraints final : public StateInputConstraint {
 public:
  EXP1_StateInputIneqConstraints(scalar_t umin, scalar_t umax) : StateInputConstraint(ConstraintOrder::Linear), umin_(umin), umax_(umax) {}
  ~EXP1_StateInputIneqConstraints() override = default;

  EXP1_StateInputIneqConstraints* clone() const override { return new EXP1_StateInputIneqConstraints(*this); }

  size_t getNumConstraints(scalar_t time) const override { return 2; }

  vector_t getValue(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation&) const override {
    vector_t e(2);
    e << (u.coeff(0) - umin_), (umax_ - u.coeff(0));
    return e;
  }

  VectorFunctionLinearApproximation getLinearApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                           const PreComputation& preComp) const override {
    VectorFunctionLinearApproximation e;
    e.f = getValue(t, x, u, preComp);
    e.dfdx = matrix_t::Zero(2, x.size());
    e.dfdu = (matrix_t(2, 1) << 1, -1).finished();
    return e;
  }

 private:
  scalar_t umin_, umax_;
};

class EXP1_MixedStateInputIneqConstraints final : public StateInputConstraint {
 public:
  EXP1_MixedStateInputIneqConstraints(scalar_t xumin, scalar_t xumax)
      : StateInputConstraint(ConstraintOrder::Linear), xumin_(xumin), xumax_(xumax) {}
  ~EXP1_MixedStateInputIneqConstraints() override = default;

  EXP1_MixedStateInputIneqConstraints* clone() const override { return new EXP1_MixedStateInputIneqConstraints(*this); }

  size_t getNumConstraints(scalar_t time) const override { return 2; }

  vector_t getValue(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation&) const override {
    vector_t e(2);
    e << (x.coeff(0) * u.coeff(0) - xumin_), (xumax_ - x.coeff(1) * u.coeff(0));
    return e;
  }

  VectorFunctionLinearApproximation getLinearApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                           const PreComputation& preComp) const override {
    VectorFunctionLinearApproximation e;
    e.f = getValue(t, x, u, preComp);
    e.dfdx = (matrix_t(2, 2) << u.coeff(0), 0, 0, -u.coeff(0)).finished();
    e.dfdu = (matrix_t(2, 1) << x.coeff(0), -x.coeff(1)).finished();
    return e;
  }

 private:
  scalar_t xumin_, xumax_;
};

TEST(Exp1Test, Unconstrained) {
  static constexpr size_t STATE_DIM = 2;
  static constexpr size_t INPUT_DIM = 1;

  // Solver settings
  const auto settings = []() {
    ipm::Settings s;
    s.dt = 0.01;
    s.ipmIteration = 20;
    s.projectStateInputEqualityConstraints = true;
    s.useFeedbackPolicy = true;
    s.printSolverStatistics = true;
    s.printSolverStatus = true;
    s.printLinesearch = true;
    s.printSolverStatistics = true;
    s.printSolverStatus = true;
    s.printLinesearch = true;
    s.nThreads = 1;
    s.initialBarrierParameter = 1.0e-02;
    s.targetBarrierParameter = 1.0e-04;
    s.barrierLinearDecreaseFactor = 0.2;
    s.barrierSuperlinearDecreasePower = 1.5;
    s.fractionToBoundaryMargin = 0.995;
    return s;
  }();

  const scalar_array_t initEventTimes{0.2262, 1.0176};
  const size_array_t modeSequence{0, 1, 2};
  auto referenceManagerPtr = getExp1ReferenceManager(initEventTimes, modeSequence);
  auto problem = createExp1Problem(referenceManagerPtr);

  const scalar_t startTime = 0.0;
  const scalar_t finalTime = 3.0;
  const vector_t initState = (vector_t(STATE_DIM) << 2.0, 3.0).finished();

  DefaultInitializer zeroInitializer(INPUT_DIM);

  // Solve
  IpmSolver solver(settings, problem, zeroInitializer);
  solver.setReferenceManager(referenceManagerPtr);
  solver.run(startTime, initState, finalTime);
}

TEST(Exp1Test, Constrained) {
  static constexpr size_t STATE_DIM = 2;
  static constexpr size_t INPUT_DIM = 1;

  // Solver settings
  const auto settings = []() {
    ipm::Settings s;
    s.dt = 0.01;
    s.ipmIteration = 20;
    s.projectStateInputEqualityConstraints = true;
    s.useFeedbackPolicy = true;
    s.printSolverStatistics = true;
    s.printSolverStatus = true;
    s.printLinesearch = true;
    s.printSolverStatistics = true;
    s.printSolverStatus = true;
    s.printLinesearch = true;
    s.nThreads = 1;
    s.initialBarrierParameter = 1.0e-02;
    s.targetBarrierParameter = 1.0e-04;
    s.barrierLinearDecreaseFactor = 0.2;
    s.barrierSuperlinearDecreasePower = 1.5;
    s.fractionToBoundaryMargin = 0.995;
    return s;
  }();

  const scalar_array_t initEventTimes{0.2262, 1.0176};
  const size_array_t modeSequence{0, 1, 2};
  auto referenceManagerPtr = getExp1ReferenceManager(initEventTimes, modeSequence);
  auto problem = createExp1Problem(referenceManagerPtr);

  // add inequality constraints
  const scalar_t umin = -1.0;
  const scalar_t umax = 1.0;
  auto stateInputIneqConstraint = std::make_unique<EXP1_StateInputIneqConstraints>(umin, umax);
  problem.inequalityConstraintPtr->add("ubound", std::move(stateInputIneqConstraint));
  const vector_t xmin = (vector_t(2) << -0.0, -0.0).finished();
  const vector_t xmax = (vector_t(2) << 3.0, 4.0).finished();
  auto stateIneqConstraint = std::make_unique<EXP1_StateIneqConstraints>(xmin, xmax);
  auto finalStateIneqConstraint = std::make_unique<EXP1_StateIneqConstraints>(xmin, xmax);
  problem.stateInequalityConstraintPtr->add("xbound", std::move(stateIneqConstraint));
  problem.finalInequalityConstraintPtr->add("xbound", std::move(finalStateIneqConstraint));

  const scalar_t startTime = 0.0;
  const scalar_t finalTime = 3.0;
  const vector_t initState = (vector_t(STATE_DIM) << 2.0, 3.0).finished();

  DefaultInitializer zeroInitializer(INPUT_DIM);

  // Solve
  IpmSolver solver(settings, problem, zeroInitializer);
  solver.setReferenceManager(referenceManagerPtr);
  solver.run(startTime, initState, finalTime);

  const auto primalSolution = solver.primalSolution(finalTime);

  // check constraint satisfaction
  for (const auto& e : primalSolution.stateTrajectory_) {
    if (e.size() > 0) {
      ASSERT_TRUE(e.coeff(0) >= xmin.coeff(0));
      ASSERT_TRUE(e.coeff(1) >= xmin.coeff(1));
      ASSERT_TRUE(e.coeff(0) <= xmax.coeff(0));
      ASSERT_TRUE(e.coeff(1) <= xmax.coeff(1));
    }
  }
  for (const auto& e : primalSolution.inputTrajectory_) {
    if (e.size() > 0) {
      ASSERT_TRUE(e.coeff(0) >= umin);
      ASSERT_TRUE(e.coeff(0) <= umax);
    }
  }
}

TEST(Exp1Test, MixedConstrained) {
  static constexpr size_t STATE_DIM = 2;
  static constexpr size_t INPUT_DIM = 1;

  // Solver settings
  const auto settings = []() {
    ipm::Settings s;
    s.dt = 0.01;
    s.ipmIteration = 20;
    s.projectStateInputEqualityConstraints = true;
    s.useFeedbackPolicy = true;
    s.printSolverStatistics = true;
    s.printSolverStatus = true;
    s.printLinesearch = true;
    s.printSolverStatistics = true;
    s.printSolverStatus = true;
    s.printLinesearch = true;
    s.nThreads = 1;
    s.initialBarrierParameter = 1.0e-02;
    s.targetBarrierParameter = 1.0e-04;
    s.barrierLinearDecreaseFactor = 0.2;
    s.barrierSuperlinearDecreasePower = 1.5;
    s.fractionToBoundaryMargin = 0.995;
    return s;
  }();

  const scalar_array_t initEventTimes{0.2262, 1.0176};
  const size_array_t modeSequence{0, 1, 2};
  auto referenceManagerPtr = getExp1ReferenceManager(initEventTimes, modeSequence);
  auto problem = createExp1Problem(referenceManagerPtr);

  // add inequality constraints
  const scalar_t xumin = -1.0;
  const scalar_t xumax = 1.0;
  auto stateInputIneqConstraint = std::make_unique<EXP1_MixedStateInputIneqConstraints>(xumin, xumax);
  auto stateInputIneqConstraintCloned = stateInputIneqConstraint->clone();
  problem.inequalityConstraintPtr->add("bound", std::move(stateInputIneqConstraint));
  const scalar_t startTime = 0.0;
  const scalar_t finalTime = 3.0;
  const vector_t initState = (vector_t(STATE_DIM) << 2.0, 3.0).finished();

  DefaultInitializer zeroInitializer(INPUT_DIM);

  // Solve
  IpmSolver solver(settings, problem, zeroInitializer);
  solver.setReferenceManager(referenceManagerPtr);
  solver.run(startTime, initState, finalTime);

  const auto primalSolution = solver.primalSolution(finalTime);

  // check constraint satisfaction
  const size_t N = primalSolution.inputTrajectory_.size();
  for (size_t i = 0; i < N; ++i) {
    const auto t = primalSolution.timeTrajectory_[i];
    const auto& x = primalSolution.stateTrajectory_[i];
    const auto& u = primalSolution.inputTrajectory_[i];
    const auto constraintValue = stateInputIneqConstraintCloned->getValue(t, x, u, PreComputation());
    ASSERT_TRUE(constraintValue.minCoeff() >= 0.0);
  }
}
