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

#include <gtest/gtest.h>

#include "ocs2_ipm/IpmSolver.h"

#include <ocs2_core/initialization/DefaultInitializer.h>

#include <ocs2_oc/test/circular_kinematics.h>

namespace ocs2 {

class CircleKinematics_StateIneqConstraints final : public StateConstraint {
 public:
  CircleKinematics_StateIneqConstraints(const vector_t& xmin, const vector_t& xmax)
      : StateConstraint(ConstraintOrder::Linear), xmin_(xmin), xmax_(xmax) {}
  ~CircleKinematics_StateIneqConstraints() override = default;

  CircleKinematics_StateIneqConstraints* clone() const override { return new CircleKinematics_StateIneqConstraints(*this); }

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

class CircleKinematics_StateInputIneqConstraints final : public StateInputConstraint {
 public:
  CircleKinematics_StateInputIneqConstraints(const vector_t& umin, const vector_t& umax)
      : StateInputConstraint(ConstraintOrder::Linear), umin_(umin), umax_(umax) {}
  ~CircleKinematics_StateInputIneqConstraints() override = default;

  CircleKinematics_StateInputIneqConstraints* clone() const override { return new CircleKinematics_StateInputIneqConstraints(*this); }

  size_t getNumConstraints(scalar_t time) const override { return 4; }

  vector_t getValue(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation&) const override {
    vector_t e(4);
    e.head(2) = u - umin_;
    e.tail(2) = umax_ - u;
    return e;
  }

  VectorFunctionLinearApproximation getLinearApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                           const PreComputation& preComp) const override {
    VectorFunctionLinearApproximation e;
    e.f = getValue(t, x, u, preComp);
    e.dfdx = matrix_t::Zero(4, x.size());
    e.dfdu = matrix_t::Zero(4, u.size());
    e.dfdu.topLeftCorner(2, 2) = matrix_t::Identity(2, 2);
    e.dfdu.bottomRightCorner(2, 2) = -matrix_t::Identity(2, 2);
    return e;
  }

 private:
  vector_t umin_, umax_;
};

class CircleKinematics_MixedStateInputIneqConstraints final : public StateInputConstraint {
 public:
  CircleKinematics_MixedStateInputIneqConstraints(scalar_t xumin, scalar_t xumax)
      : StateInputConstraint(ConstraintOrder::Linear), xumin_(xumin), xumax_(xumax) {}
  ~CircleKinematics_MixedStateInputIneqConstraints() override = default;

  CircleKinematics_MixedStateInputIneqConstraints* clone() const override {
    return new CircleKinematics_MixedStateInputIneqConstraints(*this);
  }

  size_t getNumConstraints(scalar_t time) const override { return 2; }

  vector_t getValue(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation&) const override {
    vector_t e(2);
    e << (x.coeff(0) * u.coeff(0) - xumin_), (xumax_ - x.coeff(1) * u.coeff(1));
    return e;
  }

  VectorFunctionLinearApproximation getLinearApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                           const PreComputation& preComp) const override {
    VectorFunctionLinearApproximation e;
    e.f = getValue(t, x, u, preComp);
    e.dfdx = (matrix_t(2, 2) << u.coeff(0), 0, 0, -u.coeff(1)).finished();
    e.dfdu = (matrix_t(2, 2) << x.coeff(0), 0, 0, -x.coeff(1)).finished();
    return e;
  }

 private:
  scalar_t xumin_, xumax_;
};

}  // namespace ocs2

TEST(test_circular_kinematics, solve_projected_EqConstraints) {
  // optimal control problem
  ocs2::OptimalControlProblem problem = ocs2::createCircularKinematicsProblem("/tmp/ocs2/ipm_test_generated");

  // Initializer
  ocs2::DefaultInitializer zeroInitializer(2);

  // Solver settings
  ocs2::ipm::Settings settings;
  settings.dt = 0.01;
  settings.ipmIteration = 20;
  settings.projectStateInputEqualityConstraints = true;
  settings.computeLagrangeMultipliers = true;
  settings.useFeedbackPolicy = true;
  settings.printSolverStatistics = true;
  settings.printSolverStatus = true;
  settings.printLinesearch = true;
  settings.nThreads = 1;

  // Additional problem definitions
  const ocs2::scalar_t startTime = 0.0;
  const ocs2::scalar_t finalTime = 1.0;
  const ocs2::vector_t initState = (ocs2::vector_t(2) << 1.0, 0.0).finished();  // radius 1.0

  // Solve
  ocs2::IpmSolver solver(settings, problem, zeroInitializer);
  solver.run(startTime, initState, finalTime);

  // Inspect solution
  const auto primalSolution = solver.primalSolution(finalTime);
  for (int i = 0; i < primalSolution.timeTrajectory_.size(); i++) {
    std::cout << "time: " << primalSolution.timeTrajectory_[i] << "\t state: " << primalSolution.stateTrajectory_[i].transpose()
              << "\t input: " << primalSolution.inputTrajectory_[i].transpose() << std::endl;
  }

  // Check initial condition
  ASSERT_TRUE(primalSolution.stateTrajectory_.front().isApprox(initState));
  ASSERT_DOUBLE_EQ(primalSolution.timeTrajectory_.front(), startTime);
  ASSERT_DOUBLE_EQ(primalSolution.timeTrajectory_.back(), finalTime);

  // Check constraint satisfaction.
  const auto performance = solver.getPerformanceIndeces();
  ASSERT_LT(performance.dynamicsViolationSSE, 1e-6);
  ASSERT_LT(performance.equalityConstraintsSSE, 1e-6);

  // Check feedback controller
  for (int i = 0; i < primalSolution.timeTrajectory_.size() - 1; i++) {
    const auto t = primalSolution.timeTrajectory_[i];
    const auto& x = primalSolution.stateTrajectory_[i];
    const auto& u = primalSolution.inputTrajectory_[i];
    // Feed forward part
    ASSERT_TRUE(u.isApprox(primalSolution.controllerPtr_->computeInput(t, x)));
  }
}

TEST(test_circular_kinematics, solve_EqConstraints_inQPSubproblem) {
  // optimal control problem
  ocs2::OptimalControlProblem problem = ocs2::createCircularKinematicsProblem("/tmp/ipm_test_generated");

  // Initializer
  ocs2::DefaultInitializer zeroInitializer(2);

  // Solver settings
  ocs2::ipm::Settings settings;
  settings.dt = 0.01;
  settings.ipmIteration = 20;
  settings.projectStateInputEqualityConstraints = false;  // <- false to turn off projection of state-input equalities
  settings.computeLagrangeMultipliers = true;
  settings.useFeedbackPolicy = true;
  settings.printSolverStatistics = true;
  settings.printSolverStatus = true;
  settings.printLinesearch = true;

  // Additional problem definitions
  const ocs2::scalar_t startTime = 0.0;
  const ocs2::scalar_t finalTime = 1.0;
  const ocs2::vector_t initState = (ocs2::vector_t(2) << 1.0, 0.0).finished();  // radius 1.0

  // Solve
  ocs2::IpmSolver solver(settings, problem, zeroInitializer);
  solver.run(startTime, initState, finalTime);

  // Inspect solution
  const auto primalSolution = solver.primalSolution(finalTime);
  for (int i = 0; i < primalSolution.timeTrajectory_.size(); i++) {
    std::cout << "time: " << primalSolution.timeTrajectory_[i] << "\t state: " << primalSolution.stateTrajectory_[i].transpose()
              << "\t input: " << primalSolution.inputTrajectory_[i].transpose() << std::endl;
  }

  // Check initial condition
  ASSERT_TRUE(primalSolution.stateTrajectory_.front().isApprox(initState));
  ASSERT_DOUBLE_EQ(primalSolution.timeTrajectory_.front(), startTime);
  ASSERT_DOUBLE_EQ(primalSolution.timeTrajectory_.back(), finalTime);

  // Check constraint satisfaction.
  const auto performance = solver.getPerformanceIndeces();
  ASSERT_LT(performance.dynamicsViolationSSE, 1e-6);
  ASSERT_LT(performance.equalityConstraintsSSE, 1e-6);

  // Check feedback controller
  for (int i = 0; i < primalSolution.timeTrajectory_.size() - 1; i++) {
    const auto t = primalSolution.timeTrajectory_[i];
    const auto& x = primalSolution.stateTrajectory_[i];
    const auto& u = primalSolution.inputTrajectory_[i];
    // Feed forward part
    ASSERT_TRUE(u.isApprox(primalSolution.controllerPtr_->computeInput(t, x)));
  }
}

TEST(test_circular_kinematics, solve_projected_EqConstraints_IneqConstraints) {
  // optimal control problem
  ocs2::OptimalControlProblem problem = ocs2::createCircularKinematicsProblem("/tmp/ocs2/ipm_test_generated");

  // inequality constraints
  const ocs2::vector_t umin = (ocs2::vector_t(2) << -0.5, -0.5).finished();
  const ocs2::vector_t umax = (ocs2::vector_t(2) << 0.5, 0.5).finished();
  std::unique_ptr<ocs2::StateInputConstraint> stateInputIneqConstraint(new ocs2::CircleKinematics_StateInputIneqConstraints(umin, umax));
  problem.inequalityConstraintPtr->add("ubound", std::move(stateInputIneqConstraint));
  const ocs2::vector_t xmin = (ocs2::vector_t(2) << -0.5, -0.5).finished();
  const ocs2::vector_t xmax = (ocs2::vector_t(2) << 1.0e03, 1.0e03).finished();  // no upper bound
  std::unique_ptr<ocs2::StateConstraint> stateIneqConstraint(new ocs2::CircleKinematics_StateIneqConstraints(xmin, xmax));
  std::unique_ptr<ocs2::StateConstraint> finalStateIneqConstraint(new ocs2::CircleKinematics_StateIneqConstraints(xmin, xmax));
  problem.stateInequalityConstraintPtr->add("xbound", std::move(stateIneqConstraint));
  problem.finalInequalityConstraintPtr->add("xbound", std::move(finalStateIneqConstraint));

  // Initializer
  ocs2::DefaultInitializer zeroInitializer(2);

  // Solver settings
  ocs2::ipm::Settings settings;
  settings.dt = 0.01;
  settings.ipmIteration = 40;
  settings.projectStateInputEqualityConstraints = true;
  settings.computeLagrangeMultipliers = true;
  settings.useFeedbackPolicy = true;
  settings.printSolverStatistics = true;
  settings.printSolverStatus = true;
  settings.printLinesearch = true;
  settings.nThreads = 1;

  settings.initialBarrierParameter = 1.0e-02;
  settings.targetBarrierParameter = 1.0e-04;
  settings.barrierLinearDecreaseFactor = 0.2;
  settings.barrierSuperlinearDecreasePower = 1.5;
  settings.fractionToBoundaryMargin = 0.995;

  // Additional problem definitions
  const ocs2::scalar_t startTime = 0.0;
  const ocs2::scalar_t finalTime = 1.0;
  const ocs2::vector_t initState = (ocs2::vector_t(2) << 1.0, 0.0).finished();  // radius 1.0

  // Solve
  ocs2::IpmSolver solver(settings, problem, zeroInitializer);
  solver.run(startTime, initState, finalTime);

  // Inspect solution
  const auto primalSolution = solver.primalSolution(finalTime);
  for (int i = 0; i < primalSolution.timeTrajectory_.size(); i++) {
    std::cout << "time: " << std::setprecision(4) << primalSolution.timeTrajectory_[i]
              << "\t state: " << primalSolution.stateTrajectory_[i].transpose()
              << "\t input: " << primalSolution.inputTrajectory_[i].transpose() << std::endl;
  }

  // check constraint satisfaction
  for (int i = 0; i < primalSolution.timeTrajectory_.size() - 1; i++) {
    if (primalSolution.inputTrajectory_[i].size() > 0) {
      const ocs2::scalar_t projectionConstraintViolation = primalSolution.stateTrajectory_[i].dot(primalSolution.inputTrajectory_[i]);
      EXPECT_LT(std::abs(projectionConstraintViolation), 1.0e-06);
    }
  }

  // check constraint satisfaction
  for (const auto& e : primalSolution.stateTrajectory_) {
    if (e.size() > 0) {
      EXPECT_TRUE(e.coeff(0) >= xmin.coeff(0));
      EXPECT_TRUE(e.coeff(1) >= xmin.coeff(1));
      EXPECT_TRUE(e.coeff(0) <= xmax.coeff(0));
      EXPECT_TRUE(e.coeff(1) <= xmax.coeff(1));
    }
  }
  for (const auto& e : primalSolution.inputTrajectory_) {
    if (e.size() > 0) {
      EXPECT_TRUE(e.coeff(0) >= umin.coeff(0));
      EXPECT_TRUE(e.coeff(1) >= umin.coeff(1));
      EXPECT_TRUE(e.coeff(0) <= umax.coeff(0));
      EXPECT_TRUE(e.coeff(1) <= umax.coeff(1));
    }
  }

  // Check initial condition
  ASSERT_TRUE(primalSolution.stateTrajectory_.front().isApprox(initState));
  ASSERT_DOUBLE_EQ(primalSolution.timeTrajectory_.front(), startTime);
  ASSERT_DOUBLE_EQ(primalSolution.timeTrajectory_.back(), finalTime);

  // Check constraint satisfaction.
  const auto performance = solver.getPerformanceIndeces();
  ASSERT_LT(performance.dynamicsViolationSSE, 1e-6);
  ASSERT_LT(performance.equalityConstraintsSSE, 1e-6);

  // Check feedback controller
  for (int i = 0; i < primalSolution.timeTrajectory_.size() - 1; i++) {
    const auto t = primalSolution.timeTrajectory_[i];
    const auto& x = primalSolution.stateTrajectory_[i];
    const auto& u = primalSolution.inputTrajectory_[i];
    // Feed forward part
    ASSERT_TRUE(u.isApprox(primalSolution.controllerPtr_->computeInput(t, x)));
  }
}

TEST(test_circular_kinematics, solve_projected_EqConstraints_MixedIneqConstraints) {
  // optimal control problem
  ocs2::OptimalControlProblem problem = ocs2::createCircularKinematicsProblem("/tmp/ocs2/ipm_test_generated");

  // inequality constraints
  const ocs2::scalar_t xumin = -2.0;
  const ocs2::scalar_t xumax = 2.0;
  std::unique_ptr<ocs2::StateInputConstraint> stateInputIneqConstraint(
      new ocs2::CircleKinematics_MixedStateInputIneqConstraints(xumin, xumax));
  auto stateInputIneqConstraintCloned = stateInputIneqConstraint->clone();
  problem.inequalityConstraintPtr->add("xubound", std::move(stateInputIneqConstraint));

  // Initializer
  ocs2::DefaultInitializer zeroInitializer(2);

  // Solver settings
  ocs2::ipm::Settings settings;
  settings.dt = 0.01;
  settings.ipmIteration = 100;
  settings.projectStateInputEqualityConstraints = true;
  settings.computeLagrangeMultipliers = true;
  settings.useFeedbackPolicy = true;
  settings.printSolverStatistics = true;
  settings.printSolverStatus = true;
  settings.printLinesearch = true;
  settings.nThreads = 1;

  settings.initialBarrierParameter = 1.0e-02;
  settings.targetBarrierParameter = 1.0e-04;
  settings.barrierLinearDecreaseFactor = 0.2;
  settings.barrierSuperlinearDecreasePower = 1.5;
  settings.fractionToBoundaryMargin = 0.995;

  // Additional problem definitions
  const ocs2::scalar_t startTime = 0.0;
  const ocs2::scalar_t finalTime = 1.0;
  const ocs2::vector_t initState = (ocs2::vector_t(2) << 1.0, 0.0).finished();  // radius 1.0

  // Solve
  ocs2::IpmSolver solver(settings, problem, zeroInitializer);
  solver.run(startTime, initState, finalTime);

  // Inspect solution
  const auto primalSolution = solver.primalSolution(finalTime);
  for (int i = 0; i < primalSolution.timeTrajectory_.size(); i++) {
    std::cout << "time: " << std::setprecision(4) << primalSolution.timeTrajectory_[i]
              << "\t state: " << primalSolution.stateTrajectory_[i].transpose()
              << "\t input: " << primalSolution.inputTrajectory_[i].transpose() << std::endl;
  }

  // check constraint satisfaction
  for (int i = 0; i < primalSolution.timeTrajectory_.size() - 1; i++) {
    if (primalSolution.inputTrajectory_[i].size() > 0) {
      const ocs2::scalar_t projectionConstraintViolation = primalSolution.stateTrajectory_[i].dot(primalSolution.inputTrajectory_[i]);
      EXPECT_LT(std::abs(projectionConstraintViolation), 1.0e-06);
    }
  }

  // check constraint satisfaction
  const size_t N = primalSolution.inputTrajectory_.size();
  for (size_t i = 0; i < N; ++i) {
    const auto t = primalSolution.timeTrajectory_[i];
    const auto& x = primalSolution.stateTrajectory_[i];
    const auto& u = primalSolution.inputTrajectory_[i];
    const auto constraintValue = stateInputIneqConstraintCloned->getValue(t, x, u, ocs2::PreComputation());
    EXPECT_TRUE(constraintValue.minCoeff() >= 0.0);
  }

  // Check initial condition
  ASSERT_TRUE(primalSolution.stateTrajectory_.front().isApprox(initState));
  ASSERT_DOUBLE_EQ(primalSolution.timeTrajectory_.front(), startTime);
  ASSERT_DOUBLE_EQ(primalSolution.timeTrajectory_.back(), finalTime);

  // Check constraint satisfaction.
  const auto performance = solver.getPerformanceIndeces();
  ASSERT_LT(performance.dynamicsViolationSSE, 1e-6);
  ASSERT_LT(performance.equalityConstraintsSSE, 1e-6);

  // Check feedback controller
  for (int i = 0; i < primalSolution.timeTrajectory_.size() - 1; i++) {
    const auto t = primalSolution.timeTrajectory_[i];
    const auto& x = primalSolution.stateTrajectory_[i];
    const auto& u = primalSolution.inputTrajectory_[i];
    // Feed forward part
    ASSERT_TRUE(u.isApprox(primalSolution.controllerPtr_->computeInput(t, x)));
  }
}