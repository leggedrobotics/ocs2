#include <gtest/gtest.h>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <memory>

#include "ocs2_ipm/IpmSolver.h"

#include <ocs2_core/initialization/DefaultInitializer.h>

#include <ocs2_oc/test/EXP0.h>

using namespace ocs2;

class EXP0_StateIneqConstraints final : public StateConstraint {
 public:
  EXP0_StateIneqConstraints(const vector_t& xmin, const vector_t& xmax)
      : StateConstraint(ConstraintOrder::Linear), xmin_(xmin), xmax_(xmax) {}
  ~EXP0_StateIneqConstraints() override = default;

  EXP0_StateIneqConstraints* clone() const override { return new EXP0_StateIneqConstraints(*this); }

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

class EXP0_StateInputIneqConstraints final : public StateInputConstraint {
 public:
  EXP0_StateInputIneqConstraints(scalar_t umin, scalar_t umax) : StateInputConstraint(ConstraintOrder::Linear), umin_(umin), umax_(umax) {}
  ~EXP0_StateInputIneqConstraints() override = default;

  EXP0_StateInputIneqConstraints* clone() const override { return new EXP0_StateInputIneqConstraints(*this); }

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

TEST(Exp0Test, Unconstrained) {
  static constexpr size_t STATE_DIM = 2;
  static constexpr size_t INPUT_DIM = 1;

  // Solver settings
  ipm::Settings settings;
  settings.dt = 0.01;
  settings.ipmIteration = 20;
  settings.projectStateInputEqualityConstraints = true;
  settings.useFeedbackPolicy = true;
  settings.printSolverStatistics = true;
  settings.printSolverStatus = true;
  settings.printLinesearch = true;
  settings.printSolverStatistics = true;
  settings.printSolverStatus = true;
  settings.printLinesearch = true;
  settings.nThreads = 1;

  settings.initialBarrierParameter = 1.0e-02;
  settings.targetBarrierParameter = 1.0e-04;
  settings.barrierLinearDecreaseFactor = 0.2;
  settings.barrierSuperlinearDecreasePower = 1.5;
  settings.fractionToBoundaryMargin = 0.995;

  const scalar_array_t initEventTimes{0.1897};
  const size_array_t modeSequence{0, 1};
  auto referenceManagerPtr = getExp0ReferenceManager(initEventTimes, modeSequence);
  auto problem = createExp0Problem(referenceManagerPtr);

  const scalar_t startTime = 0.0;
  const scalar_t finalTime = 2.0;
  const vector_t initState = (vector_t(STATE_DIM) << 0.0, 2.0).finished();

  DefaultInitializer zeroInitializer(INPUT_DIM);

  // Solve
  IpmSolver solver(settings, problem, zeroInitializer);
  solver.setReferenceManager(referenceManagerPtr);
  solver.run(startTime, initState, finalTime);
  solver.run(startTime, initState, finalTime);
  solver.run(startTime, initState, finalTime);

  const auto primalSolution = solver.primalSolution(finalTime);
  std::cout << "Optimal unconstrained trajectory" << std::endl;
  for (int i = 0; i < primalSolution.timeTrajectory_.size(); i++) {
    std::cout << "time: " << std::setprecision(4) << primalSolution.timeTrajectory_[i]
              << "\t state: " << primalSolution.stateTrajectory_[i].transpose()
              << "\t input: " << primalSolution.inputTrajectory_[i].transpose() << std::endl;
  }
}

TEST(Exp0Test, Constrained) {
  static constexpr size_t STATE_DIM = 2;
  static constexpr size_t INPUT_DIM = 1;

  // Solver settings
  ipm::Settings settings;
  settings.dt = 0.01;
  settings.ipmIteration = 20;
  settings.projectStateInputEqualityConstraints = true;
  settings.useFeedbackPolicy = true;
  settings.printSolverStatistics = true;
  settings.printSolverStatus = true;
  settings.printLinesearch = true;
  settings.printSolverStatistics = true;
  settings.printSolverStatus = true;
  settings.printLinesearch = true;
  settings.nThreads = 1;

  settings.initialBarrierParameter = 1.0e-02;
  settings.targetBarrierParameter = 1.0e-04;
  settings.barrierLinearDecreaseFactor = 0.2;
  settings.barrierSuperlinearDecreasePower = 1.5;
  settings.fractionToBoundaryMargin = 0.995;

  const scalar_array_t initEventTimes{0.1897};
  const size_array_t modeSequence{0, 1};
  auto referenceManagerPtr = getExp0ReferenceManager(initEventTimes, modeSequence);
  auto problem = createExp0Problem(referenceManagerPtr);

  // add inequality constraints
  const scalar_t umin = -7.5;
  const scalar_t umax = 7.5;
  std::unique_ptr<StateInputConstraint> stateInputIneqConstraint(new EXP0_StateInputIneqConstraints(umin, umax));
  problem.inequalityConstraintPtr->add("ubound", std::move(stateInputIneqConstraint));
  const vector_t xmin = (vector_t(2) << -7.5, -7.5).finished();
  const vector_t xmax = (vector_t(2) << 7.5, 7.5).finished();
  std::unique_ptr<StateConstraint> stateIneqConstraint(new EXP0_StateIneqConstraints(xmin, xmax));
  std::unique_ptr<StateConstraint> finalStateIneqConstraint(new EXP0_StateIneqConstraints(xmin, xmax));
  problem.stateInequalityConstraintPtr->add("xbound", std::move(stateIneqConstraint));
  problem.finalInequalityConstraintPtr->add("xbound", std::move(finalStateIneqConstraint));

  const scalar_t startTime = 0.0;
  const scalar_t finalTime = 2.0;
  const vector_t initState = (vector_t(STATE_DIM) << 0.0, 2.0).finished();

  DefaultInitializer zeroInitializer(INPUT_DIM);

  // Solve
  IpmSolver solver(settings, problem, zeroInitializer);
  solver.setReferenceManager(referenceManagerPtr);
  solver.run(startTime, initState, finalTime);
  // std::cout << solver.getBenchmarkingInformation() << std::endl;

  const auto primalSolution = solver.primalSolution(finalTime);
  std::cout << "Optimal unconstrained trajectory" << std::endl;
  for (int i = 0; i < primalSolution.timeTrajectory_.size(); i++) {
    std::cout << "time: " << std::setprecision(4) << primalSolution.timeTrajectory_[i]
              << "\t state: " << primalSolution.stateTrajectory_[i].transpose()
              << "\t input: " << primalSolution.inputTrajectory_[i].transpose() << std::endl;
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
      EXPECT_TRUE(e.coeff(0) >= umin);
      EXPECT_TRUE(e.coeff(0) <= umax);
    }
  }
}