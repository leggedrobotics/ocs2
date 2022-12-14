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

  const scalar_array_t initEventTimes{0.2262, 1.0176};
  const size_array_t modeSequence{0, 1, 2};
  auto referenceManagerPtr = getExp1ReferenceManager(initEventTimes, modeSequence);
  auto problem = createExp1Problem(referenceManagerPtr);

  const scalar_t startTime = 0.0;
  const scalar_t finalTime = 3.0;
  const vector_t initState = (vector_t(STATE_DIM) << 2.0, 3.0).finished();

  DefaultInitializer zeroInitializer(INPUT_DIM);

  auto initializerPtr = std::unique_ptr<Initializer>(new DefaultInitializer(INPUT_DIM));

  // Solve
  IpmSolver solver(settings, problem, zeroInitializer);
  solver.setReferenceManager(referenceManagerPtr);
  solver.run(startTime, initState, finalTime);

  const auto primalSolution = solver.primalSolution(finalTime);
  std::cout << "Optimal unconstrained trajectory" << std::endl;
  for (int i = 0; i < primalSolution.timeTrajectory_.size(); i++) {
    std::cout << "time: " << std::setprecision(4) << primalSolution.timeTrajectory_[i]
              << "\t state: " << primalSolution.stateTrajectory_[i].transpose()
              << "\t input: " << primalSolution.inputTrajectory_[i].transpose() << std::endl;
  }
}

TEST(Exp1Test, Constrained) {
  static constexpr size_t STATE_DIM = 2;
  static constexpr size_t INPUT_DIM = 1;

  // Solver settings
  ipm::Settings settings;
  settings.dt = 0.01;
  settings.ipmIteration = 100;
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

  const scalar_array_t initEventTimes{0.2262, 1.0176};
  const size_array_t modeSequence{0, 1, 2};
  auto referenceManagerPtr = getExp1ReferenceManager(initEventTimes, modeSequence);
  auto problem = createExp1Problem(referenceManagerPtr);

  // add inequality constraints
  const scalar_t umin = -1.0;
  const scalar_t umax = 1.0;
  std::unique_ptr<StateInputConstraint> stateInputIneqConstraint(new EXP1_StateInputIneqConstraints(umin, umax));
  problem.inequalityConstraintPtr->add("ubound", std::move(stateInputIneqConstraint));
  const vector_t xmin = (vector_t(2) << -0.0, -0.0).finished();
  const vector_t xmax = (vector_t(2) << 3.0, 4.0).finished();
  std::unique_ptr<StateConstraint> stateIneqConstraint(new EXP1_StateIneqConstraints(xmin, xmax));
  std::unique_ptr<StateConstraint> finalStateIneqConstraint(new EXP1_StateIneqConstraints(xmin, xmax));
  problem.stateInequalityConstraintPtr->add("xbound", std::move(stateIneqConstraint));
  problem.finalInequalityConstraintPtr->add("xbound", std::move(finalStateIneqConstraint));

  const scalar_t startTime = 0.0;
  const scalar_t finalTime = 3.0;
  const vector_t initState = (vector_t(STATE_DIM) << 2.0, 3.0).finished();

  DefaultInitializer zeroInitializer(INPUT_DIM);

  auto initializerPtr = std::unique_ptr<Initializer>(new DefaultInitializer(INPUT_DIM));

  // Solve
  IpmSolver solver(settings, problem, zeroInitializer);
  solver.setReferenceManager(referenceManagerPtr);
  solver.run(startTime, initState, finalTime);

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

TEST(Exp1Test, MixedConstrained) {
  static constexpr size_t STATE_DIM = 2;
  static constexpr size_t INPUT_DIM = 1;

  // Solver settings
  ipm::Settings settings;
  settings.dt = 0.01;
  settings.ipmIteration = 100;
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

  const scalar_array_t initEventTimes{0.2262, 1.0176};
  const size_array_t modeSequence{0, 1, 2};
  auto referenceManagerPtr = getExp1ReferenceManager(initEventTimes, modeSequence);
  auto problem = createExp1Problem(referenceManagerPtr);

  // add inequality constraints
  const scalar_t xumin = -1.0;
  const scalar_t xumax = 1.0;
  std::unique_ptr<StateInputConstraint> stateInputIneqConstraint(new EXP1_MixedStateInputIneqConstraints(xumin, xumax));
  auto stateInputIneqConstraintCloned = stateInputIneqConstraint->clone();
  problem.inequalityConstraintPtr->add("bound", std::move(stateInputIneqConstraint));
  const scalar_t startTime = 0.0;
  const scalar_t finalTime = 3.0;
  const vector_t initState = (vector_t(STATE_DIM) << 2.0, 3.0).finished();

  DefaultInitializer zeroInitializer(INPUT_DIM);

  auto initializerPtr = std::unique_ptr<Initializer>(new DefaultInitializer(INPUT_DIM));

  // Solve
  IpmSolver solver(settings, problem, zeroInitializer);
  solver.setReferenceManager(referenceManagerPtr);
  solver.run(startTime, initState, finalTime);

  const auto primalSolution = solver.primalSolution(finalTime);
  std::cout << "Optimal unconstrained trajectory" << std::endl;
  for (int i = 0; i < primalSolution.timeTrajectory_.size(); i++) {
    std::cout << "time: " << std::setprecision(4) << primalSolution.timeTrajectory_[i]
              << "\t state: " << primalSolution.stateTrajectory_[i].transpose()
              << "\t input: " << primalSolution.inputTrajectory_[i].transpose() << std::endl;
  }

  // check constraint satisfaction
  const size_t N = primalSolution.inputTrajectory_.size();
  for (size_t i = 0; i < N; ++i) {
    const auto t = primalSolution.timeTrajectory_[i];
    const auto& x = primalSolution.stateTrajectory_[i];
    const auto& u = primalSolution.inputTrajectory_[i];
    const auto constraintValue = stateInputIneqConstraintCloned->getValue(t, x, u, PreComputation());
    EXPECT_TRUE(constraintValue.minCoeff() >= 0.0);
  }
}
