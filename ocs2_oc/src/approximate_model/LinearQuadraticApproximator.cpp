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

#include <iostream>

#include <ocs2_core/misc/LinearAlgebra.h>
#include <ocs2_oc/approximate_model/LinearQuadraticApproximator.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LinearQuadraticApproximator::approximateLQProblem(const scalar_t& time, const vector_t& state, const vector_t& input,
                                                       ModelData& modelData) const {
  constexpr auto request = Request::Cost + Request::SoftConstraint + Request::Constraint + Request::Dynamics + Request::Approximation;
  problemPtr_->preComputationPtr->request(request, time, state, input);

  // dynamics
  approximateDynamics(time, state, input, modelData);

  // constraints
  approximateConstraints(time, state, input, modelData);

  // cost
  approximateCost(time, state, input, modelData);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LinearQuadraticApproximator::approximateLQProblemAtEventTime(const scalar_t& time, const vector_t& state, ModelData& modelData) const {
  auto& preComputation = *problemPtr_->preComputationPtr;
  constexpr auto request = Request::Cost + Request::SoftConstraint + Request::Constraint + Request::Dynamics + Request::Approximation;
  preComputation.requestPreJump(request, time, state);

  // Jump map
  modelData.dynamics_ = problemPtr_->dynamicsPtr->jumpMapLinearApproximation(time, state, preComputation);

  // state equality constraint
  auto& targetTrajectories = *problemPtr_->targetTrajectoriesPtr;
  modelData.stateEqConstr_ =
      problemPtr_->preJumpEqualityConstraintPtr->getQuadraticApproximation(time, state, targetTrajectories, preComputation);

  // state-input inequality constraint
  modelData.stateIneqConstr_ =
      problemPtr_->preJumpInequalityConstraintPtr->getQuadraticApproximation(time, state, targetTrajectories, preComputation);

  // Pre-jump cost
  modelData.cost_ = approximateEventCost(*problemPtr_, time, state);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LinearQuadraticApproximator::approximateLQProblemAtFinalTime(const scalar_t& time, const vector_t& state, ModelData& modelData) const {
  auto& preComputation = *problemPtr_->preComputationPtr;
  constexpr auto request = Request::Cost + Request::SoftConstraint + Request::Constraint + Request::Approximation;
  preComputation.requestFinal(request, time, state);

  // state equality constraint
  auto& targetTrajectories = *problemPtr_->targetTrajectoriesPtr;
  modelData.stateEqConstr_ =
      problemPtr_->finalEqualityConstraintPtr->getQuadraticApproximation(time, state, targetTrajectories, preComputation);

  // state-input inequality constraint
  modelData.stateIneqConstr_ =
      problemPtr_->finalInequalityConstraintPtr->getQuadraticApproximation(time, state, targetTrajectories, preComputation);

  // Final cost
  modelData.cost_ = approximateFinalCost(*problemPtr_, time, state);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LinearQuadraticApproximator::approximateDynamics(const scalar_t& time, const vector_t& state, const vector_t& input,
                                                      ModelData& modelData) const {
  // get results
  modelData.dynamics_ = problemPtr_->dynamicsPtr->linearApproximation(time, state, input, *problemPtr_->preComputationPtr);
  modelData.dynamicsCovariance_ = problemPtr_->dynamicsPtr->dynamicsCovariance(time, state, input);

  // checking the numerical stability
  if (checkNumericalCharacteristics_) {
    std::string err = modelData.checkDynamicsDerivativsProperties();
    if (!err.empty()) {
      std::cerr << "what(): " << err << " at time " << time << " [sec]." << std::endl;
      std::cerr << "x: " << state.transpose() << '\n';
      std::cerr << "u: " << input.transpose() << '\n';
      std::cerr << "Am: \n" << modelData.dynamics_.dfdx << std::endl;
      std::cerr << "Bm: \n" << modelData.dynamics_.dfdu << std::endl;
      throw std::runtime_error(err);
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LinearQuadraticApproximator::approximateConstraints(const scalar_t& time, const vector_t& state, const vector_t& input,
                                                         ModelData& modelData) const {
  const auto& targetTrajectories = *problemPtr_->targetTrajectoriesPtr;

  // state-input equality constraint
  modelData.stateInputEqConstr_ =
      problemPtr_->equalityConstraintPtr->getLinearApproximation(time, state, input, *problemPtr_->preComputationPtr);
  if (modelData.stateInputEqConstr_.f.rows() > input.rows()) {
    throw std::runtime_error("Number of active state-input equality constraints should be less-equal to the input dimension.");
  }

  // state equality constraint
  modelData.stateEqConstr_ =
      problemPtr_->stateEqualityConstraintPtr->getQuadraticApproximation(time, state, targetTrajectories, *problemPtr_->preComputationPtr);

  // state inequality constraint
  modelData.stateIneqConstr_ = problemPtr_->stateInequalityConstraintPtr->getQuadraticApproximation(time, state, targetTrajectories,
                                                                                                    *problemPtr_->preComputationPtr);

  // state-input inequality constraint
  modelData.stateInputIneqConstr_ = problemPtr_->inequalityConstraintPtr->getQuadraticApproximation(time, state, input, targetTrajectories,
                                                                                                    *problemPtr_->preComputationPtr);

  if (checkNumericalCharacteristics_) {
    const std::string err = modelData.checkConstraintProperties();
    if (!err.empty()) {
      std::stringstream errorDescription;
      errorDescription << "ill-posed problem at time " << time << " [sec].\n" << err;
      throw std::runtime_error(err);
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LinearQuadraticApproximator::approximateCost(const scalar_t& time, const vector_t& state, const vector_t& input,
                                                  ModelData& modelData) const {
  modelData.cost_ = ocs2::approximateCost(*problemPtr_, time, state, input);

  // checking the numerical stability
  if (checkNumericalCharacteristics_) {
    std::string err = modelData.checkCostProperties();
    if (!err.empty()) {
      std::cerr << "ill-posed problem at time " << time << " [sec].\n";
      std::cerr << "x: " << state.transpose() << '\n';
      std::cerr << "u: " << input.transpose() << '\n';
      std::cerr << "q: " << modelData.cost_.f << '\n';
      std::cerr << "Qv: " << modelData.cost_.dfdx.transpose() << '\n';
      std::cerr << "Qm: \n" << modelData.cost_.dfdxx << '\n';
      std::cerr << "Qm eigenvalues : " << LinearAlgebra::eigenvalues(modelData.cost_.dfdxx).transpose() << '\n';
      std::cerr << "Rv: " << modelData.cost_.dfdu.transpose() << '\n';
      std::cerr << "Rm: \n" << modelData.cost_.dfduu << '\n';
      std::cerr << "Rm eigenvalues : " << LinearAlgebra::eigenvalues(modelData.cost_.dfduu).transpose() << '\n';
      std::cerr << "Pm: \n" << modelData.cost_.dfdux << '\n';
      throw std::runtime_error(err);
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t computeCost(const OptimalControlProblem& problem, const scalar_t& time, const vector_t& state, const vector_t& input) {
  const auto& targetTrajectories = *problem.targetTrajectoriesPtr;
  const auto& preComputation = *problem.preComputationPtr;

  // Compute and sum all costs
  auto cost = problem.costPtr->getValue(time, state, input, targetTrajectories, preComputation);
  cost += problem.softConstraintPtr->getValue(time, state, input, targetTrajectories, preComputation);
  cost += problem.stateCostPtr->getValue(time, state, targetTrajectories, preComputation);
  cost += problem.stateSoftConstraintPtr->getValue(time, state, targetTrajectories, preComputation);

  return cost;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation approximateCost(const OptimalControlProblem& problem, const scalar_t& time, const vector_t& state,
                                                     const vector_t& input) {
  const auto& targetTrajectories = *problem.targetTrajectoriesPtr;
  const auto& preComputation = *problem.preComputationPtr;

  // get the state-input cost approximations
  auto cost = problem.costPtr->getQuadraticApproximation(time, state, input, targetTrajectories, preComputation);

  if (!problem.softConstraintPtr->empty()) {
    cost += problem.softConstraintPtr->getQuadraticApproximation(time, state, input, targetTrajectories, preComputation);
  }

  // get the state only cost approximations
  if (!problem.stateCostPtr->empty()) {
    auto stateCost = problem.stateCostPtr->getQuadraticApproximation(time, state, targetTrajectories, preComputation);
    cost.f += stateCost.f;
    cost.dfdx += stateCost.dfdx;
    cost.dfdxx += stateCost.dfdxx;
  }

  if (!problem.stateSoftConstraintPtr->empty()) {
    auto stateCost = problem.stateSoftConstraintPtr->getQuadraticApproximation(time, state, targetTrajectories, preComputation);
    cost.f += stateCost.f;
    cost.dfdx += stateCost.dfdx;
    cost.dfdxx += stateCost.dfdxx;
  }

  return cost;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t computeEventCost(const OptimalControlProblem& problem, const scalar_t& time, const vector_t& state) {
  const auto& targetTrajectories = *problem.targetTrajectoriesPtr;
  const auto& preComputation = *problem.preComputationPtr;

  auto cost = problem.preJumpCostPtr->getValue(time, state, targetTrajectories, preComputation);
  cost += problem.preJumpSoftConstraintPtr->getValue(time, state, targetTrajectories, preComputation);

  return cost;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation approximateEventCost(const OptimalControlProblem& problem, const scalar_t& time,
                                                          const vector_t& state) {
  const auto& targetTrajectories = *problem.targetTrajectoriesPtr;
  const auto& preComputation = *problem.preComputationPtr;

  auto cost = problem.preJumpCostPtr->getQuadraticApproximation(time, state, targetTrajectories, preComputation);
  if (!problem.preJumpSoftConstraintPtr->empty()) {
    cost += problem.preJumpSoftConstraintPtr->getQuadraticApproximation(time, state, targetTrajectories, preComputation);
  }

  return cost;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t computeFinalCost(const OptimalControlProblem& problem, const scalar_t& time, const vector_t& state) {
  const auto& targetTrajectories = *problem.targetTrajectoriesPtr;
  const auto& preComputation = *problem.preComputationPtr;

  auto cost = problem.finalCostPtr->getValue(time, state, targetTrajectories, preComputation);
  cost += problem.finalSoftConstraintPtr->getValue(time, state, targetTrajectories, preComputation);

  return cost;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation approximateFinalCost(const OptimalControlProblem& problem, const scalar_t& time,
                                                          const vector_t& state) {
  const auto& targetTrajectories = *problem.targetTrajectoriesPtr;
  const auto& preComputation = *problem.preComputationPtr;

  auto cost = problem.finalCostPtr->getQuadraticApproximation(time, state, targetTrajectories, preComputation);
  if (!problem.finalSoftConstraintPtr->empty()) {
    cost += problem.finalSoftConstraintPtr->getQuadraticApproximation(time, state, targetTrajectories, preComputation);
  }

  return cost;
}

}  // namespace ocs2
