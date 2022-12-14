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

#include "ocs2_python_interface/PythonInterface.h"

#include <ocs2_core/misc/LinearAlgebra.h>
#include <ocs2_core/penalties/MultidimensionalPenalty.h>

#include <ocs2_oc/approximate_model/LinearQuadraticApproximator.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void PythonInterface::init(const RobotInterface& robot, std::unique_ptr<MPC_BASE> mpcPtr) {
  if (!mpcPtr) {
    throw std::runtime_error("[PythonInterface] Mpc pointer must be initialized before passing to the Python interface.");
  }
  mpcPtr_ = std::move(mpcPtr);

  mpcMrtInterface_.reset(new MPC_MRT_Interface(*mpcPtr_));

  problem_ = robot.getOptimalControlProblem();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void PythonInterface::reset(TargetTrajectories targetTrajectories) {
  targetTrajectories_ = std::move(targetTrajectories);
  mpcMrtInterface_->resetMpcNode(targetTrajectories_);
  problem_.targetTrajectoriesPtr = &targetTrajectories_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void PythonInterface::setObservation(scalar_t t, Eigen::Ref<const vector_t> x, Eigen::Ref<const vector_t> u) {
  SystemObservation observation;
  observation.time = t;
  observation.state = x;
  observation.input = u;
  mpcMrtInterface_->setCurrentObservation(observation);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void PythonInterface::setTargetTrajectories(TargetTrajectories targetTrajectories) {
  targetTrajectories_ = std::move(targetTrajectories);
  problem_.targetTrajectoriesPtr = &targetTrajectories_;
  mpcMrtInterface_->getReferenceManager().setTargetTrajectories(targetTrajectories_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void PythonInterface::advanceMpc() {
  mpcMrtInterface_->advanceMpc();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void PythonInterface::getMpcSolution(scalar_array_t& t, vector_array_t& x, vector_array_t& u) {
  mpcMrtInterface_->updatePolicy();
  t = mpcMrtInterface_->getPolicy().timeTrajectory_;
  x = mpcMrtInterface_->getPolicy().stateTrajectory_;
  u = mpcMrtInterface_->getPolicy().inputTrajectory_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t PythonInterface::getLinearFeedbackGain(scalar_t time) {
  return mpcMrtInterface_->getLinearFeedbackGain(time);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t PythonInterface::flowMap(scalar_t t, Eigen::Ref<const vector_t> x, Eigen::Ref<const vector_t> u) {
  return problem_.dynamicsPtr->computeFlowMap(t, x, u);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation PythonInterface::flowMapLinearApproximation(scalar_t t, Eigen::Ref<const vector_t> x,
                                                                              Eigen::Ref<const vector_t> u) {
  return problem_.dynamicsPtr->linearApproximation(t, x, u);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t PythonInterface::cost(scalar_t t, Eigen::Ref<const vector_t> x, Eigen::Ref<const vector_t> u) {
  auto& preComputation = *problem_.preComputationPtr;
  const auto request = Request::Cost + Request::SoftConstraint + Request::Constraint;
  preComputation.request(request, t, x, u);

  // cost
  scalar_t cost = computeCost(problem_, t, x, u);

  // Lagrangians
  const auto m = mpcMrtInterface_->getIntermediateDualSolution(t);
  if (!problem_.stateEqualityLagrangianPtr->empty()) {
    cost += sumPenalties(problem_.stateEqualityLagrangianPtr->getValue(t, x, m.stateEq, preComputation));
  }
  if (!problem_.stateInequalityLagrangianPtr->empty()) {
    cost += sumPenalties(problem_.stateInequalityLagrangianPtr->getValue(t, x, m.stateIneq, preComputation));
  }
  if (!problem_.equalityLagrangianPtr->empty()) {
    cost += sumPenalties(problem_.equalityLagrangianPtr->getValue(t, x, u, m.stateInputEq, preComputation));
  }
  if (!problem_.inequalityLagrangianPtr->empty()) {
    cost += sumPenalties(problem_.inequalityLagrangianPtr->getValue(t, x, u, m.stateInputIneq, preComputation));
  }

  return cost;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation PythonInterface::costQuadraticApproximation(scalar_t t, Eigen::Ref<const vector_t> x,
                                                                                 Eigen::Ref<const vector_t> u) {
  auto& preComputation = *problem_.preComputationPtr;
  const auto request = Request::Cost + Request::SoftConstraint + Request::Constraint + Request::Approximation;
  preComputation.request(request, t, x, u);

  // cost
  auto cost = approximateCost(problem_, t, x, u);

  // Lagrangians
  const auto m = mpcMrtInterface_->getIntermediateDualSolution(t);
  if (!problem_.stateEqualityLagrangianPtr->empty()) {
    auto approx = problem_.stateEqualityLagrangianPtr->getQuadraticApproximation(t, x, m.stateEq, preComputation);
    cost.f += approx.f;
    cost.dfdx += approx.dfdx;
    cost.dfdxx += approx.dfdxx;
  }
  if (!problem_.stateInequalityLagrangianPtr->empty()) {
    auto approx = problem_.stateInequalityLagrangianPtr->getQuadraticApproximation(t, x, m.stateIneq, preComputation);
    cost.f += approx.f;
    cost.dfdx += approx.dfdx;
    cost.dfdxx += approx.dfdxx;
  }
  if (!problem_.equalityLagrangianPtr->empty()) {
    cost += problem_.equalityLagrangianPtr->getQuadraticApproximation(t, x, u, m.stateInputEq, preComputation);
  }
  if (!problem_.inequalityLagrangianPtr->empty()) {
    cost += problem_.inequalityLagrangianPtr->getQuadraticApproximation(t, x, u, m.stateInputIneq, preComputation);
  }

  return cost;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t PythonInterface::valueFunction(scalar_t t, Eigen::Ref<const vector_t> x) {
  return mpcMrtInterface_->getValueFunction(t, x).f;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t PythonInterface::valueFunctionStateDerivative(scalar_t t, Eigen::Ref<const vector_t> x) {
  return mpcMrtInterface_->getValueFunction(t, x).dfdx;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t PythonInterface::stateInputEqualityConstraint(scalar_t t, Eigen::Ref<const vector_t> x, Eigen::Ref<const vector_t> u) {
  problem_.preComputationPtr->request(Request::Constraint, t, x, u);
  return toVector(problem_.equalityConstraintPtr->getValue(t, x, u, *problem_.preComputationPtr));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation PythonInterface::stateInputEqualityConstraintLinearApproximation(scalar_t t, Eigen::Ref<const vector_t> x,
                                                                                                   Eigen::Ref<const vector_t> u) {
  problem_.preComputationPtr->request(Request::Constraint + Request::Approximation, t, x, u);
  return problem_.equalityConstraintPtr->getLinearApproximation(t, x, u, *problem_.preComputationPtr);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t PythonInterface::stateInputEqualityConstraintLagrangian(scalar_t t, Eigen::Ref<const vector_t> x, Eigen::Ref<const vector_t> u) {
  vector_t zero_u = vector_t::Zero(u.rows());

  const auto g = stateInputEqualityConstraintLinearApproximation(t, x, zero_u);
  const matrix_t& Dm = g.dfdu;
  const vector_t& c = g.f;

  const auto Phi = costQuadraticApproximation(t, x, zero_u);
  const matrix_t& R = Phi.dfduu;
  const vector_t& r = Phi.dfdu;

  const matrix_t B = flowMapLinearApproximation(t, x, zero_u).dfdu;

  matrix_t RinvChol;
  LinearAlgebra::computeInverseMatrixUUT(R, RinvChol);
  matrix_t DmDager, DdaggerT_R_Ddagger_Chol, RmInvConstrainedChol;
  ocs2::LinearAlgebra::computeConstraintProjection(Dm, RinvChol, DmDager, DdaggerT_R_Ddagger_Chol, RmInvConstrainedChol);

  vector_t costate = valueFunctionStateDerivative(t, x);

  return DmDager.transpose() * (R * DmDager * c - r - B.transpose() * costate);
}

}  // namespace ocs2
