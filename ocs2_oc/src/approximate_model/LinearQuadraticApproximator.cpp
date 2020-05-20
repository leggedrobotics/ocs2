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
void LinearQuadraticApproximator::approximateUnconstrainedLQProblem(const scalar_t& time, const vector_t& state, const vector_t& input,
                                                                    ModelDataBase& modelData) {
  // dynamics
  approximateDynamics(time, state, input, modelData);

  // constraints
  approximateConstraints(time, state, input, modelData);

  // cost
  approximateIntermediateCost(time, state, input, modelData);

  // check dimensions
  modelData.checkSizes(state.rows(), input.rows());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LinearQuadraticApproximator::approximateUnconstrainedLQProblemAtEventTime(const scalar_t& time, const vector_t& state,
                                                                               const vector_t& input, ModelDataBase& modelData) {
  // Jump map
  systemDerivativesPtr_->setCurrentStateAndControl(time, state, input);

  // get results
  systemDerivativesPtr_->getJumpMapDerivativeState(modelData.dynamicsStateDerivative_);
  systemDerivativesPtr_->getJumpMapDerivativeInput(modelData.dynamicsInputDerivative_);

  // Final state-only equality constraint
  systemConstraintsPtr_->setCurrentStateAndControl(time, state, input);

  // if final constraint type 2 is active
  const size_t ncFinalEqStateOnly = systemConstraintsPtr_->numStateOnlyFinalConstraint(time);
  if (ncFinalEqStateOnly > 0) {
    systemConstraintsPtr_->getFinalConstraint2(modelData.stateEqConstr_);
    systemConstraintsPtr_->getFinalConstraint2DerivativesState(modelData.stateEqConstrStateDerivative_);
  } else {
    modelData.stateEqConstrStateDerivative_.setZero(ncFinalEqStateOnly, modelData.stateDim_);
  }
  modelData.numIneqConstr_ = 0;          // no inequality constraint
  modelData.numStateInputEqConstr_ = 0;  // no state-input equality constraint
  modelData.numStateEqConstr_ = ncFinalEqStateOnly;
  // TODO(mspieler): is slicing necessary here?
  modelData.stateEqConstr_ = modelData.stateEqConstr_.head(ncFinalEqStateOnly);
  modelData.stateEqConstrStateDerivative_ = modelData.stateEqConstrStateDerivative_.topRows(ncFinalEqStateOnly);

  // Final cost
  costFunctionPtr_->setCurrentStateAndControl(time, state, input);
  costFunctionPtr_->getTerminalCost(modelData.cost_);
  costFunctionPtr_->getTerminalCostDerivativeState(modelData.costStateDerivative_);
  costFunctionPtr_->getTerminalCostSecondDerivativeState(modelData.costStateSecondDerivative_);

  modelData.costInputDerivative_.setZero(input.rows());
  modelData.costInputSecondDerivative_.setZero(input.rows(), input.rows());
  modelData.costInputStateDerivative_.setZero(input.rows(), state.rows());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LinearQuadraticApproximator::approximateDynamics(const scalar_t& time, const vector_t& state, const vector_t& input,
                                                      ModelDataBase& modelData) {
  // set data & do computation
  systemDerivativesPtr_->setCurrentStateAndControl(time, state, input);

  // get results
  systemDerivativesPtr_->getFlowMapDerivativeState(modelData.dynamicsStateDerivative_);
  systemDerivativesPtr_->getFlowMapDerivativeInput(modelData.dynamicsInputDerivative_);
  systemDerivativesPtr_->getDynamicsCovariance(modelData.dynamicsCovariance_);

  // checking the numerical stability
  if (checkNumericalCharacteristics_) {
    std::string err = modelData.checkDynamicsDerivativsProperties();
    if (!err.empty()) {
      std::cerr << "what(): " << err << " at time " << time << " [sec]." << std::endl;
      std::cerr << "Am: \n" << modelData.dynamicsStateDerivative_ << std::endl;
      std::cerr << "Bm: \n" << modelData.dynamicsInputDerivative_ << std::endl;
      throw std::runtime_error(err);
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LinearQuadraticApproximator::approximateConstraints(const scalar_t& time, const vector_t& state, const vector_t& input,
                                                         ModelDataBase& modelData) {
  // set data
  systemConstraintsPtr_->setCurrentStateAndControl(time, state, input);

  // constraint type 1
  const size_t ncEqStateInput = systemConstraintsPtr_->numStateInputConstraint(time);
  if (ncEqStateInput > input.rows()) {
    throw std::runtime_error("Number of active type-1 constraints should be less-equal to the number of input dimension.");
  }
  // if constraint type 1 is active
  if (ncEqStateInput > 0) {
    systemConstraintsPtr_->getConstraint1(modelData.stateInputEqConstr_);
    systemConstraintsPtr_->getConstraint1DerivativesState(modelData.stateInputEqConstrStateDerivative_);
    systemConstraintsPtr_->getConstraint1DerivativesControl(modelData.stateInputEqConstrInputDerivative_);
  } else {
    modelData.stateInputEqConstrStateDerivative_.setZero(ncEqStateInput, modelData.stateDim_);
    modelData.stateInputEqConstrInputDerivative_.setZero(ncEqStateInput, modelData.inputDim_);
  }
  modelData.numStateInputEqConstr_ = ncEqStateInput;
  // TODO(mspieler): is slicing necessary here?
  modelData.stateInputEqConstr_ = modelData.stateInputEqConstr_.head(ncEqStateInput);
  modelData.stateInputEqConstrStateDerivative_ = modelData.stateInputEqConstrStateDerivative_.topRows(ncEqStateInput);
  modelData.stateInputEqConstrInputDerivative_ = modelData.stateInputEqConstrInputDerivative_.topRows(ncEqStateInput);

  // constraint type 2
  const size_t ncEqStateOnly = systemConstraintsPtr_->numStateOnlyConstraint(time);
  if (ncEqStateOnly > input.rows()) {
    throw std::runtime_error("Number of active type-2 constraints should be less-equal to the number of input dimension.");
  }
  // if constraint type 2 is active
  if (ncEqStateOnly > 0) {
    systemConstraintsPtr_->getConstraint2(modelData.stateEqConstr_);
    systemConstraintsPtr_->getConstraint2DerivativesState(modelData.stateEqConstrStateDerivative_);
  } else {
    modelData.stateEqConstrStateDerivative_.setZero(ncEqStateOnly, modelData.stateDim_);
  }
  modelData.numStateEqConstr_ = ncEqStateOnly;
  // TODO(mspieler): is slicing necessary here?
  modelData.stateEqConstr_ = modelData.stateEqConstr_.head(ncEqStateOnly);
  modelData.stateEqConstrStateDerivative_ = modelData.stateEqConstrStateDerivative_.topRows(ncEqStateOnly);

  // Inequality constraint
  const size_t ncIneq = systemConstraintsPtr_->numInequalityConstraint(time);
  if (ncIneq > 0) {
    systemConstraintsPtr_->getInequalityConstraint(modelData.ineqConstr_);
    systemConstraintsPtr_->getInequalityConstraintDerivativesState(modelData.ineqConstrStateDerivative_);
    systemConstraintsPtr_->getInequalityConstraintDerivativesInput(modelData.ineqConstrInputDerivative_);
    systemConstraintsPtr_->getInequalityConstraintSecondDerivativesState(modelData.ineqConstrStateSecondDerivative_);
    systemConstraintsPtr_->getInequalityConstraintSecondDerivativesInput(modelData.ineqConstrInputSecondDerivative_);
    systemConstraintsPtr_->getInequalityConstraintDerivativesInputState(modelData.ineqConstrInputStateDerivative_);
  }
  modelData.numIneqConstr_ = ncIneq;

  if (checkNumericalCharacteristics_) {
    std::string err = modelData.checkConstraintProperties();
    if (!err.empty()) {
      std::cerr << "what(): " << err << " at time " << time << " [sec]." << std::endl;
      std::cerr << "Ev: " << modelData.stateInputEqConstr_.transpose() << std::endl;
      std::cerr << "Cm: \n" << modelData.stateInputEqConstrStateDerivative_ << std::endl;
      std::cerr << "Dm: \n" << modelData.stateInputEqConstrInputDerivative_ << std::endl;
      std::cerr << "Hv: " << modelData.stateEqConstr_.transpose() << std::endl;
      std::cerr << "Fm: \n" << modelData.stateEqConstrStateDerivative_ << std::endl;
      throw std::runtime_error(err);
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LinearQuadraticApproximator::approximateIntermediateCost(const scalar_t& time, const vector_t& state, const vector_t& input,
                                                              ModelDataBase& modelData) {
  // set data
  costFunctionPtr_->setCurrentStateAndControl(time, state, input);

  // get results
  costFunctionPtr_->getIntermediateCost(modelData.cost_);
  costFunctionPtr_->getIntermediateCostDerivativeState(modelData.costStateDerivative_);
  costFunctionPtr_->getIntermediateCostSecondDerivativeState(modelData.costStateSecondDerivative_);
  costFunctionPtr_->getIntermediateCostDerivativeInput(modelData.costInputDerivative_);
  costFunctionPtr_->getIntermediateCostSecondDerivativeInput(modelData.costInputSecondDerivative_);
  costFunctionPtr_->getIntermediateCostDerivativeInputState(modelData.costInputStateDerivative_);

  // checking the numerical stability
  if (checkNumericalCharacteristics_) {
    std::string err = modelData.checkCostProperties();
    if (!err.empty()) {
      std::cerr << "what(): " << err << " at time " << time << " [sec]." << '\n';
      std::cerr << "x: " << state.transpose() << '\n';
      std::cerr << "u: " << input.transpose() << '\n';
      std::cerr << "q: " << modelData.cost_ << '\n';
      std::cerr << "Qv: " << modelData.costStateDerivative_.transpose() << '\n';
      std::cerr << "Qm: \n" << modelData.costStateSecondDerivative_ << '\n';
      std::cerr << "Qm eigenvalues : " << LinearAlgebra::eigenvalues(modelData.costStateSecondDerivative_).transpose() << '\n';
      std::cerr << "Rv: " << modelData.costInputDerivative_.transpose() << '\n';
      std::cerr << "Rm: \n" << modelData.costInputSecondDerivative_ << '\n';
      std::cerr << "Rm eigenvalues : " << LinearAlgebra::eigenvalues(modelData.costInputSecondDerivative_).transpose() << '\n';
      std::cerr << "Pm: \n" << modelData.costInputStateDerivative_ << '\n';
      throw std::runtime_error(err);
    }
  }
}

}  // namespace ocs2
