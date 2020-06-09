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
  modelData.dynamicsStateDerivative_ = systemDerivativesPtr_->getJumpMapDerivativeState();
  modelData.dynamicsInputDerivative_ = systemDerivativesPtr_->getJumpMapDerivativeInput();

  // Final state-only equality constraint
  systemConstraintsPtr_->setCurrentStateAndControl(time, state, input);

  modelData.stateEqConstr_ = systemConstraintsPtr_->getFinalStateEqualityConstraint();
  modelData.stateEqConstrStateDerivative_ = systemConstraintsPtr_->getFinalStateEqualityConstraintDerivativesState();

  modelData.ineqConstr_.clear();             // no inequality constraint
  modelData.stateInputEqConstr_.setZero(0);  // no state-input equality constraint

  // Final cost
  costFunctionPtr_->setCurrentStateAndControl(time, state, input);
  modelData.cost_ = costFunctionPtr_->getTerminalCost();
  modelData.costStateDerivative_ = costFunctionPtr_->getTerminalCostDerivativeState();
  modelData.costStateSecondDerivative_ = costFunctionPtr_->getTerminalCostSecondDerivativeState();

  // TODO(mspieler): this isn't used anyway, maybe set to empty matrix instead?
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
  modelData.dynamicsStateDerivative_ = systemDerivativesPtr_->getFlowMapDerivativeState();
  modelData.dynamicsInputDerivative_ = systemDerivativesPtr_->getFlowMapDerivativeInput();
  modelData.dynamicsCovariance_ = systemDerivativesPtr_->getDynamicsCovariance();

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

  // State-input equality constraint
  modelData.stateInputEqConstr_ = systemConstraintsPtr_->getStateInputEqualityConstraint();
  modelData.stateInputEqConstrStateDerivative_ = systemConstraintsPtr_->getStateInputEqualityConstraintDerivativesState();
  modelData.stateInputEqConstrInputDerivative_ = systemConstraintsPtr_->getStateInputEqualityConstraintDerivativesInput();
  if (modelData.stateInputEqConstr_.rows() > input.rows()) {
    throw std::runtime_error("Number of active state-input equality constraints should be less-equal to the input dimension.");
  }

  // State-only equality constraint
  modelData.stateEqConstr_ = systemConstraintsPtr_->getStateEqualityConstraint();
  modelData.stateEqConstrStateDerivative_ = systemConstraintsPtr_->getStateEqualityConstraintDerivativesState();
  if (modelData.stateEqConstr_.rows() > input.rows()) {
    throw std::runtime_error("Number of active state-only equality constraints should be less-equal to the input dimension.");
  }

  // Inequality constraint
  modelData.ineqConstr_ = systemConstraintsPtr_->getInequalityConstraint();
  modelData.ineqConstrStateDerivative_ = systemConstraintsPtr_->getInequalityConstraintDerivativesState();
  modelData.ineqConstrInputDerivative_ = systemConstraintsPtr_->getInequalityConstraintDerivativesInput();
  modelData.ineqConstrStateSecondDerivative_ = systemConstraintsPtr_->getInequalityConstraintSecondDerivativesState();
  modelData.ineqConstrInputSecondDerivative_ = systemConstraintsPtr_->getInequalityConstraintSecondDerivativesInput();
  modelData.ineqConstrInputStateDerivative_ = systemConstraintsPtr_->getInequalityConstraintDerivativesInputState();

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
  modelData.cost_ = costFunctionPtr_->getCost();
  modelData.costStateDerivative_ = costFunctionPtr_->getCostDerivativeState();
  modelData.costStateSecondDerivative_ = costFunctionPtr_->getCostSecondDerivativeState();
  modelData.costInputDerivative_ = costFunctionPtr_->getCostDerivativeInput();
  modelData.costInputSecondDerivative_ = costFunctionPtr_->getCostSecondDerivativeInput();
  modelData.costInputStateDerivative_ = costFunctionPtr_->getCostDerivativeInputState();

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
