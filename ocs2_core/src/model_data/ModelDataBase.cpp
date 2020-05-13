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
#include <ocs2_core/model_data/ModelDataBase.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ModelDataBase* ModelDataBase::clone() const {
  return new ModelDataBase(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ModelDataBase::display() const {
  std::cerr << *this;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ModelDataBase::checkSizes(int stateDim, int inputDim) const {
  assert(stateDim_ == stateDim);
  assert(inputDim_ == inputDim);

  // dynamics flow map
  assert(dynamics_.size() == stateDim);
  assert(dynamicsBias_.size() == stateDim);
  assert(dynamicsStateDerivative_.rows() == stateDim);
  assert(dynamicsStateDerivative_.cols() == stateDim);
  assert(dynamicsInputDerivative_.rows() == stateDim);
  assert(dynamicsInputDerivative_.cols() == inputDim);

  // cost
  assert(costStateDerivative_.size() == stateDim);
  assert(costStateSecondDerivative_.rows() == stateDim);
  assert(costStateSecondDerivative_.cols() == stateDim);
  assert(costInputDerivative_.size() == inputDim);
  assert(costInputSecondDerivative_.rows() == inputDim);
  assert(costInputSecondDerivative_.cols() == inputDim);
  assert(costInputStateDerivative_.rows() == inputDim);
  assert(costInputStateDerivative_.cols() == stateDim);

  // state equality constraints
  assert(stateEqConstr_.size() == numStateEqConstr_);
  assert(stateEqConstrStateDerivative_.rows() == stateEqConstr_.size());
  assert(stateEqConstrStateDerivative_.cols() == stateDim);

  // state-input equality constraints
  assert(stateInputEqConstr_.size() == numStateInputEqConstr_);
  assert(stateInputEqConstrStateDerivative_.rows() == stateInputEqConstr_.size());
  assert(stateInputEqConstrStateDerivative_.cols() == stateDim);
  assert(stateInputEqConstrInputDerivative_.rows() == stateInputEqConstr_.size());
  assert(stateInputEqConstrInputDerivative_.cols() == inputDim);

  // inequality constraints
  assert(ineqConstr_.size() == numIneqConstr_);
  assert(ineqConstrStateDerivative_.size() == ineqConstr_.size());
  assert(ineqConstrInputDerivative_.size() == ineqConstr_.size());
  assert(ineqConstrStateSecondDerivative_.size() == ineqConstr_.size());
  assert(ineqConstrInputSecondDerivative_.size() == ineqConstr_.size());
  assert(ineqConstrInputStateDerivative_.size() == ineqConstr_.size());
  for (int i = 0; i < ineqConstr_.size(); i++) {
    assert(ineqConstrStateDerivative_[i].size() == stateDim);
    assert(ineqConstrInputDerivative_[i].size() == inputDim);
    assert(ineqConstrStateSecondDerivative_[i].rows() == stateDim);
    assert(ineqConstrStateSecondDerivative_[i].cols() == stateDim);
    assert(ineqConstrInputSecondDerivative_[i].rows() == inputDim);
    assert(ineqConstrInputSecondDerivative_[i].cols() == inputDim);
    assert(ineqConstrInputStateDerivative_[i].rows() == inputDim);
    assert(ineqConstrInputStateDerivative_[i].cols() == stateDim);
  }  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::string ModelDataBase::checkCostProperties() const {
  std::stringstream errorDescription;

  if (cost_ != cost_) {
    errorDescription << "Intermediate cost is not finite.\n";
  }
  if (!costStateDerivative_.allFinite()) {
    errorDescription << "Intermediate cost first derivative w.r.t. state is not finite.\n";
  }
  if (!costStateSecondDerivative_.allFinite()) {
    errorDescription << "Intermediate cost second derivative w.r.t. state is not finite.\n";
  }
  if (!costStateSecondDerivative_.isApprox(costStateSecondDerivative_.transpose())) {
    errorDescription << "Intermediate cost second derivative w.r.t. state is not self-adjoint.\n";
  }
  if (LinearAlgebra::eigenvalues(costStateSecondDerivative_).real().minCoeff() < -Eigen::NumTraits<scalar_t>::epsilon()) {
    errorDescription << "Q matrix is not positive semi-definite. It's smallest eigenvalue is " +
                            std::to_string(LinearAlgebra::eigenvalues(costStateSecondDerivative_).real().minCoeff()) + ".\n";
  }
  if (!costInputDerivative_.allFinite()) {
    errorDescription << "Intermediate cost first derivative w.r.t. input is not finite.\n";
  }
  if (!costInputSecondDerivative_.allFinite()) {
    errorDescription << "Intermediate cost second derivative w.r.t. input is not finite.\n";
  }
  if (!costInputSecondDerivative_.isApprox(costInputSecondDerivative_.transpose())) {
    errorDescription << "Intermediate cost second derivative w.r.t. input is not self-adjoint.\n";
  }
  if (!costInputStateDerivative_.allFinite()) {
    errorDescription << "Intermediate cost second derivative w.r.t. input-state is not finite.\n";
  }
  if (costInputSecondDerivative_.ldlt().rcond() < Eigen::NumTraits<scalar_t>::epsilon()) {
    errorDescription << "R matrix is not invertible. It's reciprocal condition number is " +
                            std::to_string(costInputSecondDerivative_.ldlt().rcond()) + ".\n";
  }
  if (LinearAlgebra::eigenvalues(costInputSecondDerivative_).real().minCoeff() < Eigen::NumTraits<scalar_t>::epsilon()) {
    errorDescription << "R matrix is not positive definite. It's smallest eigenvalue is " +
                            std::to_string(LinearAlgebra::eigenvalues(costInputSecondDerivative_).real().minCoeff()) + ".\n";
  }

  return errorDescription.str();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::string ModelDataBase::checkDynamicsDerivativsProperties() const {
  std::stringstream errorDescription;

  if (!dynamics_.allFinite()) {
    errorDescription << "Dynamics is not finite.";
  }
  if (!dynamicsBias_.allFinite()) {
    errorDescription << "Dynamics bias is not finite.";
  }
  if (!dynamicsStateDerivative_.allFinite()) {
    errorDescription << "Dynamics derivative w.r.t. state is not finite.";
  }
  if (!dynamicsInputDerivative_.allFinite()) {
    errorDescription << "Dynamics derivative w.r.t. input is not finite.";
  }

  return errorDescription.str();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::string ModelDataBase::checkConstraintProperties() const {
  std::stringstream errorDescription;

  if (numStateInputEqConstr_ > 0) {
    if (!stateInputEqConstr_.head(numStateInputEqConstr_).allFinite()) {
      errorDescription << "Input-state constraint is not finite.\n";
    }
    if (!stateInputEqConstrStateDerivative_.topRows(numStateInputEqConstr_).allFinite()) {
      errorDescription << "Input-state constraint derivative w.r.t. state is not finite.\n";
    }
    if (!stateInputEqConstrInputDerivative_.topRows(numStateInputEqConstr_).allFinite()) {
      errorDescription << "Input-state constraint derivative w.r.t. input is not finite.\n";
    }
    size_t DmRank = LinearAlgebra::rank(stateInputEqConstrInputDerivative_.topRows(numStateInputEqConstr_));
    if (DmRank != numStateInputEqConstr_) {
      errorDescription << "Input-state constraint derivative w.r.t. input is not full-row rank. It's rank is " + std::to_string(DmRank) +
                              " while the expected rank is " + std::to_string(numStateInputEqConstr_) + ".\n";
    }
  }

  if (numStateEqConstr_ > 0) {
    if (!stateEqConstr_.head(numStateEqConstr_).allFinite()) {
      errorDescription << "State-only constraint is not finite.\n";
    }
    if (!stateEqConstrStateDerivative_.topRows(numStateEqConstr_).allFinite()) {
      errorDescription << "State-only constraint derivative w.r.t. state is not finite.\n";
    }
  }

  for (size_t i = 0; i < numIneqConstr_; i++) {
    if (ineqConstr_[i] != ineqConstr_[i]) {
      errorDescription << "Inequality constraint " + std::to_string(i) + " is not finite.\n";
    }
    if (!ineqConstrStateDerivative_[i].allFinite()) {
      errorDescription << "Inequality constraint " + std::to_string(i) + " derivative w.r.t. state is not finite.\n";
    }
    if (!ineqConstrInputDerivative_[i].allFinite()) {
      errorDescription << "Inequality constraint " + std::to_string(i) + " derivative w.r.t. input is not finite.\n";
    }
    if (!ineqConstrStateSecondDerivative_[i].allFinite()) {
      errorDescription << "Inequality constraint " + std::to_string(i) + " second derivative w.r.t. state is not finite.\n";
    }
    if (!ineqConstrInputSecondDerivative_[i].allFinite()) {
      errorDescription << "Inequality constraint " + std::to_string(i) + " second derivative w.r.t. state is not finite.\n";
    }
    if (!ineqConstrInputStateDerivative_[i].allFinite()) {
      errorDescription << "Inequality constraint " + std::to_string(i) + " second derivative w.r.t. input-state is not finite.\n";
    }
  }

  return errorDescription.str();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::ostream& operator<<(std::ostream& out, const ModelDataBase& data) {
  out << '\n';
  out << "time: " << data.time_ << '\n';
  out << "Dynamics: " << data.dynamics_.transpose() << '\n';
  out << "dynamicsBias: " << data.dynamicsBias_.transpose() << '\n';
  out << "Dynamics State Derivative:\n" << data.dynamicsStateDerivative_ << '\n';
  out << "Dynamics Input Derivative:\n" << data.dynamicsInputDerivative_ << '\n';
  out << "Dynamics Covariance:\n" << data.dynamicsCovariance_ << '\n';

  out << "Cost: " << data.cost_ << '\n';
  out << "Cost State Derivative: " << data.costStateDerivative_.transpose() << '\n';
  out << "Cost Input Derivative: " << data.costInputDerivative_.transpose() << '\n';
  out << "Cost State Second Derivative:\n" << data.costStateSecondDerivative_ << '\n';
  out << "Cost Input Second Derivative:\n" << data.costInputSecondDerivative_ << '\n';
  out << "Cost Input State Derivative:\n" << data.costInputStateDerivative_ << '\n';
  out << '\n';
}

}  // namespace ocs2
