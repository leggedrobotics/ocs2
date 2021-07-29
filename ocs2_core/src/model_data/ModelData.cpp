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
#include <ocs2_core/model_data/ModelData.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ModelData::display() const {
  std::cerr << *this;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ModelData::checkSizes(int stateDim, int inputDim) const {
  assert(stateDim_ == stateDim);
  assert(inputDim_ == inputDim);

  // dynamics flow map
  assert(dynamics_.f.size() == stateDim);
  assert(dynamicsBias_.size() == stateDim);
  assert(dynamics_.dfdx.rows() == stateDim);
  assert(dynamics_.dfdx.cols() == stateDim);
  assert(dynamics_.dfdu.rows() == stateDim);
  assert(dynamics_.dfdu.cols() == inputDim);

  // cost
  assert(cost_.dfdx.size() == stateDim);
  assert(cost_.dfdxx.rows() == stateDim);
  assert(cost_.dfdxx.cols() == stateDim);
  assert(cost_.dfdu.size() == inputDim);
  assert(cost_.dfduu.rows() == inputDim);
  assert(cost_.dfduu.cols() == inputDim);
  assert(cost_.dfdux.rows() == inputDim);
  assert(cost_.dfdux.cols() == stateDim);

  // state equality constraints
  assert(stateEqConstr_.dfdx.rows() == stateEqConstr_.f.rows());
  assert(stateEqConstr_.dfdx.cols() == stateDim);

  // state-input equality constraints
  assert(stateInputEqConstr_.dfdx.rows() == stateInputEqConstr_.f.rows());
  assert(stateInputEqConstr_.dfdx.cols() == stateDim);
  assert(stateInputEqConstr_.dfdu.rows() == stateInputEqConstr_.f.rows());
  assert(stateInputEqConstr_.dfdu.cols() == inputDim);

  // inequality constraints
  assert(ineqConstr_.dfdx.rows() == ineqConstr_.f.rows());
  assert(ineqConstr_.dfdx.cols() == stateDim);
  assert(ineqConstr_.dfdu.rows() == ineqConstr_.f.rows());
  assert(ineqConstr_.dfdu.cols() == inputDim);
  assert(ineqConstr_.dfdxx.size() == ineqConstr_.f.rows());
  assert(ineqConstr_.dfduu.size() == ineqConstr_.f.rows());
  assert(ineqConstr_.dfdux.size() == ineqConstr_.f.rows());
  for (int i = 0; i < ineqConstr_.f.rows(); i++) {
    assert(ineqConstr_.dfdxx[i].rows() == stateDim);
    assert(ineqConstr_.dfdxx[i].cols() == stateDim);
    assert(ineqConstr_.dfduu[i].rows() == inputDim);
    assert(ineqConstr_.dfduu[i].cols() == inputDim);
    assert(ineqConstr_.dfdux[i].rows() == inputDim);
    assert(ineqConstr_.dfdux[i].cols() == stateDim);
  }  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::string ModelData::checkCostProperties() const {
  std::stringstream errorDescription;

  if (cost_.f != cost_.f) {
    errorDescription << "Intermediate cost is not finite.\n";
  }
  if (!cost_.dfdx.allFinite()) {
    errorDescription << "Intermediate cost first derivative w.r.t. state is not finite.\n";
  }
  if (!cost_.dfdxx.allFinite()) {
    errorDescription << "Intermediate cost second derivative w.r.t. state is not finite.\n";
  }
  if (!cost_.dfdxx.isApprox(cost_.dfdxx.transpose())) {
    errorDescription << "Intermediate cost second derivative w.r.t. state is not self-adjoint.\n";
  }
  if (LinearAlgebra::symmetricEigenvalues(cost_.dfdxx).minCoeff() < -Eigen::NumTraits<scalar_t>::epsilon()) {
    errorDescription << "Q matrix is not positive semi-definite. It's smallest eigenvalue is " +
                            std::to_string(LinearAlgebra::symmetricEigenvalues(cost_.dfdxx).minCoeff()) + ".\n";
  }
  if (!cost_.dfdu.allFinite()) {
    errorDescription << "Intermediate cost first derivative w.r.t. input is not finite.\n";
  }
  if (!cost_.dfduu.allFinite()) {
    errorDescription << "Intermediate cost second derivative w.r.t. input is not finite.\n";
  }
  if (!cost_.dfduu.isApprox(cost_.dfduu.transpose())) {
    errorDescription << "Intermediate cost second derivative w.r.t. input is not self-adjoint.\n";
  }
  if (!cost_.dfduu.allFinite()) {
    errorDescription << "Intermediate cost second derivative w.r.t. input-state is not finite.\n";
  }
  if (cost_.dfduu.ldlt().rcond() < Eigen::NumTraits<scalar_t>::epsilon()) {
    errorDescription << "R matrix is not invertible. It's reciprocal condition number is " + std::to_string(cost_.dfduu.ldlt().rcond()) +
                            ".\n";
  }
  if (LinearAlgebra::symmetricEigenvalues(cost_.dfduu).minCoeff() < Eigen::NumTraits<scalar_t>::epsilon()) {
    errorDescription << "R matrix is not positive definite. It's smallest eigenvalue is " +
                            std::to_string(LinearAlgebra::symmetricEigenvalues(cost_.dfduu).minCoeff()) + ".\n";
  }

  return errorDescription.str();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::string ModelData::checkDynamicsDerivativsProperties() const {
  std::stringstream errorDescription;

  if (!dynamics_.f.allFinite()) {
    errorDescription << "Dynamics is not finite.";
  }
  if (!dynamicsBias_.allFinite()) {
    errorDescription << "Dynamics bias is not finite.";
  }
  if (!dynamics_.dfdx.allFinite()) {
    errorDescription << "Dynamics derivative w.r.t. state is not finite.";
  }
  if (!dynamics_.dfdu.allFinite()) {
    errorDescription << "Dynamics derivative w.r.t. input is not finite.";
  }

  return errorDescription.str();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::string ModelData::checkConstraintProperties() const {
  std::stringstream errorDescription;

  if (stateInputEqConstr_.f.rows() > 0) {
    if (!stateInputEqConstr_.f.allFinite()) {
      errorDescription << "Input-state constraint is not finite.\n";
    }
    if (!stateInputEqConstr_.dfdx.allFinite()) {
      errorDescription << "Input-state constraint derivative w.r.t. state is not finite.\n";
    }
    if (!stateInputEqConstr_.dfdu.allFinite()) {
      errorDescription << "Input-state constraint derivative w.r.t. input is not finite.\n";
    }
    size_t DmRank = LinearAlgebra::rank(stateInputEqConstr_.dfdu);
    if (DmRank != stateInputEqConstr_.f.rows()) {
      errorDescription << "Input-state constraint derivative w.r.t. input is not full-row rank. It's rank is " + std::to_string(DmRank) +
                              " while the expected rank is " + std::to_string(stateInputEqConstr_.f.rows()) + ".\n";
    }
  }

  if (stateEqConstr_.f.rows() > 0) {
    if (!stateEqConstr_.f.allFinite()) {
      errorDescription << "State-only constraint is not finite.\n";
    }
    if (!stateEqConstr_.dfdx.allFinite()) {
      errorDescription << "State-only constraint derivative w.r.t. state is not finite.\n";
    }
  }

  if (ineqConstr_.f.rows() > 0) {
    if (!ineqConstr_.f.allFinite()) {
      errorDescription << "Inequality constraint is not finite.\n";
    }
    if (!ineqConstr_.dfdx.allFinite()) {
      errorDescription << "Inequality constraint derivative w.r.t. state is not finite.\n";
    }
    if (!ineqConstr_.dfdu.allFinite()) {
      errorDescription << "Inequality constraint derivative w.r.t. input is not finite.\n";
    }
    for (size_t i = 0; i < ineqConstr_.f.rows(); i++) {
      if (!ineqConstr_.dfdxx[i].allFinite()) {
        errorDescription << "Inequality constraint " + std::to_string(i) + " second derivative w.r.t. state is not finite.\n";
      }
      if (!ineqConstr_.dfdxx[i].isApprox(ineqConstr_.dfdxx[i].transpose())) {
        errorDescription << "Inequality constraint " + std::to_string(i) + " second derivative w.r.t. state is not self-adjoint.\n";
      }
      if (LinearAlgebra::symmetricEigenvalues(ineqConstr_.dfdxx[i]).maxCoeff() > Eigen::NumTraits<scalar_t>::epsilon()) {
        errorDescription
            << "Inequality constraint " + std::to_string(i) +
                   " second derivative w.r.t. state is not negative semi-definite. This will lead to a negative-definite penalty Hessian. "
                   "It's largest eigenvalue is " +
                   std::to_string(LinearAlgebra::symmetricEigenvalues(ineqConstr_.dfdxx[i]).maxCoeff()) + ".\n";
        std::cerr << "dfdxx:\n" << ineqConstr_.dfdxx[i] << std::endl;
      }
      if (!ineqConstr_.dfduu[i].allFinite()) {
        errorDescription << "Inequality constraint " + std::to_string(i) + " second derivative w.r.t. input is not finite.\n";
      }
      if (!ineqConstr_.dfduu[i].isApprox(ineqConstr_.dfduu[i].transpose())) {
        errorDescription << "Inequality constraint " + std::to_string(i) + " second derivative w.r.t. input is not self-adjoint.\n";
      }
      if (LinearAlgebra::symmetricEigenvalues(ineqConstr_.dfduu[i]).maxCoeff() > Eigen::NumTraits<scalar_t>::epsilon()) {
        errorDescription
            << "Inequality constraint " + std::to_string(i) +
                   " second derivative w.r.t. input is not negative semi-definite. This will lead to a negative-definite penalty Hessian. "
                   "It's largest eigenvalue is " +
                   std::to_string(LinearAlgebra::symmetricEigenvalues(ineqConstr_.dfduu[i]).maxCoeff()) + ".\n";
      }
      if (!ineqConstr_.dfdux[i].allFinite()) {
        errorDescription << "Inequality constraint " + std::to_string(i) + " second derivative w.r.t. input-state is not finite.\n";
      }
    }
  }

  return errorDescription.str();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::ostream& operator<<(std::ostream& out, const ModelData& data) {
  out << '\n';
  out << "time: " << data.time_ << '\n';
  out << "Dynamics: " << data.dynamics_.f.transpose() << '\n';
  out << "dynamicsBias: " << data.dynamicsBias_.transpose() << '\n';
  out << "Dynamics State Derivative:\n" << data.dynamics_.dfdx << '\n';
  out << "Dynamics Input Derivative:\n" << data.dynamics_.dfdu << '\n';
  out << "Dynamics Covariance:\n" << data.dynamicsCovariance_ << '\n';

  out << "Cost: " << data.cost_.f << '\n';
  out << "Cost State Derivative: " << data.cost_.dfdx.transpose() << '\n';
  out << "Cost Input Derivative: " << data.cost_.dfdu.transpose() << '\n';
  out << "Cost State Second Derivative:\n" << data.cost_.dfdxx << '\n';
  out << "Cost Input Second Derivative:\n" << data.cost_.dfduu << '\n';
  out << "Cost Input State Derivative:\n" << data.cost_.dfdux << '\n';
  out << '\n';
  return out;
}

}  // namespace ocs2
