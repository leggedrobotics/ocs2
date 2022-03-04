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

#include "ocs2_core/misc/LinearAlgebra.h"
#include "ocs2_core/model_data/ModelData.h"

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::string checkSize(const ModelData& data, int stateDim, int inputDim) {
  std::stringstream errorDescription;

  if (data.stateDim != stateDim) {
    errorDescription << "data.stateDim != " << stateDim << "\n";
  }
  if (data.inputDim != inputDim) {
    errorDescription << "data.inputDim != " << inputDim << "\n";
  }

  // dynamics
  if (data.dynamics.f.size() > 0) {
    errorDescription << checkSize(stateDim, stateDim, inputDim, data.dynamics, "dynamics");

    if (data.dynamicsBias.size() != stateDim) {
      errorDescription << "dynamicsBias.size() != " << stateDim << "\n";
    }
  }

  // cost
  errorDescription << checkSize(stateDim, inputDim, data.cost, "cost");

  // state equality constraints
  errorDescription << checkSize(data.stateEqConstraint.f.size(), stateDim, 0, data.stateEqConstraint, "stateEqConstraint");

  // state-input equality constraints
  errorDescription << checkSize(data.stateInputEqConstraint.f.size(), stateDim, inputDim, data.stateInputEqConstraint,
                                "stateInputEqConstraint");

  return errorDescription.str();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::string checkCostProperties(const ModelData& data) {
  std::stringstream errorDescription;

  errorDescription << checkBeingPSD(data.cost, "cost");

  // check if R is invertible and its schur complement is PSD
  if (data.cost.dfduu.size() > 0) {
    const auto rcond = data.cost.dfduu.ldlt().rcond();
    if (rcond < Eigen::NumTraits<scalar_t>::epsilon()) {
      errorDescription << "Cost second derivative w.r.t. input is not invertible. It's reciprocal condition number is " +
                              std::to_string(rcond) + ".\n";
    } else {
      // check schur complement of R being PSD
      errorDescription << schurComplementOfCostHessianIsPsd(data.cost);
    }
  }

  return errorDescription.str();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::string schurComplementOfCostHessianIsPsd(const ScalarFunctionQuadraticApproximation& cost) {
  if (cost.dfdxx.size() > 0 && cost.dfduu.size() > 0) {
    matrix_t UofUUT;
    LinearAlgebra::computeInverseMatrixUUT(cost.dfduu, UofUUT);
    const matrix_t UT_P = UofUUT.transpose() * cost.dfdux;
    matrix_t inputHessianSchurComplement = cost.dfdxx;
    inputHessianSchurComplement.noalias() -= UT_P.transpose() * UT_P;

    // check for being psd
    return checkBeingPSD(inputHessianSchurComplement, "Schur complement of cost second derivative w.r.t. input");

  } else {
    return "Either cost.dfdxx or cost.dfduu are not set!";
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::string checkDynamicsProperties(const ModelData& data) {
  std::stringstream errorDescription;

  if (!data.dynamics.f.allFinite()) {
    errorDescription << "Dynamics is not finite.";
  }
  if (!data.dynamicsBias.allFinite()) {
    errorDescription << "Dynamics bias is not finite.";
  }
  if (!data.dynamics.dfdx.allFinite()) {
    errorDescription << "Dynamics derivative w.r.t. state is not finite.";
  }
  if (!data.dynamics.dfdu.allFinite()) {
    errorDescription << "Dynamics derivative w.r.t. input is not finite.";
  }

  return errorDescription.str();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::string checkControllability(const VectorFunctionLinearApproximation& dynamics) {
  const size_t stateDim = dynamics.dfdu.rows();
  const size_t inputDim = dynamics.dfdu.cols();

  // controllability matrix
  matrix_t ctrlMatrix(stateDim, inputDim * stateDim);
  ctrlMatrix.leftCols(inputDim) = dynamics.dfdu;
  for (size_t i = 1; i < stateDim; i++) {
    ctrlMatrix.middleCols(i * inputDim, inputDim).noalias() = dynamics.dfdx * ctrlMatrix.middleCols((i - 1) * inputDim, inputDim);
  }

  // controllability rank
  const size_t ctrlMatrixRank = LinearAlgebra::rank(ctrlMatrix);

  std::stringstream errorDescription;
  if (ctrlMatrixRank < stateDim) {
    errorDescription << "Uncontrollable system: controllability matrix rank should be " << stateDim << " as opposed to " << ctrlMatrixRank
                     << "\n";
  }

  return errorDescription.str();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::string checkConstraintProperties(const ModelData& data) {
  std::stringstream errorDescription;

  if (data.stateEqConstraint.f.rows() > 0) {
    if (!data.stateEqConstraint.f.allFinite()) {
      errorDescription << "State-only constraint is not finite.\n";
    }
    if (!data.stateEqConstraint.dfdx.allFinite()) {
      errorDescription << "State-only constraint derivative w.r.t. state is not finite.\n";
    }
  }

  if (data.stateInputEqConstraint.f.rows() > 0) {
    const auto inputDim = data.stateInputEqConstraint.dfdu.cols();
    const auto numConstraints = data.stateInputEqConstraint.f.rows();

    if (!data.stateInputEqConstraint.f.allFinite()) {
      errorDescription << "Input-state constraint is not finite.\n";
    }
    if (!data.stateInputEqConstraint.dfdx.allFinite()) {
      errorDescription << "Input-state constraint derivative w.r.t. state is not finite.\n";
    }
    if (!data.stateInputEqConstraint.dfdu.allFinite()) {
      errorDescription << "Input-state constraint derivative w.r.t. input is not finite.\n";
    }
    if (numConstraints > inputDim) {
      errorDescription << "Number of active state-input equality constraints (a.k.a. " + std::to_string(numConstraints) +
                              ") should be less-equal to the input dimension (a.k.a. " + std::to_string(inputDim) + ").\n";
    }
    const size_t DmRank = LinearAlgebra::rank(data.stateInputEqConstraint.dfdu);
    if (DmRank != numConstraints) {
      errorDescription << "Input-state constraint derivative w.r.t. input is not full-row rank. It's rank is " + std::to_string(DmRank) +
                              " while the expected rank is " + std::to_string(numConstraints) + ".\n";
    }
  }

  return errorDescription.str();
}

}  // namespace ocs2
