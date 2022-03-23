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

#pragma once

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <vector>

#include <ocs2_core/Types.h>
#include <ocs2_core/integration/OdeBase.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_core/model_data/ModelData.h>

#include "ocs2_ddp/riccati_equations/RiccatiModification.h"

namespace ocs2 {

/**
 * Data cache for continuous-time Riccati equation
 */
struct ContinuousTimeRiccatiData {
  scalar_t s_ = 0.0;
  vector_t Sv_;
  matrix_t Sm_;

  scalar_t ds_ = 0.0;
  vector_t dSv_;
  matrix_t dSm_;

  vector_t projectedHv_;
  matrix_t projectedAm_;
  matrix_t projectedBm_;
  matrix_t projectedRm_;
  matrix_t dynamicsCovariance_;

  matrix_t deltaQm_;

  matrix_t projectedGm_;
  vector_t projectedGv_;

  matrix_t projectedKm_;
  vector_t projectedLv_;

  matrix_t SmTrans_projectedAm_;
  matrix_t projectedKm_T_projectedGm_;
  matrix_t projectedRm_projectedKm_;
  vector_t projectedRm_projectedLv_;

  // risk sensitive data
  vector_t Sigma_Sv_;
  matrix_t Sigma_Sm_;
};

/**
 * Helper function to define the s vector dimension, also supports dynamic size -1.
 *
 * @param [in] state_dim: Dimension of the state space.
 * @return Dimension of the flattened and concatenated vector from Sm, Sv and s.
 */
static constexpr int s_vector_dim(int state_dim) {
  /** If STATE_DIM=n, Then: n(n+1)/2 entries from triangular matrix Sm, n entries from vector Sv and +1 one from a scalar */
  return state_dim == Eigen::Dynamic ? Eigen::Dynamic : (state_dim * (state_dim + 1) / 2 + state_dim + 1);
}

/**
 * Helper function to define the Riccati matrix dimension, also supports dynamic size -1.
 *
 * @param [in] state_dim: Dimension of the state space.
 * @return Dimension of the flattened and concatenated vector from Sm, Sv and s.
 */
static constexpr int riccati_matrix_dim(int flattened_dim) {
  /** If STATE_DIM=n, Then: n(n+1)/2 entries from triangular matrix Sm, n entries from vector Sv and +1 one from a scalar */
  return flattened_dim == Eigen::Dynamic ? Eigen::Dynamic : (static_cast<int>(std::sqrt(8 * flattened_dim + 1)) - 3) / 2;
}

/**
 * This class implements the Riccati differential equations for SLQ problem.
 */
class ContinuousTimeRiccatiEquations final : public OdeBase {
 public:
  /**
   * Constructor.
   *
   * @param [in] reducedFormRiccati: The reduced form of the Riccati equation is yield by assuming that Hessein of
   * the Hamiltonian is positive definite. In this case, the computation of Riccati equation is more efficient.
   * @param [in] isRiskSensitive: Neither the risk sensitive variant is used or not.
   */
  explicit ContinuousTimeRiccatiEquations(bool reducedFormRiccati, bool isRiskSensitive = false);

  /**
   * Default destructor.
   */
  ~ContinuousTimeRiccatiEquations() override = default;

  /**
   * Sets risk-sensitive coefficient.
   */
  void setRiskSensitiveCoefficient(scalar_t riskSensitiveCoeff);

  /**
   * Transcribe symmetric matrix Sm, vector Sv and scalar s into a single vector.
   *
   * @param [in] Sm: \f$ S_m \f$
   * @param [in] Sv: \f$ S_v \f$
   * @param [in] s: \f$ s \f$
   * @return Single vector constructed by concatenating Sm, Sv and s.
   */
  static vector_t convert2Vector(const matrix_t& Sm, const vector_t& Sv, const scalar_t& s);

  /**
   * Transcribe value function approximation into a single vector.
   *
   * @param [in] valueFunction: value function approximation
   * @return Single vector constructed by concatenating Sm, Sv and s.
   */
  static vector_t convert2Vector(const ScalarFunctionQuadraticApproximation& valueFunction) {
    return ContinuousTimeRiccatiEquations::convert2Vector(valueFunction.dfdxx, valueFunction.dfdx, valueFunction.f);
  }

  /**
   * Transcribes the stacked vector allSs into a symmetric matrix, Sm, a vector, Sv and a single scalar, s.
   *
   * @param [in] allSs: Single vector constructed by concatenating Sm, Sv and s.
   * @param [out] Sm: \f$ S_m \f$
   * @param [out] Sv: \f$ S_v \f$
   * @param [out] s: \f$ s \f$
   */
  static void convert2Matrix(const vector_t& allSs, matrix_t& Sm, vector_t& Sv, scalar_t& s);

  /**
   * Transcribes the stacked vector allSs into value function approximation.
   *
   * @param [in] allSs: Single vector constructed by concatenating Sm, Sv and s.
   * @param [out] valueFunction: value function approximation
   */
  static void convert2Matrix(const vector_t& allSs, ScalarFunctionQuadraticApproximation& valueFunction) {
    ContinuousTimeRiccatiEquations::convert2Matrix(allSs, valueFunction.dfdxx, valueFunction.dfdx, valueFunction.f);
  }

  /**
   * Sets coefficients of the model.
   *
   * @param [in] timeStampPtr: A pointer to the time stamp trajectory.
   * @param [in] projectedModelDataPtr: A pointer to the projected model data trajectory.
   * @param [in] eventsPastTheEndIndecesPtr: A pointer to the post event indices.
   * @param [in] modelDataEventTimesPtr: A pointer to the model data at event times.
   * @param [in] riccatiModificationPtr: A pointer to the RiccatiModification trajectory.
   */
  void setData(const scalar_array_t* timeStampPtr, const std::vector<ModelData>* projectedModelDataPtr,
               const size_array_t* eventsPastTheEndIndecesPtr, const std::vector<ModelData>* modelDataEventTimesPtr,
               const std::vector<riccati_modification::Data>* riccatiModificationPtr);

  /**
   * Riccati jump map at switching moments
   *
   * @param [in] z: Normalized transition time
   * @param [in] allSs: A flattened vector constructed by concatenating Sm, Sv and s.
   * @return mapped flattened state after transition.
   */
  vector_t computeJumpMap(scalar_t z, const vector_t& allSs) override;

  /**
   * Computes derivatives.
   *
   * @param [in] z: Normalized time.
   * @param [in] allSs: A flattened vector constructed by concatenating Sm, Sv and s.
   * @return d(allSs)/dz.
   */
  vector_t computeFlowMap(scalar_t z, const vector_t& allSs) override;

 private:
  /**
   * Computes the Riccati equations for SLQ problem.
   *
   * @param [in] indexAlpha: The index and interpolation coefficient (alpha) pair.
   * @param [in] Sm: The current Riccati matrix.
   * @param [in] Sv: The current Riccati vector.
   * @param [in] s: The current Riccati scalar.
   * @param [out] creCache: The continuous-time Riccati equation cache date.
   * @param [out] dSm: The time derivative of the Riccati matrix.
   * @param [out] dSv: The time derivative of the  Riccati vector.
   * @param [out] ds: The time derivative of the  Riccati scalar.
   */
  void computeFlowMapSLQ(std::pair<int, scalar_t> indexAlpha, const matrix_t& Sm, const vector_t& Sv, const scalar_t& s,
                         ContinuousTimeRiccatiData& creCache, matrix_t& dSm, vector_t& dSv, scalar_t& ds) const;

  /**
   * Computes the Riccati equations for ILEG problem.
   *
   * @param [in] indexAlpha: The index and interpolation coefficient (alpha) pair.
   * @param [in] Sm: The current Riccati matrix.
   * @param [in] Sv: The current Riccati vector.
   * @param [in] s: The current Riccati scalar.
   * @param [out] creCache: The continuous-time Riccati equation cache date.
   * @param [out] dSm: The time derivative of the Riccati matrix.
   * @param [out] dSv: The time derivative of the  Riccati vector.
   * @param [out] ds: The time derivative of the  Riccati scalar.
   */
  void computeFlowMapILEG(std::pair<int, scalar_t> indexAlpha, const matrix_t& Sm, const vector_t& Sv, const scalar_t& s,
                          ContinuousTimeRiccatiData& creCache, matrix_t& dSm, vector_t& dSv, scalar_t& ds) const;

 private:
  bool reducedFormRiccati_;
  bool isRiskSensitive_;
  scalar_t riskSensitiveCoeff_ = 0.0;

  // array pointers
  const scalar_array_t* timeStampPtr_ = nullptr;
  const std::vector<ModelData>* projectedModelDataPtr_ = nullptr;
  const std::vector<ModelData>* modelDataEventTimesPtr_ = nullptr;
  const std::vector<riccati_modification::Data>* riccatiModificationPtr_ = nullptr;
  scalar_array_t eventTimes_;

  ContinuousTimeRiccatiData continuousTimeRiccatiData_;
};

}  // namespace ocs2
