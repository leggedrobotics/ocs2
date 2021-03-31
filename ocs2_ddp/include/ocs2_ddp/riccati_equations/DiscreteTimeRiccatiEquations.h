/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

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

#include <ocs2_core/Types.h>
#include <ocs2_core/model_data/ModelData.h>

#include "ocs2_ddp/riccati_equations/RiccatiModification.h"

namespace ocs2 {

/**
 * Data cache for discrete-time Riccati equation
 */
struct DiscreteTimeRiccatiData {
  vector_t Sm_projectedHv_;
  matrix_t Sm_projectedAm_;
  matrix_t Sm_projectedBm_;
  vector_t Sv_plus_Sm_projectedHv_;

  matrix_t projectedHm_;
  matrix_t projectedGm_;
  vector_t projectedGv_;

  matrix_t projectedKm_T_projectedGm_;
  matrix_t projectedHm_projectedKm_;
  vector_t projectedHm_projectedLv_;

  // risk sensitive data
  vector_t Sigma_Sv_;
  matrix_t I_minus_Sm_Sigma_;
  matrix_t inv_I_minus_Sm_Sigma_;
  scalar_t sNextStochastic_ = 0.0;
  vector_t SvNextStochastic_;
  matrix_t SmNextStochastic_;
};

/**
 * This class implements the Riccati difference equations for iLQR problem.
 */
class DiscreteTimeRiccatiEquations {
 public:
  /**
   * Constructor.
   *
   * @param [in] reducedFormRiccati: The reduced form of the Riccati equation is yield by assuming that Hessein of
   * the Hamiltonian is positive definite. In this case, the computation of Riccati equation is more efficient.
   * @param [in] isRiskSensitive: Neither the risk sensitive variant is used or not.
   */
  explicit DiscreteTimeRiccatiEquations(bool reducedFormRiccati, bool isRiskSensitive = false);

  /**
   * Default destructor.
   */
  ~DiscreteTimeRiccatiEquations() = default;

  /**
   * Sets risk-sensitive coefficient.
   */
  void setRiskSensitiveCoefficient(scalar_t riskSensitiveCoeff);

  /**
   * Computes one step Riccati difference equations.
   *
   * @param [in] projectedModelData: The projected model data.
   * @param [in] riccatiModification: The RiccatiModification.
   * @param [in] SmNext: The Riccati matrix of the next time step.
   * @param [in] SvNext: The Riccati vector of the next time step.
   * @param [in] sNext: The Riccati scalar of the next time step.
   * @param [out] projectedKm: The projected feedback controller.
   * @param [out] projectedLv: The projected feedforward controller.
   * @param [out] Sm: The current Riccati matrix.
   * @param [out] Sv: The current Riccati vector.
   * @param [out] s: The current Riccati scalar.
   */
  void computeMap(const ModelData& projectedModelData, const riccati_modification::Data& riccatiModification, const matrix_t& SmNext,
                  const vector_t& SvNext, const scalar_t& sNext, matrix_t& projectedKm, vector_t& projectedLv, matrix_t& Sm, vector_t& Sv,
                  scalar_t& s);

 private:
  /**
   * Computes one step Riccati difference equations for ILQR formulation.
   *
   * @param [in] projectedModelData: The projected model data.
   * @param [in] riccatiModification: The RiccatiModification.
   * @param [in] SmNext: The Riccati matrix of the next time step.
   * @param [in] SvNext: The Riccati vector of the next time step.
   * @param [in] sNext: The Riccati scalar of the next time step.
   * @param [out] dreCache: The discrete-time Riccati equation cache date.
   * @param [out] projectedKm: The projected feedback controller.
   * @param [out] projectedLv: The projected feedforward controller.
   * @param [out] Sm: The current Riccati matrix.
   * @param [out] Sv: The current Riccati vector.
   * @param [out] s: The current Riccati scalar.
   */
  void computeMapILQR(const ModelData& projectedModelData, const riccati_modification::Data& riccatiModification, const matrix_t& SmNext,
                      const vector_t& SvNext, const scalar_t& sNext, DiscreteTimeRiccatiData& dreCache, matrix_t& projectedKm,
                      vector_t& projectedLv, matrix_t& Sm, vector_t& Sv, scalar_t& s) const;

  /**
   * Computes one step Riccati difference equations for ILEG formulation.
   *
   * @param [in] projectedModelData: The projected model data.
   * @param [in] riccatiModification: The RiccatiModification.
   * @param [in] SmNext: The Riccati matrix of the next time step.
   * @param [in] SvNext: The Riccati vector of the next time step.
   * @param [in] sNext: The Riccati scalar of the next time step.
   * @param [out] dreCache: The discrete-time Riccati equation cache date.
   * @param [out] projectedKm: The projected feedback controller.
   * @param [out] projectedLv: The projected feedforward controller.
   * @param [out] Sm: The current Riccati matrix.
   * @param [out] Sv: The current Riccati vector.
   * @param [out] s: The current Riccati scalar.
   */
  void computeMapILEG(const ModelData& projectedModelData, const riccati_modification::Data& riccatiModification, const matrix_t& SmNext,
                      const vector_t& SvNext, const scalar_t& sNext, DiscreteTimeRiccatiData& dreCache, matrix_t& projectedKm,
                      vector_t& projectedLv, matrix_t& Sm, vector_t& Sv, scalar_t& s) const;

 private:
  bool reducedFormRiccati_;
  bool isRiskSensitive_;
  scalar_t riskSensitiveCoeff_ = 0.0;

  DiscreteTimeRiccatiData discreteTimeRiccatiData_;
};

}  // namespace ocs2
