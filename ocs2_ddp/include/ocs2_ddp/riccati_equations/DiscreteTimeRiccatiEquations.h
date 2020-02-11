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

#include <ocs2_core/Dimensions.h>
#include <ocs2_core/model_data/ModelDataBase.h>

#include "ocs2_ddp/riccati_equations/RiccatiModificationBase.h"

namespace ocs2 {

/**
 * This class implements the Riccati difference equations for iLQR problem.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <int STATE_DIM, int INPUT_DIM>
class DiscreteTimeRiccatiEquations {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using DIMENSIONS = Dimensions<STATE_DIM, INPUT_DIM>;
  using scalar_t = typename DIMENSIONS::scalar_t;
  using state_vector_t = typename DIMENSIONS::state_vector_t;
  using state_matrix_t = typename DIMENSIONS::state_matrix_t;
  using dynamic_vector_t = typename DIMENSIONS::dynamic_vector_t;
  using dynamic_matrix_t = typename DIMENSIONS::dynamic_matrix_t;

  /**
   * Constructor.
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
  void computeMap(const ModelDataBase projectedModelData, const RiccatiModificationBase& riccatiModification, const state_matrix_t& SmNext,
                  const state_vector_t& SvNext, const scalar_t& sNext, dynamic_matrix_t& projectedKm, dynamic_vector_t& projectedLv,
                  state_matrix_t& Sm, state_vector_t& Sv, scalar_t& s);

 protected:
  /**
   * Computes one step Riccati difference equations for ILQR formulation.
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
  void computeMapILQR(const ModelDataBase projectedModelData, const RiccatiModificationBase& riccatiModification,
                      const state_matrix_t& SmNext, const state_vector_t& SvNext, const scalar_t& sNext, dynamic_matrix_t& projectedKm,
                      dynamic_vector_t& projectedLv, state_matrix_t& Sm, state_vector_t& Sv, scalar_t& s);

  /**
   * Computes one step Riccati difference equations for ILEG formulation.
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
  void computeMapILEG(const ModelDataBase projectedModelData, const RiccatiModificationBase& riccatiModification,
                      const state_matrix_t& SmNext, const state_vector_t& SvNext, const scalar_t& sNext, dynamic_matrix_t& projectedKm,
                      dynamic_vector_t& projectedLv, state_matrix_t& Sm, state_vector_t& Sv, scalar_t& s);

 private:
  bool reducedFormRiccati_;
  bool isRiskSensitive_;
  scalar_t riskSensitiveCoeff_ = 0.0;

  dynamic_vector_t Sm_projectedHv_;
  dynamic_matrix_t Sm_projectedAm_;
  dynamic_matrix_t Sm_projectedBm_;
  dynamic_vector_t Sv_plus_Sm_projectedHv_;

  dynamic_matrix_t projectedHm_;
  dynamic_matrix_t projectedGm_;
  dynamic_vector_t projectedGv_;

  dynamic_matrix_t projectedKm_T_projectedGm_;
  dynamic_matrix_t projectedHm_projectedKm_;
  dynamic_vector_t projectedHm_projectedLv_;

  dynamic_vector_t Sigma_Sv_;
  state_matrix_t I_minus_Sm_Sigma_;
  state_matrix_t inv_I_minus_Sm_Sigma_;
  scalar_t sNextStochastic_;
  state_vector_t SvNextStochastic_;
  state_matrix_t SmNextStochastic_;
};

extern template class DiscreteTimeRiccatiEquations<Eigen::Dynamic, Eigen::Dynamic>;

}  // namespace ocs2

#include <ocs2_ddp/riccati_equations/implementation/DiscreteTimeRiccatiEquations.h>
