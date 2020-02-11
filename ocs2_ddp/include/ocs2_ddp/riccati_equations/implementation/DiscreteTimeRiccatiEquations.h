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

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int STATE_DIM, int INPUT_DIM>
DiscreteTimeRiccatiEquations<STATE_DIM, INPUT_DIM>::DiscreteTimeRiccatiEquations(bool reducedFormRiccati, bool isRiskSensitive)
    : reducedFormRiccati_(reducedFormRiccati), isRiskSensitive_(isRiskSensitive) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int STATE_DIM, int INPUT_DIM>
void DiscreteTimeRiccatiEquations<STATE_DIM, INPUT_DIM>::setRiskSensitiveCoefficient(scalar_t riskSensitiveCoeff) {
  riskSensitiveCoeff_ = riskSensitiveCoeff;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int STATE_DIM, int INPUT_DIM>
void DiscreteTimeRiccatiEquations<STATE_DIM, INPUT_DIM>::computeMap(const ModelDataBase projectedModelData,
                                                                    const RiccatiModificationBase& riccatiModification,
                                                                    const state_matrix_t& SmNext, const state_vector_t& SvNext,
                                                                    const scalar_t& sNext, dynamic_matrix_t& projectedKm,
                                                                    dynamic_vector_t& projectedLv, state_matrix_t& Sm, state_vector_t& Sv,
                                                                    scalar_t& s) {
  if (isRiskSensitive_) {
    computeMapILEG(projectedModelData, riccatiModification, SmNext, SvNext, sNext, projectedKm, projectedLv, Sm, Sv, s);
  } else {
    computeMapILQR(projectedModelData, riccatiModification, SmNext, SvNext, sNext, projectedKm, projectedLv, Sm, Sv, s);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int STATE_DIM, int INPUT_DIM>
void DiscreteTimeRiccatiEquations<STATE_DIM, INPUT_DIM>::computeMapILQR(const ModelDataBase projectedModelData,
                                                                        const RiccatiModificationBase& riccatiModification,
                                                                        const state_matrix_t& SmNext, const state_vector_t& SvNext,
                                                                        const scalar_t& sNext, dynamic_matrix_t& projectedKm,
                                                                        dynamic_vector_t& projectedLv, state_matrix_t& Sm,
                                                                        state_vector_t& Sv, scalar_t& s) {
  // precomputation (1)
  Sm_projectedHv_.noalias() = SmNext * projectedModelData.dynamicsBias_;
  Sm_projectedAm_.noalias() = SmNext * projectedModelData.dynamicsStateDerivative_;
  Sm_projectedBm_.noalias() = SmNext * projectedModelData.dynamicsInputDerivative_;
  Sv_plus_Sm_projectedHv_ = SvNext + Sm_projectedHv_;

  // projectedGm = projectedPm + projectedBm^T * Sm * projectedAm
  projectedGm_ = projectedModelData.costInputStateDerivative_;
  projectedGm_.noalias() += projectedModelData.dynamicsInputDerivative_.transpose() * Sm_projectedAm_;

  // projectedGv = projectedRv + projectedBm^T * (Sv + Sm * projectedHv)
  projectedGv_ = projectedModelData.costInputDerivative_;
  projectedGv_.noalias() += projectedModelData.dynamicsInputDerivative_.transpose() * Sv_plus_Sm_projectedHv_;

  // projected feedback
  projectedKm = -projectedGm_ - riccatiModification.deltaGm_;
  // projected feedforward
  projectedLv = -projectedGv_ - riccatiModification.deltaGv_;

  // precomputation (2)
  projectedKm_T_projectedGm_.noalias() = projectedKm.transpose() * projectedGm_;
  if (!reducedFormRiccati_) {
    // projectedHm
    projectedHm_ = projectedModelData.costInputSecondDerivative_;
    projectedHm_.noalias() += Sm_projectedBm_.transpose() * projectedModelData.dynamicsInputDerivative_;

    projectedHm_projectedKm_.noalias() = projectedHm_ * projectedKm;
    projectedHm_projectedLv_.noalias() = projectedHm_ * projectedLv;
  }

  /*
   * Sm
   */
  // = Qm + deltaQm
  Sm = projectedModelData.costStateSecondDerivative_ + riccatiModification.deltaQm_;
  // += Am^T * Sm * Am
  Sm.noalias() += Sm_projectedAm_.transpose() * projectedModelData.dynamicsStateDerivative_;
  if (reducedFormRiccati_) {
    // += Km^T * Gm + Gm^T * Km
    Sm += projectedKm_T_projectedGm_;
  } else {
    // += Km^T * Gm + Gm^T * Km
    Sm += projectedKm_T_projectedGm_ + projectedKm_T_projectedGm_.transpose();
    // += Km^T * Hm * Km
    Sm.noalias() += projectedKm.transpose() * projectedHm_projectedKm_;
  }

  /*
   * Sv
   */
  // = Qv
  Sv = projectedModelData.costStateDerivative_;
  // += Am^T * (Sv + Sm * Hv)
  Sv.noalias() += projectedModelData.dynamicsStateDerivative_.transpose() * Sv_plus_Sm_projectedHv_;
  if (reducedFormRiccati_) {
    // += Gm^T * Lv
    Sv.noalias() += projectedGm_.transpose() * projectedLv;
  } else {
    // += Gm^T * Lv
    Sv.noalias() += projectedGm_.transpose() * projectedLv;
    // += Km^T * Gv
    Sv.noalias() += projectedKm.transpose() * projectedGv_;
    // Km^T * Hm * Lv
    Sv.noalias() += projectedHm_projectedKm_.transpose() * projectedLv;
  }

  /*
   * s
   */
  // = s + q
  s = sNext + projectedModelData.cost_;
  // += Hv^T * (Sv + Sm * Hv)
  s += projectedModelData.dynamicsBias_.dot(Sv_plus_Sm_projectedHv_);
  // -= 0.5 Hv^T * Sm * Hv
  s -= 0.5 * projectedModelData.dynamicsBias_.dot(Sm_projectedHv_);
  if (reducedFormRiccati_) {
    // += 0.5 Lv^T Gv
    s += 0.5 * projectedLv.dot(projectedGv_);
  } else {
    // += Lv^T Gv
    s += projectedLv.dot(projectedGv_);
    // += 0.5 Lv^T Hm Lv
    s += 0.5 * projectedLv.dot(projectedHm_projectedLv_);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int STATE_DIM, int INPUT_DIM>
void DiscreteTimeRiccatiEquations<STATE_DIM, INPUT_DIM>::computeMapILEG(const ModelDataBase projectedModelData,
                                                                        const RiccatiModificationBase& riccatiModification,
                                                                        const state_matrix_t& SmNext, const state_vector_t& SvNext,
                                                                        const scalar_t& sNext, dynamic_matrix_t& projectedKm,
                                                                        dynamic_vector_t& projectedLv, state_matrix_t& Sm,
                                                                        state_vector_t& Sv, scalar_t& s) {
  Sigma_Sv_.noalias() = projectedModelData.dynamicsCovariance_ * SvNext;
  I_minus_Sm_Sigma_.setIdentity(projectedModelData.stateDim_, projectedModelData.stateDim_);
  I_minus_Sm_Sigma_.noalias() -= SmNext * projectedModelData.dynamicsCovariance_;

  Eigen::LDLT<state_matrix_t> ldltSm(I_minus_Sm_Sigma_);
  scalar_t det_I_minus_Sm_Sigma_ = ldltSm.vectorD().array().log().sum();

  inv_I_minus_Sm_Sigma_.setIdentity(projectedModelData.stateDim_, projectedModelData.stateDim_);
  ldltSm.solveInPlace(inv_I_minus_Sm_Sigma_);

  SmNextStochastic_.noalias() = inv_I_minus_Sm_Sigma_ * SmNext;
  SvNextStochastic_.noalias() = inv_I_minus_Sm_Sigma_ * SvNext;
  sNextStochastic_ = sNext + riskSensitiveCoeff_ * Sv.dot(Sigma_Sv_) - 0.5 / riskSensitiveCoeff_ * det_I_minus_Sm_Sigma_;

  computeMapILQR(projectedModelData, riccatiModification, SmNextStochastic_, SvNextStochastic_, sNextStochastic_, projectedKm, projectedLv,
                 Sm, Sv, s);
}

}  // namespace ocs2
