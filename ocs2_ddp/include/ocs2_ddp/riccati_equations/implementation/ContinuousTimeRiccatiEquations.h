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

#include <ocs2_core/misc/Lookup.h>
#include <ocs2_core/model_data/ModelDataLinearInterpolation.h>

#include "ocs2_ddp/riccati_equations/RiccatiModificationInterpolation.h"

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int STATE_DIM, int INPUT_DIM>
ContinuousTimeRiccatiEquations<STATE_DIM, INPUT_DIM>::ContinuousTimeRiccatiEquations(bool reducedFormRiccati, bool isRiskSensitive)
    : reducedFormRiccati_(reducedFormRiccati), isRiskSensitive_(isRiskSensitive) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int STATE_DIM, int INPUT_DIM>
void ContinuousTimeRiccatiEquations<STATE_DIM, INPUT_DIM>::setRiskSensitiveCoefficient(scalar_t riskSensitiveCoeff) {
  riskSensitiveCoeff_ = riskSensitiveCoeff;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int STATE_DIM, int INPUT_DIM>
void ContinuousTimeRiccatiEquations<STATE_DIM, INPUT_DIM>::convert2Vector(const dynamic_matrix_t& Sm, const dynamic_vector_t& Sv,
                                                                          const scalar_t& s, s_vector_t& allSs) {
  /* Sm is symmetric. Here, we only extract the upper triangular part and
   * transcribe it in column-wise fashion into allSs*/
  size_t count = 0;  // count the total number of scalar entries covered
  size_t nRows = 0;
  const size_t state_dim = Sm.cols();

  assert(Sm.rows() == state_dim);
  assert(Sv.rows() == state_dim);

  // ensure proper size in case of Eigen::Dynamic size.
  allSs.resize(s_vector_dim(state_dim));

  for (size_t col = 0; col < state_dim; col++) {
    nRows = col + 1;
    allSs.segment(count, nRows) << Eigen::Map<const dynamic_vector_t>(Sm.data() + col * state_dim, nRows);
    count += nRows;
  }

  /* add data from Sv on top*/
  allSs.segment(count, state_dim) << Eigen::Map<const dynamic_vector_t>(Sv.data(), state_dim);

  /* add s as last element*/
  allSs.template tail<1>() << s;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int STATE_DIM, int INPUT_DIM>
void ContinuousTimeRiccatiEquations<STATE_DIM, INPUT_DIM>::convert2Matrix(const s_vector_t& allSs, state_matrix_t& Sm, state_vector_t& Sv,
                                                                          scalar_t& s) {
  /* Sm is symmetric. Here, we map the first entries from allSs onto the upper triangular part of the symmetric matrix*/
  int count = 0;
  int nRows = 0;
  const auto state_dim = riccati_matrix_dim(allSs.size());

  assert(s_vector_dim(state_dim) == allSs.size());

  // ensure proper size in case of Eigen::Dynamic size.
  Sv.resize(state_dim);
  Sm.resize(state_dim, state_dim);

  for (int col = 0; col < state_dim; col++) {
    nRows = col + 1;
    Sm.block(0, col, nRows, 1) << Eigen::Map<const dynamic_vector_t>(allSs.data() + count, nRows);
    count += nRows;
  }
  Sm.template triangularView<Eigen::Lower>() = Sm.template triangularView<Eigen::Upper>().transpose();

  /* extract the vector Sv*/
  Sv = Eigen::Map<const dynamic_vector_t>(allSs.data() + count, state_dim);

  /* extract s as the last element */
  s = allSs.template tail<1>()(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int STATE_DIM, int INPUT_DIM>
void ContinuousTimeRiccatiEquations<STATE_DIM, INPUT_DIM>::setData(const scalar_array_t* timeStampPtr,
                                                                   const ModelDataBase::array_t* projectedModelDataPtr,
                                                                   const size_array_t* postEventIndicesPtr,
                                                                   const ModelDataBase::array_t* modelDataEventTimesPtr,
                                                                   const RiccatiModificationBase::array_t* riccatiModificationPtr) {
  BASE::resetNumFunctionCalls();

  // saving array pointers
  timeStampPtr_ = timeStampPtr;
  projectedModelDataPtr_ = projectedModelDataPtr;
  modelDataEventTimesPtr_ = modelDataEventTimesPtr;
  riccatiModificationPtr_ = riccatiModificationPtr;

  eventTimes_.clear();
  eventTimes_.reserve(postEventIndicesPtr->size());
  for (const auto& postEventIndex : *postEventIndicesPtr) {
    eventTimes_.push_back((*timeStampPtr)[postEventIndex - 1]);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int STATE_DIM, int INPUT_DIM>
void ContinuousTimeRiccatiEquations<STATE_DIM, INPUT_DIM>::computeJumpMap(const scalar_t& z, const s_vector_t& allSs,
                                                                          s_vector_t& allSsPreEvent) {
  // epsilon is set to include times past event times which have been artificially increased in the rollout
  scalar_t time = -z;
  size_t index = lookup::findFirstIndexWithinTol(eventTimes_, time, 1e-5);

  // jump model data
  const auto& jumpModelData = (*modelDataEventTimesPtr_)[index];

  //  s_vector_t allSsJump;
  //  convert2Vector(jumpModelData.costStateSecondDerivative_, jumpModelData.costStateDerivative_, jumpModelData.cost_, allSsJump);
  //
  //  allSsPreEvent = allSs + allSsJump;

  // convert to Riccati coefficients
  convert2Matrix(allSs, Sm_, Sv_, s_);

  // TODO: Fix this
  const state_vector_t Hv = state_vector_t::Zero(STATE_DIM);                 // jumpModelData.dynamicsBias_;
  const state_matrix_t Am = state_matrix_t::Identity(STATE_DIM, STATE_DIM);  // jumpModelData.costStateSecondDerivative_;

  // Sm
  state_matrix_t SmPreEvent = jumpModelData.costStateSecondDerivative_;
  SmTrans_projectedAm_.noalias() = Sm_.transpose() * Am;
  SmPreEvent.noalias() += SmTrans_projectedAm_.transpose() * Am;

  // Sv
  dynamic_vector_t Sv_plus_Sm_Hv = Sv_;
  Sv_plus_Sm_Hv.noalias() += Sm_ * Hv;
  state_vector_t SvPreEvent = jumpModelData.costStateDerivative_;
  SvPreEvent.noalias() += Am.transpose() * Sv_plus_Sm_Hv;

  // s
  scalar_t sPreEvent = s_ + jumpModelData.cost_;
  sPreEvent += Hv.dot(Sv_plus_Sm_Hv);

  convert2Vector(SmPreEvent, SvPreEvent, sPreEvent, allSsPreEvent);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int STATE_DIM, int INPUT_DIM>
void ContinuousTimeRiccatiEquations<STATE_DIM, INPUT_DIM>::computeFlowMap(const scalar_t& z, const s_vector_t& allSs,
                                                                          s_vector_t& derivatives) {
  BASE::numFunctionCalls_++;

  // index
  const scalar_t t = -z;  // denormalized time
  const auto indexAlpha = EigenLinearInterpolation<state_matrix_t>::timeSegment(t, timeStampPtr_);

  convert2Matrix(allSs, Sm_, Sv_, s_);
  if (isRiskSensitive_) {
    computeFlowMapILEG(indexAlpha, Sm_, Sv_, s_, dSm_, dSv_, ds_);
  } else {
    computeFlowMapSLQ(indexAlpha, Sm_, Sv_, s_, dSm_, dSv_, ds_);
  }

  convert2Vector(dSm_, dSv_, ds_, derivatives);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int STATE_DIM, int INPUT_DIM>
void ContinuousTimeRiccatiEquations<STATE_DIM, INPUT_DIM>::computeFlowMapSLQ(std::pair<int, scalar_t> indexAlpha, const state_matrix_t& Sm,
                                                                             const state_vector_t& Sv, const scalar_t& s,
                                                                             dynamic_matrix_t& dSm, dynamic_vector_t& dSv, scalar_t& ds) {
  /* note: according to some discussions on stackoverflow, it does not buy
   * computation time if multiplications with symmetric matrices are executed
   * using selfadjointView(). Doing the full multiplication seems to be faster
   * because of vectorization
   */

  // Hv
  ModelData::LinearInterpolation::interpolate(indexAlpha, projectedHv_, projectedModelDataPtr_, ModelData::dynamicsBias);
  // Am
  ModelData::LinearInterpolation::interpolate(indexAlpha, projectedAm_, projectedModelDataPtr_, ModelData::dynamicsStateDerivative);
  // Bm
  ModelData::LinearInterpolation::interpolate(indexAlpha, projectedBm_, projectedModelDataPtr_, ModelData::dynamicsInputDerivative);
  // q
  ModelData::LinearInterpolation::interpolate(indexAlpha, ds, projectedModelDataPtr_, ModelData::cost);
  // Qv
  ModelData::LinearInterpolation::interpolate(indexAlpha, dSv, projectedModelDataPtr_, ModelData::costStateDerivative);
  // Qm
  ModelData::LinearInterpolation::interpolate(indexAlpha, dSm, projectedModelDataPtr_, ModelData::costStateSecondDerivative);
  // Rv
  ModelData::LinearInterpolation::interpolate(indexAlpha, projectedGv_, projectedModelDataPtr_, ModelData::costInputDerivative);
  // Pm
  ModelData::LinearInterpolation::interpolate(indexAlpha, projectedGm_, projectedModelDataPtr_, ModelData::costInputStateDerivative);
  // delatQm
  RiccatiModification::LinearInterpolation::interpolate(indexAlpha, deltaQm_, riccatiModificationPtr_, RiccatiModification::deltaQm);
  // delatGm
  RiccatiModification::LinearInterpolation::interpolate(indexAlpha, projectedKm_, riccatiModificationPtr_, RiccatiModification::deltaGm);
  // delatGv
  RiccatiModification::LinearInterpolation::interpolate(indexAlpha, projectedLv_, riccatiModificationPtr_, RiccatiModification::deltaGv);

  // projectedGm = projectedPm + projectedBm^T * Sm [COMPLEXITY: nx^2 * np]
  projectedGm_.noalias() += projectedBm_.transpose() * Sm;

  // projectedGv = projectedRv + projectedBm^T * Sv [COMPLEXITY: nx * np]
  projectedGv_.noalias() += projectedBm_.transpose() * Sv;

  // projected feedback
  projectedKm_ = -(projectedGm_ + projectedKm_);
  // projected feedforward
  projectedLv_ = -(projectedGv_ + projectedLv_);

  // precomputation
  // [COMPLEXITY: nx^3 + nx^2 * np]
  SmTrans_projectedAm_.noalias() = Sm.transpose() * projectedAm_;
  projectedKm_T_projectedGm_.noalias() = projectedKm_.transpose() * projectedGm_;
  if (!reducedFormRiccati_) {
    // Rm
    ModelData::LinearInterpolation::interpolate(indexAlpha, projectedRm_, projectedModelDataPtr_, ModelData::costInputSecondDerivative);
    // [COMPLEXITY: nx * np^2]
    projectedRm_projectedKm_.noalias() = projectedRm_ * projectedKm_;
    // [COMPLEXITY: np^2]
    projectedRm_projectedLv_.noalias() = projectedRm_ * projectedLv_;
  }

  /*
   * Sm
   *
   * reducedFormRiccati:
   *   [TOTAL COMPLEXITY: (nx^3) + 2(nx^2 * np)]
   * other
   *   [TOTAL COMPLEXITY: (nx^3) + 3(nx^2 * np) + (nx * np^2)]
   */
  // += deltaQm + Sm^T * Am + Am^T * Sm
  dSm += deltaQm_ + SmTrans_projectedAm_ + SmTrans_projectedAm_.transpose();
  if (reducedFormRiccati_) {
    // += Km^T * Gm-
    dSm += projectedKm_T_projectedGm_;
  } else {
    // += Km^T * Gm + Gm^T * Km
    dSm += projectedKm_T_projectedGm_ + projectedKm_T_projectedGm_.transpose();
    // += Km^T * Hm * Km
    dSm.noalias() += projectedKm_.transpose() * projectedRm_projectedKm_;
  }

  /*
   * Sv
   *
   * reducedFormRiccati:
   *   [TOTAL COMPLEXITY: 2*(nx^2) + (nx * np)]
   * other
   *   [TOTAL COMPLEXITY: 2*(nx^2) + 3(nx * np)]
   */
  // += Sm * Hv
  dSv.noalias() += Sm.transpose() * projectedHv_;
  // += Am^T * Sv
  dSv.noalias() += projectedAm_.transpose() * Sv;
  if (reducedFormRiccati_) {
    // += Gm^T * Lv
    dSv.noalias() += projectedGm_.transpose() * projectedLv_;
  } else {
    // += Gm^T * Lv
    dSv.noalias() += projectedGm_.transpose() * projectedLv_;
    // += Km^T * Gv
    dSv.noalias() += projectedKm_.transpose() * projectedGv_;
    // Km^T * Hm * Lv
    dSv.noalias() += projectedRm_projectedKm_.transpose() * projectedLv_;
  }

  /*
   * ds
   * reducedFormRiccati:
   *   [TOTAL COMPLEXITY: nx + np]
   * other
   *   [TOTAL COMPLEXITY: nx + 2np + np^2]
   */
  // += Hv^T * Sv
  ds += projectedHv_.dot(Sv);
  if (reducedFormRiccati_) {
    // += 0.5 Lv^T Gv
    ds += 0.5 * projectedLv_.dot(projectedGv_);
  } else {
    // += Lv^T Gv
    ds += projectedLv_.dot(projectedGv_);
    // += 0.5 Lv^T Hm Lv
    ds += 0.5 * projectedLv_.dot(projectedRm_projectedLv_);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int STATE_DIM, int INPUT_DIM>
void ContinuousTimeRiccatiEquations<STATE_DIM, INPUT_DIM>::computeFlowMapILEG(std::pair<int, scalar_t> indexAlpha, const state_matrix_t& Sm,
                                                                              const state_vector_t& Sv, const scalar_t& s,
                                                                              dynamic_matrix_t& dSm, dynamic_vector_t& dSv, scalar_t& ds) {
  computeFlowMapSLQ(indexAlpha, Sm, Sv, s, dSm, dSv, ds);

  // Sigma
  ModelData::LinearInterpolation::interpolate(indexAlpha, dynamicsCovariance_, projectedModelDataPtr_, ModelData::dynamicsCovariance);

  Sigma_Sv_.noalias() = dynamicsCovariance_ * Sv;
  Sigma_Sm_.noalias() = dynamicsCovariance_ * Sm;

  dSm.noalias() += riskSensitiveCoeff_ * Sm.transpose() * Sigma_Sm_;
  dSv.noalias() += riskSensitiveCoeff_ * Sigma_Sm_.transpose() * Sv;
  ds += 0.5 * Sigma_Sm_.trace() + 0.5 * riskSensitiveCoeff_ * Sv.dot(Sigma_Sv_);
}

}  // namespace ocs2
