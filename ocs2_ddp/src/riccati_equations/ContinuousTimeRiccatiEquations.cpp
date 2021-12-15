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

#include <ocs2_ddp/riccati_equations/ContinuousTimeRiccatiEquations.h>
#include <ocs2_ddp/riccati_equations/RiccatiModificationInterpolation.h>
#include <ocs2_ddp/riccati_equations/RiccatiTransversalityConditions.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ContinuousTimeRiccatiEquations::ContinuousTimeRiccatiEquations(bool reducedFormRiccati, bool isRiskSensitive)
    : reducedFormRiccati_(reducedFormRiccati), isRiskSensitive_(isRiskSensitive) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ContinuousTimeRiccatiEquations::setRiskSensitiveCoefficient(scalar_t riskSensitiveCoeff) {
  riskSensitiveCoeff_ = riskSensitiveCoeff;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t ContinuousTimeRiccatiEquations::convert2Vector(const matrix_t& Sm, const vector_t& Sv, const scalar_t& s) {
  /* Sm is symmetric. Here, we only extract the upper triangular part and
   * transcribe it in column-wise fashion into allSs*/
  size_t count = 0;  // count the total number of scalar entries covered
  size_t nRows = 0;

  const auto state_dim = Sm.cols();
  assert(state_dim > 0);
  assert(Sm.rows() == state_dim);
  assert(Sv.rows() == state_dim);

  vector_t allSs(s_vector_dim(state_dim));

  for (size_t col = 0; col < state_dim; col++) {
    nRows = col + 1;
    allSs.segment(count, nRows) << Eigen::Map<const vector_t>(Sm.data() + col * state_dim, nRows);
    count += nRows;
  }

  /* add data from Sv on top*/
  allSs.segment(count, state_dim) << Eigen::Map<const vector_t>(Sv.data(), state_dim);

  /* add s as last element*/
  allSs.template tail<1>() << s;

  return allSs;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ContinuousTimeRiccatiEquations::convert2Matrix(const vector_t& allSs, matrix_t& Sm, vector_t& Sv, scalar_t& s) {
  /* Sm is symmetric. Here, we map the first entries from allSs onto the upper triangular part of the symmetric matrix*/
  int count = 0;
  int nRows = 0;

  const auto state_dim = riccati_matrix_dim(allSs.size());
  assert(state_dim > 0);

  Sm.resize(state_dim, state_dim);

  for (int col = 0; col < state_dim; col++) {
    nRows = col + 1;
    Sm.block(0, col, nRows, 1) << Eigen::Map<const vector_t>(allSs.data() + count, nRows);
    count += nRows;
  }
  Sm.template triangularView<Eigen::Lower>() = Sm.template triangularView<Eigen::Upper>().transpose();

  /* extract the vector Sv*/
  Sv = Eigen::Map<const vector_t>(allSs.data() + count, state_dim);

  /* extract s as the last element */
  s = allSs.template tail<1>()(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ContinuousTimeRiccatiEquations::setData(const scalar_array_t* timeStampPtr, const std::vector<ModelData>* projectedModelDataPtr,
                                             const size_array_t* eventsPastTheEndIndecesPtr,
                                             const std::vector<ModelData>* modelDataEventTimesPtr,
                                             const std::vector<riccati_modification::Data>* riccatiModificationPtr) {
  OdeBase::resetNumFunctionCalls();

  // saving array pointers
  timeStampPtr_ = timeStampPtr;
  projectedModelDataPtr_ = projectedModelDataPtr;
  modelDataEventTimesPtr_ = modelDataEventTimesPtr;
  riccatiModificationPtr_ = riccatiModificationPtr;

  eventTimes_.clear();
  eventTimes_.reserve(eventsPastTheEndIndecesPtr->size());
  for (const auto& postEventIndex : *eventsPastTheEndIndecesPtr) {
    eventTimes_.push_back((*timeStampPtr)[postEventIndex - 1]);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t ContinuousTimeRiccatiEquations::computeJumpMap(scalar_t z, const vector_t& allSs) {
  // convert to Riccati coefficients
  convert2Matrix(allSs, continuousTimeRiccatiData_.Sm_, continuousTimeRiccatiData_.Sv_, continuousTimeRiccatiData_.s_);

  // epsilon is set to include times past event times which have been artificially increased in the rollout
  const auto time = -z;
  const auto index = lookup::findFirstIndexWithinTol(eventTimes_, time, 1e-5);

  const auto SsPreEvent = riccatiTransversalityConditions((*modelDataEventTimesPtr_)[index], continuousTimeRiccatiData_.Sm_,
                                                          continuousTimeRiccatiData_.Sv_, continuousTimeRiccatiData_.s_);

  return convert2Vector(std::get<0>(SsPreEvent), std::get<1>(SsPreEvent), std::get<2>(SsPreEvent));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t ContinuousTimeRiccatiEquations::computeFlowMap(scalar_t z, const vector_t& allSs) {
  // index
  const scalar_t t = -z;  // denormalized time
  const auto indexAlpha = LinearInterpolation::timeSegment(t, *timeStampPtr_);

  convert2Matrix(allSs, continuousTimeRiccatiData_.Sm_, continuousTimeRiccatiData_.Sv_, continuousTimeRiccatiData_.s_);
  if (isRiskSensitive_) {
    computeFlowMapILEG(indexAlpha, continuousTimeRiccatiData_.Sm_, continuousTimeRiccatiData_.Sv_, continuousTimeRiccatiData_.s_,
                       continuousTimeRiccatiData_, continuousTimeRiccatiData_.dSm_, continuousTimeRiccatiData_.dSv_,
                       continuousTimeRiccatiData_.ds_);
  } else {
    computeFlowMapSLQ(indexAlpha, continuousTimeRiccatiData_.Sm_, continuousTimeRiccatiData_.Sv_, continuousTimeRiccatiData_.s_,
                      continuousTimeRiccatiData_, continuousTimeRiccatiData_.dSm_, continuousTimeRiccatiData_.dSv_,
                      continuousTimeRiccatiData_.ds_);
  }

  return convert2Vector(continuousTimeRiccatiData_.dSm_, continuousTimeRiccatiData_.dSv_, continuousTimeRiccatiData_.ds_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ContinuousTimeRiccatiEquations::computeFlowMapSLQ(std::pair<int, scalar_t> indexAlpha, const matrix_t& Sm, const vector_t& Sv,
                                                       const scalar_t& s, ContinuousTimeRiccatiData& creCache, matrix_t& dSm, vector_t& dSv,
                                                       scalar_t& ds) const {
  /* note: according to some discussions on stackoverflow, it does not buy
   * computation time if multiplications with symmetric matrices are executed
   * using selfadjointView(). Doing the full multiplication seems to be faster
   * because of vectorization
   */

  // Hv
  creCache.projectedHv_ = LinearInterpolation::interpolate(indexAlpha, *projectedModelDataPtr_, model_data::dynamicsBias);
  // Am
  creCache.projectedAm_ = LinearInterpolation::interpolate(indexAlpha, *projectedModelDataPtr_, model_data::dynamics_dfdx);
  // Bm
  creCache.projectedBm_ = LinearInterpolation::interpolate(indexAlpha, *projectedModelDataPtr_, model_data::dynamics_dfdu);
  // q
  ds = LinearInterpolation::interpolate(indexAlpha, *projectedModelDataPtr_, model_data::cost_f);
  // Qv
  dSv = LinearInterpolation::interpolate(indexAlpha, *projectedModelDataPtr_, model_data::cost_dfdx);
  // Qm
  dSm = LinearInterpolation::interpolate(indexAlpha, *projectedModelDataPtr_, model_data::cost_dfdxx);
  // Rv
  creCache.projectedGv_ = LinearInterpolation::interpolate(indexAlpha, *projectedModelDataPtr_, model_data::cost_dfdu);
  // Pm
  creCache.projectedGm_ = LinearInterpolation::interpolate(indexAlpha, *projectedModelDataPtr_, model_data::cost_dfdux);
  // delatQm
  creCache.deltaQm_ = LinearInterpolation::interpolate(indexAlpha, *riccatiModificationPtr_, riccati_modification::deltaQm);
  // delatGm
  creCache.projectedKm_ = LinearInterpolation::interpolate(indexAlpha, *riccatiModificationPtr_, riccati_modification::deltaGm);
  // delatGv
  creCache.projectedLv_ = LinearInterpolation::interpolate(indexAlpha, *riccatiModificationPtr_, riccati_modification::deltaGv);

  // projectedGm = projectedPm + projectedBm^T * Sm [COMPLEXITY: nx^2 * np]
  creCache.projectedGm_.noalias() += creCache.projectedBm_.transpose() * Sm;

  // projectedGv = projectedRv + projectedBm^T * Sv [COMPLEXITY: nx * np]
  creCache.projectedGv_.noalias() += creCache.projectedBm_.transpose() * Sv;

  // projected feedback
  creCache.projectedKm_ = -(creCache.projectedGm_ + creCache.projectedKm_);
  // projected feedforward
  creCache.projectedLv_ = -(creCache.projectedGv_ + creCache.projectedLv_);

  // precomputation
  // [COMPLEXITY: nx^3 + nx^2 * np]
  creCache.SmTrans_projectedAm_.noalias() = Sm.transpose() * creCache.projectedAm_;
  creCache.projectedKm_T_projectedGm_.noalias() = creCache.projectedKm_.transpose() * creCache.projectedGm_;
  if (!reducedFormRiccati_) {
    // Rm
    creCache.projectedRm_ = LinearInterpolation::interpolate(indexAlpha, *projectedModelDataPtr_, model_data::cost_dfduu);
    // [COMPLEXITY: nx * np^2]
    creCache.projectedRm_projectedKm_.noalias() = creCache.projectedRm_ * creCache.projectedKm_;
    // [COMPLEXITY: np^2]
    creCache.projectedRm_projectedLv_.noalias() = creCache.projectedRm_ * creCache.projectedLv_;
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
  dSm += creCache.deltaQm_ + creCache.SmTrans_projectedAm_ + creCache.SmTrans_projectedAm_.transpose();
  if (reducedFormRiccati_) {
    // += Km^T * Gm-
    dSm += creCache.projectedKm_T_projectedGm_;
  } else {
    // += Km^T * Gm + Gm^T * Km
    dSm += creCache.projectedKm_T_projectedGm_ + creCache.projectedKm_T_projectedGm_.transpose();
    // += Km^T * Hm * Km
    dSm.noalias() += creCache.projectedKm_.transpose() * creCache.projectedRm_projectedKm_;
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
  dSv.noalias() += Sm.transpose() * creCache.projectedHv_;
  // += Am^T * Sv
  dSv.noalias() += creCache.projectedAm_.transpose() * Sv;
  if (reducedFormRiccati_) {
    // += Gm^T * Lv
    dSv.noalias() += creCache.projectedGm_.transpose() * creCache.projectedLv_;
  } else {
    // += Gm^T * Lv
    dSv.noalias() += creCache.projectedGm_.transpose() * creCache.projectedLv_;
    // += Km^T * Gv
    dSv.noalias() += creCache.projectedKm_.transpose() * creCache.projectedGv_;
    // Km^T * Hm * Lv
    dSv.noalias() += creCache.projectedRm_projectedKm_.transpose() * creCache.projectedLv_;
  }

  /*
   * ds
   * reducedFormRiccati:
   *   [TOTAL COMPLEXITY: nx + np]
   * other
   *   [TOTAL COMPLEXITY: nx + 2np + np^2]
   */
  // += Hv^T * Sv
  ds += creCache.projectedHv_.dot(Sv);
  if (reducedFormRiccati_) {
    // += 0.5 Lv^T Gv
    ds += 0.5 * creCache.projectedLv_.dot(creCache.projectedGv_);
  } else {
    // += Lv^T Gv
    ds += creCache.projectedLv_.dot(creCache.projectedGv_);
    // += 0.5 Lv^T Hm Lv
    ds += 0.5 * creCache.projectedLv_.dot(creCache.projectedRm_projectedLv_);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ContinuousTimeRiccatiEquations::computeFlowMapILEG(std::pair<int, scalar_t> indexAlpha, const matrix_t& Sm, const vector_t& Sv,
                                                        const scalar_t& s, ContinuousTimeRiccatiData& creCache, matrix_t& dSm,
                                                        vector_t& dSv, scalar_t& ds) const {
  computeFlowMapSLQ(indexAlpha, Sm, Sv, s, creCache, dSm, dSv, ds);

  // Sigma
  creCache.dynamicsCovariance_ = LinearInterpolation::interpolate(indexAlpha, *projectedModelDataPtr_, model_data::dynamicsCovariance);

  creCache.Sigma_Sv_.noalias() = creCache.dynamicsCovariance_ * Sv;
  creCache.Sigma_Sm_.noalias() = creCache.dynamicsCovariance_ * Sm;

  dSm.noalias() += riskSensitiveCoeff_ * Sm.transpose() * creCache.Sigma_Sm_;
  dSv.noalias() += riskSensitiveCoeff_ * creCache.Sigma_Sm_.transpose() * Sv;
  ds += 0.5 * creCache.Sigma_Sm_.trace() + 0.5 * riskSensitiveCoeff_ * Sv.dot(creCache.Sigma_Sv_);
}

}  // namespace ocs2
