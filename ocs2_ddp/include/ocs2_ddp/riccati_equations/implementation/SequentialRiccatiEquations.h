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

#include <ocs2_core/misc/LinearAlgebra.h>
#include <ocs2_core/misc/Lookup.h>
#include <ocs2_core/model_data/ModelDataLinearInterpolation.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int STATE_DIM, int INPUT_DIM>
SequentialRiccatiEquations<STATE_DIM, INPUT_DIM>::SequentialRiccatiEquations(bool preComputeRiccatiTerms)
    : preComputeRiccatiTerms_(preComputeRiccatiTerms) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int STATE_DIM, int INPUT_DIM>
void SequentialRiccatiEquations<STATE_DIM, INPUT_DIM>::convert2Vector(const dynamic_matrix_t& Sm, const dynamic_vector_t& Sv,
                                                                      const scalar_t& s, s_vector_t& allSs) {
  /* Sm is symmetric. Here, we only extract the upper triangular part and
   * transcribe it in column-wise fashion into allSs*/
  size_t count = 0;  // count the total number of scalar entries covered
  size_t nRows = 0;
  const size_t state_dim = Sm.cols();

  assert(Sm.rows() == state_dim);
  assert(Sv.rows() == state_dim);

  // Ensure proper size in case of Eigen::Dynamic size.
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
void SequentialRiccatiEquations<STATE_DIM, INPUT_DIM>::convert2Matrix(const s_vector_t& allSs, state_matrix_t& Sm, state_vector_t& Sv,
                                                                      scalar_t& s) {
  /* Sm is symmetric. Here, we map the first entries from allSs onto the
   * upper triangular part of the symmetric matrix*/
  size_t count = 0;
  size_t nRows = 0;
  const size_t state_dim = Sm.cols();
  for (size_t col = 0; col < state_dim; col++) {
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
void SequentialRiccatiEquations<STATE_DIM, INPUT_DIM>::setData(const scalar_array_t* timeStampPtr,
                                                               const ModelDataBase::array_t* projectedModelDataPtr,
                                                               const size_array_t* postEventIndicesPtr,
                                                               const ModelDataBase::array_t* modelDataEventTimesPtr,
                                                               const RiccatiModificationBase::array_t* riccatiModificationPtr) {
  BASE::resetNumFunctionCalls();

  const auto state_dim = projectedModelDataPtr->front().stateDim_;
  const auto input_dim = projectedModelDataPtr->front().inputDim_;

  // Initialize members with proper dimensions for Eigen::Dynamic sized matrices.
  Sm_.resize(state_dim, state_dim);
  Sv_.resize(state_dim);
  dSm_.resize(state_dim, state_dim);
  dSv_.resize(state_dim);
  AmT_minus_P_Rinv_Bm_.resize(state_dim, state_dim);
  Am_T_Sm_.resize(state_dim, state_dim);
  Am_.resize(state_dim, state_dim);
  Bm_.resize(state_dim, input_dim);
  Gv_.resize(input_dim);
  Gm_.resize(input_dim, state_dim);

  deltaQm_.resize(state_dim, state_dim);

  eventTimes_.clear();
  eventTimes_.reserve(postEventIndicesPtr->size());
  for (const auto& postEventIndex : *postEventIndicesPtr) {
    eventTimes_.push_back((*timeStampPtr)[postEventIndex - 1]);
  }
  modelDataEventTimesPtr_ = modelDataEventTimesPtr;

  // saving array pointers
  timeStampPtr_ = timeStampPtr;
  projectedModelDataPtr_ = projectedModelDataPtr;

  riccatiModificationPtr_ = riccatiModificationPtr;

  const int N = timeStampPtr->size();
  if (preComputeRiccatiTerms_) {
    // Initialize all arrays that will store the precomputation
    B_RmInvUUT_array_.clear();
    B_RmInvUUT_array_.reserve(N);
    RmInvUUT_T_Rv_array_.clear();
    RmInvUUT_T_Rv_array_.reserve(N);
    AmT_minus_P_Rinv_B_array_.clear();
    AmT_minus_P_Rinv_B_array_.reserve(N);
    Qv_minus_P_Rinv_Rv_array_.clear();
    Qv_minus_P_Rinv_Rv_array_.reserve(N);
    Qm_minus_P_Rinv_P_array_.clear();
    Qm_minus_P_Rinv_P_array_.reserve(N);

    // Precompute all terms for all interpolation nodes
    dynamic_matrix_t Pm_T_HmInvUUT;
    for (int i = 0; i < N; i++) {
      const auto& HmInvUUT = (*riccatiModificationPtr)[i].HmInverseConstrainedLowRank_;
      // Emplace back on first touch of the array in this loop
      B_RmInvUUT_array_.emplace_back((*projectedModelDataPtr)[i].dynamicsInputDerivative_ * HmInvUUT);
      RmInvUUT_T_Rv_array_.emplace_back(HmInvUUT.transpose() * (*projectedModelDataPtr)[i].costInputDerivative_);
      AmT_minus_P_Rinv_B_array_.emplace_back((*projectedModelDataPtr)[i].dynamicsStateDerivative_.transpose());
      Qv_minus_P_Rinv_Rv_array_.emplace_back((*projectedModelDataPtr)[i].costStateDerivative_);
      Qm_minus_P_Rinv_P_array_.emplace_back((*projectedModelDataPtr)[i].costStateSecondDerivative_ + (*riccatiModificationPtr)[i].deltaQm_);

      // modify AmT_minus_P_Rinv_B_array_ in place + store temporary computation
      Pm_T_HmInvUUT.noalias() = (*projectedModelDataPtr)[i].costInputStateDerivative_.transpose() * HmInvUUT;
      AmT_minus_P_Rinv_B_array_[i].noalias() -= Pm_T_HmInvUUT * B_RmInvUUT_array_[i].transpose();

      Qv_minus_P_Rinv_Rv_array_[i].noalias() -= Pm_T_HmInvUUT * RmInvUUT_T_Rv_array_[i];
      Qm_minus_P_Rinv_P_array_[i].noalias() -= Pm_T_HmInvUUT * Pm_T_HmInvUUT.transpose();
    }

  } else {
    HmInvUUT_T_deltaGv_array_.clear();
    HmInvUUT_T_deltaGv_array_.reserve(N);
    HmInvUUT_T_deltaGm_array_.clear();
    HmInvUUT_T_deltaGm_array_.reserve(N);
    HmInvUUT_T_Rm_HmInvUUT_array_.clear();
    HmInvUUT_T_Rm_HmInvUUT_array_.reserve(N);
    // precomputation
    dynamic_matrix_t HmInvUUT_T_Rm;
    for (int i = 0; i < N; i++) {
      const auto& modelDataPtr = (*projectedModelDataPtr)[i];
      const auto& riccatiModification = (*riccatiModificationPtr)[i];
      HmInvUUT_T_Rm.noalias() = riccatiModification.HmInverseConstrainedLowRank_.transpose() * modelDataPtr.costInputSecondDerivative_;

      HmInvUUT_T_deltaGv_array_.emplace_back(HmInvUUT_T_Rm * modelDataPtr.stateInputEqConstr_);
      HmInvUUT_T_deltaGm_array_.emplace_back(HmInvUUT_T_Rm * modelDataPtr.stateInputEqConstrStateDerivative_);
      HmInvUUT_T_deltaGm_array_.back().noalias() +=
          riccatiModification.HmInverseConstrainedLowRank_.transpose() * riccatiModification.deltaPm_;
      HmInvUUT_T_Rm_HmInvUUT_array_.emplace_back(HmInvUUT_T_Rm * riccatiModification.HmInverseConstrainedLowRank_);
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int STATE_DIM, int INPUT_DIM>
void SequentialRiccatiEquations<STATE_DIM, INPUT_DIM>::computeJumpMap(const scalar_t& z, const s_vector_t& allSs,
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
  Am_T_Sm_.noalias() = Am.transpose() * Sm_;
  SmPreEvent.noalias() += Am_T_Sm_ * Am;

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
void SequentialRiccatiEquations<STATE_DIM, INPUT_DIM>::computeFlowMap(const scalar_t& z, const s_vector_t& allSs, s_vector_t& derivatives) {
  /* note: according to some discussions on stackoverflow, it does not buy
   * computation time if multiplications with symmetric matrices are executed
   * using selfadjointView(). Doing the full multiplication seems to be faster
   * because of vectorization
   */
  /*
   * Expressions written base on guidelines in
   * http://eigen.tuxfamily.org/dox/TopicWritingEfficientProductExpression.html
   */
  BASE::numFunctionCalls_++;

  // denormalized time
  const scalar_t t = -z;

  convert2Matrix(allSs, Sm_, Sv_, s_);

  const auto indexAlpha = EigenLinearInterpolation<state_matrix_t>::timeSegment(t, timeStampPtr_);
  if (preComputeRiccatiTerms_) {
    ModelData::LinearInterpolation::interpolate(indexAlpha, Hv_, projectedModelDataPtr_, ModelData::dynamicsBias);
    ModelData::LinearInterpolation::interpolate(indexAlpha, ds_, projectedModelDataPtr_, ModelData::cost);
    EigenLinearInterpolation<dynamic_matrix_t>::interpolate(indexAlpha, dSm_, &Qm_minus_P_Rinv_P_array_);
    EigenLinearInterpolation<dynamic_vector_t>::interpolate(indexAlpha, dSv_, &Qv_minus_P_Rinv_Rv_array_);
    EigenLinearInterpolation<dynamic_matrix_t>::interpolate(indexAlpha, AmT_minus_P_Rinv_Bm_, &AmT_minus_P_Rinv_B_array_);
    EigenLinearInterpolation<dynamic_vector_t>::interpolate(indexAlpha, RmInvUUT_T_Rv_, &RmInvUUT_T_Rv_array_);
    EigenLinearInterpolation<dynamic_matrix_t>::interpolate(indexAlpha, B_RmInvUUT_, &B_RmInvUUT_array_);

    // dSmdt [TOTAL COMPLEXITY: (nx^3) + 2(nx^2 * np)]
    Am_T_Sm_.noalias() = AmT_minus_P_Rinv_Bm_ * Sm_;
    dSm_ += Am_T_Sm_ + Am_T_Sm_.transpose();
    SmT_B_RmInvUUT_.noalias() = Sm_.transpose() * B_RmInvUUT_;
    dSm_.noalias() -= SmT_B_RmInvUUT_ * SmT_B_RmInvUUT_.transpose();

    // dSvdt [TOTAL COMPLEXITY: 2*(nx^2) + 2*(nx * np)]
    dSv_.noalias() += Sm_.transpose() * Hv_;
    dSv_.noalias() += AmT_minus_P_Rinv_Bm_ * Sv_;
    RmInvUUT_T_Rv_.noalias() += B_RmInvUUT_.transpose() * Sv_;
    dSv_.noalias() -= SmT_B_RmInvUUT_ * RmInvUUT_T_Rv_;

    // dsdt [TOTAL COMPLEXITY: nx + np]
    ds_ += Sv_.dot(Hv_);
    ds_ -= 0.5 * RmInvUUT_T_Rv_.dot(RmInvUUT_T_Rv_);

  } else {
    // Hv
    ModelData::LinearInterpolation::interpolate(indexAlpha, Hv_, projectedModelDataPtr_, ModelData::dynamicsBias);
    // Am
    ModelData::LinearInterpolation::interpolate(indexAlpha, Am_, projectedModelDataPtr_, ModelData::dynamicsStateDerivative);
    // Bm
    ModelData::LinearInterpolation::interpolate(indexAlpha, Bm_, projectedModelDataPtr_, ModelData::dynamicsInputDerivative);
    // q
    ModelData::LinearInterpolation::interpolate(indexAlpha, ds_, projectedModelDataPtr_, ModelData::cost);
    // Qv
    ModelData::LinearInterpolation::interpolate(indexAlpha, dSv_, projectedModelDataPtr_, ModelData::costStateDerivative);
    // Qm
    ModelData::LinearInterpolation::interpolate(indexAlpha, dSm_, projectedModelDataPtr_, ModelData::costStateSecondDerivative);
    // Rv
    ModelData::LinearInterpolation::interpolate(indexAlpha, Gv_, projectedModelDataPtr_, ModelData::costInputDerivative);
    // Pm
    ModelData::LinearInterpolation::interpolate(indexAlpha, Gm_, projectedModelDataPtr_, ModelData::costInputStateDerivative);

    EigenLinearInterpolation<dynamic_vector_t>::interpolate(indexAlpha, HmInvUUT_T_GvAug_, &HmInvUUT_T_deltaGv_array_);
    EigenLinearInterpolation<dynamic_matrix_t>::interpolate(indexAlpha, HmInvUUT_T_GmAug_, &HmInvUUT_T_deltaGm_array_);
    EigenLinearInterpolation<dynamic_matrix_t>::interpolate(indexAlpha, HmInvUUT_T_Rm_HmInvUUT_, &HmInvUUT_T_Rm_HmInvUUT_array_);

    RiccatiModification::LinearInterpolation::interpolate(indexAlpha, deltaQm_, riccatiModificationPtr_, RiccatiModification::deltaQm);
    RiccatiModification::LinearInterpolation::interpolate(indexAlpha, HmInvUUT_, riccatiModificationPtr_,
                                                          RiccatiModification::HmInverseConstrainedLowRank);

    /*
     * dSmdt
     */
    // Am^T * Sm [COMPLEXITY: nx^3]
    Am_T_Sm_.noalias() = Am_.transpose() * Sm_;

    // Gm = Pm + Bm^T Sm [COMPLEXITY: nx^2 * nu]
    Gm_.noalias() += Bm_.transpose() * Sm_;
    // Km = inv(Hm) * Gm = (HmInvUUT * HmInvUUT^T) * Gm = HmInvUUT * HmInvUUT_T_Gm [COMPLEXITY: nx * nu * np]
    HmInvUUT_T_Gm_.noalias() = HmInvUUT_.transpose() * Gm_;
    HmInvUUT_T_GmAug_ += HmInvUUT_T_Gm_;

    // Km^T * Gm [COMPLEXITY: nx^2 * np]
    Km_T_Gm_.noalias() = -HmInvUUT_T_GmAug_.transpose() * HmInvUUT_T_Gm_;

    // Km^T * Rm * Km [COMPLEXITY: (nx * np^2) + (nx^2 * np)]
    HmInvUUT_T_Rm_Km_.noalias() = -HmInvUUT_T_Rm_HmInvUUT_.transpose() * HmInvUUT_T_GmAug_;
    Km_T_Rm_Km_.noalias() = -HmInvUUT_T_GmAug_.transpose() * HmInvUUT_T_Rm_Km_;

    // dSm [TOTAL COMPLEXITY: (nx^3) + (nx^2*nu) + 2*(nx^2*np) + (nx*nu*np) + (nx*np^2)]
    dSm_ += deltaQm_ + Am_T_Sm_ + Am_T_Sm_.transpose() + Km_T_Gm_ + Km_T_Gm_.transpose() + Km_T_Rm_Km_;

    /*
     * dSvdt
     */
    // += Sm^T * Hv [COMPLEXITY: nx^2]
    dSv_.noalias() += Sm_.transpose() * Hv_;

    // += Am^T * Sv [COMPLEXITY: nx^2]
    dSv_.noalias() += Am_.transpose() * Sv_;

    // Gv = Rv + Bm^T Sv [COMPLEXITY: (nx * nu) + (nu * np)]
    Gv_.noalias() += Bm_.transpose() * Sv_;
    HmInvUUT_T_Gv_.noalias() = HmInvUUT_.transpose() * Gv_;
    HmInvUUT_T_GvAug_ += HmInvUUT_T_Gv_;

    // (Gm^T * Lv) [COMPLEXITY: nx * np]
    Gm_T_Lv_.noalias() = -HmInvUUT_T_Gm_.transpose() * HmInvUUT_T_GvAug_;

    // (Km^T * Gv) [COMPLEXITY: nx * np]
    Km_T_Gv_.noalias() = -HmInvUUT_T_GmAug_.transpose() * HmInvUUT_T_Gv_;

    // Km^T * Rm * Lv [COMPLEXITY: (np^2) + (nx * np)]
    HmInvUUT_T_Rm_Lv_.noalias() = -HmInvUUT_T_Rm_HmInvUUT_.transpose() * HmInvUUT_T_GvAug_;
    Km_T_Rm_Lv_.noalias() = -HmInvUUT_T_GmAug_.transpose() * HmInvUUT_T_Rm_Lv_;

    // dSv [TOTAL COMPLEXITY: 2*(nx^2) + (nx*nu) + 3*(nx*np) + (nu*np) + (np^2)]
    dSv_ += Gm_T_Lv_ + Km_T_Gv_ + Km_T_Rm_Lv_;

    /*
     * dsdt
     */
    // += Sv^T * Hv [COMPLEXITY: nx]
    ds_ += Sv_.dot(Hv_);
    // += Lv^T Gv [COMPLEXITY: np]
    ds_ -= HmInvUUT_T_GvAug_.dot(HmInvUUT_T_Gv_);
    // += 0.5 Lv^T Rm Lv [COMPLEXITY: np]
    ds_ -= 0.5 * HmInvUUT_T_GvAug_.dot(HmInvUUT_T_Rm_Lv_);
  }

  convert2Vector(dSm_, dSv_, ds_, derivatives);
}

}  // namespace ocs2
