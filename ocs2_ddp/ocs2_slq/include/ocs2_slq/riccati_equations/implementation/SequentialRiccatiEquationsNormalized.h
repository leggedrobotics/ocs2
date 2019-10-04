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

namespace ocs2 {

template <int STATE_DIM, int INPUT_DIM>
SequentialRiccatiEquationsNormalized<STATE_DIM, INPUT_DIM>::SequentialRiccatiEquationsNormalized(bool useMakePSD,
                                                                                                 bool preComputeRiccatiTerms)
    : useMakePSD_(useMakePSD), preComputeRiccatiTerms_(preComputeRiccatiTerms) {}

template <int STATE_DIM, int INPUT_DIM>
void SequentialRiccatiEquationsNormalized<STATE_DIM, INPUT_DIM>::convert2Vector(const state_matrix_t& Sm, const state_vector_t& Sv,
                                                                                const eigen_scalar_t& s, s_vector_t& allSs) {
  /* Sm is symmetric. Here, we only extract the upper triangular part and
   * transcribe it in column-wise fashion into allSs*/
  size_t count = 0;  // count the total number of scalar entries covered
  size_t nRows = 0;
  const size_t state_dim = Sm.rows();

  // Ensure proper size in case of Eigen::Dynamic size.
  allSs.resize(state_dim * (state_dim + 1) / 2 + state_dim + 1);

  for (size_t nCols = 0; nCols < state_dim; nCols++) {
    nRows = nCols + 1;
    allSs.segment(count, nRows) << Eigen::Map<const dynamic_vector_t>(Sm.data() + nCols * state_dim, nRows);
    count += nRows;
  }

  /* add data from Sv on top*/
  allSs.segment(count, state_dim) << Eigen::Map<const dynamic_vector_t>(Sv.data(), state_dim);

  /* add s as last element*/
  allSs.template tail<1>() << s;
}

template <int STATE_DIM, int INPUT_DIM>
void SequentialRiccatiEquationsNormalized<STATE_DIM, INPUT_DIM>::convert2Matrix(const s_vector_t& allSs, state_matrix_t& Sm,
                                                                                state_vector_t& Sv, eigen_scalar_t& s) {
  /* Sm is symmetric. Here, we map the first entries from allSs onto the
   * respective elements in the symmetric matrix*/
  size_t count = 0;
  size_t nCols = 0;
  const size_t state_dim = Sm.rows();
  for (size_t rows = 0; rows < state_dim; rows++) {
    nCols = rows + 1;
    Sm.block(rows, 0, 1, nCols) << Eigen::Map<const dynamic_vector_t>(allSs.data() + count, nCols).transpose();
    // "nCols-1" because diagonal elements have already been covered
    Sm.block(0, rows, nCols - 1, 1) << Eigen::Map<const dynamic_vector_t>(allSs.data() + count, nCols - 1);
    count += nCols;
  }

  /* extract the vector Sv*/
  Sv = Eigen::Map<const dynamic_vector_t>(allSs.data() + count, state_dim);

  /* extract s as the last element */
  s = allSs.template tail<1>();
}

template <int STATE_DIM, int INPUT_DIM>
void SequentialRiccatiEquationsNormalized<STATE_DIM, INPUT_DIM>::setData(
    const scalar_array_t* timeStampPtr, const state_matrix_array_t* AmPtr, const state_input_matrix_array_t* BmPtr,
    const eigen_scalar_array_t* qPtr, const state_vector_array_t* QvPtr, const state_matrix_array_t* QmPtr,
    const input_vector_array_t* RvPtr, const dynamic_matrix_array_t* RinvCholPtr, const input_state_matrix_array_t* PmPtr,
    const size_array_t* eventsPastTheEndIndecesPtr, const eigen_scalar_array_t* qFinalPtr, const state_vector_array_t* QvFinalPtr,
    const state_matrix_array_t* QmFinalPtr) {
  BASE::resetNumFunctionCalls();

  const int state_dim = (*BmPtr)[0].rows();
  const int input_dim = (*BmPtr)[0].cols();

  // Initialize members with proper dimensions for Eigen::Dynamic sized matrices.
  Sm_.resize(state_dim, state_dim);
  Sv_.resize(state_dim);
  Qm_.resize(state_dim, state_dim);
  Qv_.resize(state_dim);
  AmT_minus_P_Rinv_Bm_.resize(state_dim, state_dim);
  AmT_Sm_.resize(state_dim, state_dim);
  Am_.resize(state_dim, state_dim);
  Bm_.resize(state_dim, input_dim);
  Rv_.resize(input_dim);
  Pm_.resize(input_dim, state_dim);

  eventTimes_.clear();
  eventTimes_.reserve(eventsPastTheEndIndecesPtr->size());
  for (const size_t& pastTheEndIndex : *eventsPastTheEndIndecesPtr) {
    eventTimes_.push_back((*timeStampPtr)[pastTheEndIndex - 1]);
  }

  qFinalPtr_ = qFinalPtr;
  QvFinalPtr_ = QvFinalPtr;
  QmFinalPtr_ = QmFinalPtr;

  if (preComputeRiccatiTerms_) {
    // Initialize all arrays that will store the precomputation
    const size_t N = AmPtr->size();
    B_RinvChol_array_.clear();
    B_RinvChol_array_.reserve(N);
    RinvCholT_Rv_array_.clear();
    RinvCholT_Rv_array_.reserve(N);
    AmT_minus_P_Rinv_B_array_.clear();
    AmT_minus_P_Rinv_B_array_.reserve(N);

    // These terms are initialized by copying the cost function terms and substracting the rest inside the loop below
    Qm_minus_P_Rinv_P_array_ = *QmPtr;
    Qv_minus_P_Rinv_Rv_array_ = *QvPtr;
    q_minus_half_Rv_Rinv_Rv_array_ = *qPtr;

    // Precompute all terms for all interpolation nodes
    dynamic_matrix_t PmT_RinvChol;
    for (size_t i = 0; i < N; i++) {
      // Emplace back on first touch of the array in this loop
      B_RinvChol_array_.emplace_back((*BmPtr)[i] * (*RinvCholPtr)[i]);
      RinvCholT_Rv_array_.emplace_back((*RinvCholPtr)[i].transpose() * (*RvPtr)[i]);
      AmT_minus_P_Rinv_B_array_.emplace_back((*AmPtr)[i].transpose());

      // Modify AmT_minus_P_Rinv_B_array_ in place + store temporary computation
      PmT_RinvChol.noalias() = (*PmPtr)[i].transpose() * (*RinvCholPtr)[i];
      AmT_minus_P_Rinv_B_array_[i].noalias() -= PmT_RinvChol * B_RinvChol_array_[i].transpose();

      // Modify the constraints in place
      Qm_minus_P_Rinv_P_array_[i].noalias() -= PmT_RinvChol * PmT_RinvChol.transpose();
      Qv_minus_P_Rinv_Rv_array_[i].noalias() -= PmT_RinvChol * RinvCholT_Rv_array_[i];
    }

    // Set the data to the interpolator (array pointer would be the same, but time pointer might change)
    Qm_minus_P_Rinv_P_func_.setData(timeStampPtr, &Qm_minus_P_Rinv_P_array_);
    Qv_minus_P_Rinv_Rv_func_.setData(timeStampPtr, &Qv_minus_P_Rinv_Rv_array_);
    q_minus_half_Rv_Rinv_Rv_func_.setData(timeStampPtr, &q_minus_half_Rv_Rinv_Rv_array_);
    AmT_minus_P_Rinv_B_func_.setData(timeStampPtr, &AmT_minus_P_Rinv_B_array_);
    B_RinvChol_func_.setData(timeStampPtr, &B_RinvChol_array_);
    RinvCholT_Rv_func_.setData(timeStampPtr, &RinvCholT_Rv_array_);
  } else {  // if not preComputeRiccatiTerms_
    QmFunc_.setData(timeStampPtr, QmPtr);
    QvFunc_.setData(timeStampPtr, QvPtr);
    qFunc_.setData(timeStampPtr, qPtr);
    AmFunc_.setData(timeStampPtr, AmPtr);
    BmFunc_.setData(timeStampPtr, BmPtr);
    RinvChol_Func_.setData(timeStampPtr, RinvCholPtr);
    PmFunc_.setData(timeStampPtr, PmPtr);
    RvFunc_.setData(timeStampPtr, RvPtr);
  }
}

template <int STATE_DIM, int INPUT_DIM>
void SequentialRiccatiEquationsNormalized<STATE_DIM, INPUT_DIM>::computeJumpMap(const scalar_t& z, const s_vector_t& state,
                                                                                s_vector_t& mappedState) {
  scalar_t time = -z;

  // epsilon is set to include times past event times which have been artificially increased in the rollout
  size_t index = lookup::findFirstIndexWithinTol(eventTimes_, time, 1e-5);

  s_vector_t allSsJump;
  convert2Vector((*QmFinalPtr_)[index], (*QvFinalPtr_)[index], (*qFinalPtr_)[index], allSsJump);

  mappedState = state + allSsJump;
}

template <int STATE_DIM, int INPUT_DIM>
void SequentialRiccatiEquationsNormalized<STATE_DIM, INPUT_DIM>::computeFlowMap(const scalar_t& z, const s_vector_t& allSs,
                                                                                s_vector_t& derivatives) {
  /*note: according to some discussions on stackoverflow, it does not buy
   * computation time if multiplications with symmetric matrices are executed
   * using selfadjointView(). Doing the full multiplication seems to be faster
   * because of vectorization
   */
  /*
   *  Expressions written base on guidelines in
   * http://eigen.tuxfamily.org/dox/TopicWritingEfficientProductExpression.html
   */
  BASE::numFunctionCalls_++;

  // denormalized time
  const scalar_t t = -z;

  convert2Matrix(allSs, Sm_, Sv_, s_);

  if (useMakePSD_) {
    LinearAlgebra::makePSD(Sm_);
  }

  if (preComputeRiccatiTerms_) {
    const auto indexAlpha = Qm_minus_P_Rinv_P_func_.interpolate(t, Qm_);
    Qv_minus_P_Rinv_Rv_func_.interpolate(indexAlpha, Qv_);
    q_minus_half_Rv_Rinv_Rv_func_.interpolate(indexAlpha, q_);
    AmT_minus_P_Rinv_B_func_.interpolate(indexAlpha, AmT_minus_P_Rinv_Bm_);
    RinvCholT_Rv_func_.interpolate(indexAlpha, RinvCholT_Rv_);
    B_RinvChol_func_.interpolate(indexAlpha, B_RinvChol_);

    // dSmdt,  Qm_ used instead of temporary
    AmT_Sm_.noalias() = AmT_minus_P_Rinv_Bm_ * Sm_;
    Qm_ += AmT_Sm_ + AmT_Sm_.transpose();
    SmT_B_RinvChol_.noalias() = Sm_.transpose() * B_RinvChol_;
    Qm_.noalias() -= SmT_B_RinvChol_ * SmT_B_RinvChol_.transpose();

    // dSvdt,  Qv_ used instead of temporary
    Qv_.noalias() += AmT_minus_P_Rinv_Bm_ * Sv_;
    RinvCholT_Rv_.noalias() += B_RinvChol_.transpose() * Sv_;
    Qv_.noalias() -= SmT_B_RinvChol_ * RinvCholT_Rv_;

    // dsdt,   q_ used instead of temporary
    q_.noalias() -= 0.5 * RinvCholT_Rv_.transpose() * RinvCholT_Rv_;
  } else {
    const auto indexAlpha = QmFunc_.interpolate(t, Qm_);
    QvFunc_.interpolate(indexAlpha, Qv_);
    qFunc_.interpolate(indexAlpha, q_);
    AmFunc_.interpolate(indexAlpha, Am_);
    BmFunc_.interpolate(indexAlpha, Bm_);
    RinvChol_Func_.interpolate(indexAlpha, RinvChol_);
    PmFunc_.interpolate(indexAlpha, Pm_);
    RvFunc_.interpolate(indexAlpha, Rv_);

    Pm_.noalias() += Bm_.transpose() * Sm_;  // ! Pm is changed to avoid an extra temporary
    SmT_B_RinvChol_.noalias() = RinvChol_.transpose() * Pm_;
    Rv_.noalias() += Bm_.transpose() * Sv_;  // ! Rv is changed to avoid an extra temporary
    RinvCholT_Rv_.noalias() = RinvChol_.transpose() * Rv_;

    AmT_Sm_.noalias() = Am_.transpose() * Sm_.transpose();

    // dSmdt,  Qm_ used instead of temporary
    Qm_ += AmT_Sm_ + AmT_Sm_.transpose();
    Qm_.noalias() -= SmT_B_RinvChol_.transpose() * SmT_B_RinvChol_;

    // dSvdt,  Qv_ used instead of temporary
    Qv_.noalias() += Am_.transpose() * Sv_;
    Qv_.noalias() -= SmT_B_RinvChol_.transpose() * RinvCholT_Rv_;

    // dsdt,   q_ used instead of temporary
    q_.noalias() -= 0.5 * RinvCholT_Rv_.transpose() * RinvCholT_Rv_;
  }

  convert2Vector(Qm_, Qv_, q_, derivatives);
}

}  // namespace ocs2
