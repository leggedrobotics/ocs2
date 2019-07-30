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

#ifndef SEQUENTIALRICCATIEQUATIONSNORMALIZED_OCS2_H_
#define SEQUENTIALRICCATIEQUATIONSNORMALIZED_OCS2_H_

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <iostream>
#include <vector>

#include <ocs2_core/Dimensions.h>
#include <ocs2_core/integration/OdeBase.h>
#include <ocs2_core/misc/LinearAlgebra.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_core/misc/Lookup.h>

namespace ocs2 {

/**
 * This class implements the time-normalized Riccati equations for SLQ problem.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class SequentialRiccatiEquationsNormalized final : public OdeBase<STATE_DIM*(STATE_DIM + 1) / 2 + STATE_DIM + 1> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** If STATE_DIM=n, Then: n(n+1)/2 entries from triangular matrix Sm, n entries from vector Sv and +1 one from a scalar */
  static constexpr size_t S_DIM_ = (STATE_DIM * (STATE_DIM + 1) / 2 + STATE_DIM + 1);

  using BASE = OdeBase<S_DIM_>;

  typedef Dimensions<STATE_DIM, INPUT_DIM> DIMENSIONS;
  using scalar_t = typename DIMENSIONS::scalar_t;
  using scalar_array_t = typename DIMENSIONS::scalar_array_t;
  using size_array_t = typename DIMENSIONS::size_array_t;
  using eigen_scalar_t = typename DIMENSIONS::eigen_scalar_t;
  using eigen_scalar_array_t = typename DIMENSIONS::eigen_scalar_array_t;
  using state_vector_t = typename DIMENSIONS::state_vector_t;
  using state_vector_array_t = typename DIMENSIONS::state_vector_array_t;
  using input_vector_t = typename DIMENSIONS::input_vector_t;
  using input_vector_array_t = typename DIMENSIONS::input_vector_array_t;
  using input_state_matrix_t = typename DIMENSIONS::input_state_matrix_t;
  using input_state_matrix_array_t = typename DIMENSIONS::input_state_matrix_array_t;
  using state_matrix_t = typename DIMENSIONS::state_matrix_t;
  using state_matrix_array_t = typename DIMENSIONS::state_matrix_array_t;
  using input_matrix_t = typename DIMENSIONS::input_matrix_t;
  using input_matrix_array_t = typename DIMENSIONS::input_matrix_array_t;
  using state_input_matrix_t = typename DIMENSIONS::state_input_matrix_t;
  using state_input_matrix_array_t = typename DIMENSIONS::state_input_matrix_array_t;
  using dynamic_vector_t = typename DIMENSIONS::dynamic_vector_t;
  using dynamic_matrix_t = typename DIMENSIONS::dynamic_matrix_t;
  using dynamic_matrix_array_t = typename DIMENSIONS::dynamic_matrix_array_t;
  using dynamic_vector_array_t = typename DIMENSIONS::dynamic_vector_array_t;

  typedef Eigen::Matrix<scalar_t, S_DIM_, 1> s_vector_t;
  typedef std::vector<s_vector_t, Eigen::aligned_allocator<s_vector_t> > s_vector_array_t;

  /**
   * Constructor.
   */
  SequentialRiccatiEquationsNormalized(const bool& useMakePSD, const scalar_t& addedRiccatiDiagonal, bool normalizeTime,
                                       bool preComputeRiccatiTerms = true)

      : useMakePSD_(useMakePSD),
        addedRiccatiDiagonal_(addedRiccatiDiagonal),
        switchingTimeStart_(0.0),
        switchingTimeFinal_(1.0),
        scalingFactor_(1.0),
        normalizeTime_(normalizeTime),
        preComputeRiccatiTerms_(preComputeRiccatiTerms),
        Sm_(state_matrix_t::Zero()),
        Sv_(state_vector_t::Zero()),
        s_(eigen_scalar_t::Zero()),
        Qm_(state_matrix_t::Zero()),
        Qv_(state_vector_t::Zero()),
        q_(eigen_scalar_t::Zero()),
        AmT_minus_P_Rinv_Bm_(state_matrix_t::Zero()),
        AmT_Sm_(state_matrix_t::Zero()),
        Am_(state_matrix_t::Zero()),
        Bm_(state_input_matrix_t::Zero()),
        Rv_(input_vector_t::Zero()),
        Pm_(input_state_matrix_t::Zero()) {}

  /**
   * Default destructor.
   */
  ~SequentialRiccatiEquationsNormalized() = default;

  /**
   * Transcribe symmetric matrix Sm, vector Sv and scalar s into a single vector.
   *
   * @param [in] Sm: \f$ S_m \f$
   * @param [in] Sv: \f$ S_v \f$
   * @param [in] s: \f$ s \f$
   * @param [out] allSs: Single vector constructed by concatenating Sm, Sv and s.
   */
  static void convert2Vector(const state_matrix_t& Sm, const state_vector_t& Sv, const eigen_scalar_t& s, s_vector_t& allSs) {
    /* Sm is symmetric. Here, we only extract the upper triangular part and transcribe it in column-wise fashion into allSs*/
    size_t count = 0;  // count the total number of scalar entries covered
    size_t nRows = 0;
    for (size_t nCols = 0; nCols < STATE_DIM; nCols++) {
      nRows = nCols + 1;
      allSs.segment(count, nRows) << Eigen::Map<const dynamic_vector_t>(Sm.data() + nCols * STATE_DIM, nRows);
      count += nRows;
    }

    /* add data from Sv on top*/
    allSs.template segment<STATE_DIM>((STATE_DIM * (STATE_DIM + 1)) / 2) << Eigen::Map<const dynamic_vector_t>(Sv.data(), STATE_DIM);

    /* add s as last element*/
    allSs.template tail<1>() << s;
  }

  /**
   * Transcribes the stacked vector allSs into a symmetric matrix, Sm, a vector, Sv and a single scalar, s.
   *
   * @param [in] allSs: Single vector constructed by concatenating Sm, Sv and s.
   * @param [out] Sm: \f$ S_m \f$
   * @param [out] Sv: \f$ S_v \f$
   * @param [out] s: \f$ s \f$
   */
  static void convert2Matrix(const s_vector_t& allSs, state_matrix_t& Sm, state_vector_t& Sv, eigen_scalar_t& s) {
    /* Sm is symmetric. Here, we map the first entries from allSs onto the respective elements in the symmetric matrix*/
    size_t count = 0;
    size_t nCols = 0;
    for (size_t rows = 0; rows < STATE_DIM; rows++) {
      nCols = rows + 1;
      Sm.block(rows, 0, 1, nCols) << Eigen::Map<const dynamic_vector_t>(allSs.data() + count, nCols).transpose();
      // "nCols-1" because diagonal elements have already been covered
      Sm.block(0, rows, nCols - 1, 1) << Eigen::Map<const dynamic_vector_t>(allSs.data() + count, nCols - 1);
      count += nCols;
    }

    /* extract the vector Sv*/
    Sv = Eigen::Map<const dynamic_vector_t>(allSs.data() + (STATE_DIM * (STATE_DIM + 1)) / 2, STATE_DIM);

    /* extract s as the last element */
    s = allSs.template tail<1>();
  }

  /**
   * Sets coefficients of the model.
   *
   * @param [in] learningRate: The learning rate.
   * @param [in] switchingTimeStart: The start time of the subsystem.
   * @param [in] switchingTimeFinal: The final time of the subsystem.
   * @param [in] timeStampPtr: A pointer to the time stamp trajectory.
   * @param [in] AmPtr: A pointer to the trajectory of \f$ A_m(t) \f$ .
   * @param [in] BmPtr: A pointer to the trajectory of \f$ B_m(t) \f$ .
   * @param [in] qPtr: A pointer to the trajectory of \f$ q(t) \f$ .
   * @param [in] QvPtr: A pointer to the trajectory of \f$ Q_v(t) \f$ .
   * @param [in] QmPtr: A pointer to the trajectory of \f$ Q_m(t) \f$ .
   * @param [in] RvPtr: A pointer to the trajectory of \f$ R_v(t) \f$ .
   * @param [in] RmInversePtr: A pointer to the trajectory of \f$ R_m^{-1}(t) \f$ .
   * @param [in] RmPtr: A pointer to the trajectory of \f$ R_m(t) \f$ .
   * @param [in] PmPtr: A pointer to the trajectory of \f$ P_m(t) \f$ .
   */
  void setData(const scalar_t& switchingTimeStart, const scalar_t& switchingTimeFinal, const scalar_array_t* timeStampPtr,
               const state_matrix_array_t* AmPtr, const state_input_matrix_array_t* BmPtr, const eigen_scalar_array_t* qPtr,
               const state_vector_array_t* QvPtr, const state_matrix_array_t* QmPtr, const input_vector_array_t* RvPtr,
               const dynamic_matrix_array_t* RinvCholPtr, const input_state_matrix_array_t* PmPtr,
               const size_array_t* eventsPastTheEndIndecesPtr, const eigen_scalar_array_t* qFinalPtr,
               const state_vector_array_t* QvFinalPtr, const state_matrix_array_t* QmFinalPtr) {
    BASE::resetNumFunctionCalls();

    switchingTimeStart_ = switchingTimeStart;
    switchingTimeFinal_ = switchingTimeFinal;

    if (normalizeTime_) {
      scalingFactor_ = switchingTimeFinal - switchingTimeStart;
    } else {
      scalingFactor_ = 1.0;
    }

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

  /**
   * Riccati jump map at switching moments
   *
   * @param [in] time: Normalized transition time
   * @param [in] state: transition state
   * @param [out] mappedState: mapped state after transition
   */
  void computeJumpMap(const scalar_t& z, const s_vector_t& state, s_vector_t& mappedState) override {
    scalar_t time = switchingTimeFinal_ - scalingFactor_ * z;

    // epsilon is set to include times past event times which have been artificially increased in the rollout
    size_t index = Lookup::findFirstIndexWithinTol(eventTimes_, time, 1e-5);

    s_vector_t allSsJump;
    convert2Vector((*QmFinalPtr_)[index], (*QvFinalPtr_)[index], (*qFinalPtr_)[index], allSsJump);

    mappedState = state + allSsJump;
  }

  /**
   * Computes derivatives.
   *
   * @param [in] z: Normalized time.
   * @param [in] allSs: Single vector constructed by concatenating Sm, Sv and s.
   * @param [out] derivatives: d(allSs)/dz.
   */
  void computeFlowMap(const scalar_t& z, const s_vector_t& allSs, s_vector_t& derivatives) override {
    /*note: according to some discussions on stackoverflow, it does not buy computation time if multiplications
     * with symmetric matrices are executed using selfadjointView(). Doing the full multiplication seems to be faster
     * because of vectorization
     */
    /*
     *  Expressions written base on guidelines in http://eigen.tuxfamily.org/dox/TopicWritingEfficientProductExpression.html
     */
    BASE::numFunctionCalls_++;

    // denormalized time
    const scalar_t t = switchingTimeFinal_ - scalingFactor_ * z;

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

    if (!useMakePSD_) {
      Qm_.diagonal().array() += addedRiccatiDiagonal_;
    }

    Qm_ *= scalingFactor_;
    Qv_ *= scalingFactor_;
    q_ *= scalingFactor_;
    convert2Vector(Qm_, Qv_, q_, derivatives);
  }

 private:
  bool useMakePSD_;
  bool normalizeTime_;
  bool preComputeRiccatiTerms_;
  scalar_t addedRiccatiDiagonal_;
  scalar_t switchingTimeStart_;
  scalar_t switchingTimeFinal_;
  scalar_t scalingFactor_;

  // Interpolation
  EigenLinearInterpolation<state_matrix_t> QmFunc_;
  EigenLinearInterpolation<state_vector_t> QvFunc_;
  EigenLinearInterpolation<eigen_scalar_t> qFunc_;
  EigenLinearInterpolation<dynamic_matrix_t> RinvChol_Func_;
  EigenLinearInterpolation<input_state_matrix_t> PmFunc_;
  EigenLinearInterpolation<input_vector_t> RvFunc_;
  EigenLinearInterpolation<state_matrix_t> AmFunc_;
  EigenLinearInterpolation<state_input_matrix_t> BmFunc_;

  // Interpolation of precomputation
  EigenLinearInterpolation<state_matrix_t> Qm_minus_P_Rinv_P_func_;
  EigenLinearInterpolation<state_vector_t> Qv_minus_P_Rinv_Rv_func_;
  EigenLinearInterpolation<eigen_scalar_t> q_minus_half_Rv_Rinv_Rv_func_;
  EigenLinearInterpolation<state_matrix_t> AmT_minus_P_Rinv_B_func_;
  EigenLinearInterpolation<dynamic_matrix_t> B_RinvChol_func_;
  EigenLinearInterpolation<dynamic_vector_t> RinvCholT_Rv_func_;

  // Arrays to store precomputation
  state_matrix_array_t Qm_minus_P_Rinv_P_array_;
  state_vector_array_t Qv_minus_P_Rinv_Rv_array_;
  eigen_scalar_array_t q_minus_half_Rv_Rinv_Rv_array_;
  state_matrix_array_t AmT_minus_P_Rinv_B_array_;
  dynamic_matrix_array_t B_RinvChol_array_;
  dynamic_vector_array_t RinvCholT_Rv_array_;

  // members required only in computeFlowMap()
  state_matrix_t Sm_;
  state_vector_t Sv_;
  eigen_scalar_t s_;
  state_matrix_t Qm_;
  state_vector_t Qv_;
  eigen_scalar_t q_;
  state_matrix_t AmT_minus_P_Rinv_Bm_;
  dynamic_matrix_t B_RinvChol_;
  dynamic_vector_t RinvCholT_Rv_;
  dynamic_matrix_t SmT_B_RinvChol_;
  state_matrix_t AmT_Sm_;
  state_matrix_t Am_;
  state_input_matrix_t Bm_;
  input_vector_t Rv_;
  dynamic_matrix_t RinvChol_;
  input_state_matrix_t Pm_;

  scalar_array_t eventTimes_;
  const eigen_scalar_array_t* qFinalPtr_;
  const state_vector_array_t* QvFinalPtr_;
  const state_matrix_array_t* QmFinalPtr_;
};

}  // namespace ocs2

#endif /* SEQUENTIALRICCATIEQUATIONSNORMALIZED_OCS2_H_ */
