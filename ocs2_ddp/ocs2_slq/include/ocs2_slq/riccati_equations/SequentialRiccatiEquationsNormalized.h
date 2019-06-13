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

#include <Eigen/StdVector>
#include <vector>
#include <Eigen/Dense>
#include <iostream>

#include <ocs2_core/Dimensions.h>
#include <ocs2_core/integration/ODE_Base.h>
#include <ocs2_core/misc/LinearInterpolation.h>

namespace ocs2 {

/**
 * This class implements the time-normalized Riccati equations for SLQ problem.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template<size_t STATE_DIM, size_t INPUT_DIM>
class SequentialRiccatiEquationsNormalized final : public ODE_Base<STATE_DIM * (STATE_DIM + 1) / 2 + STATE_DIM + 1> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum {
    /** If STATE_DIM=n, Then: n(n+1)/2 entries from triangular matrix Sm, n entries from vector Sv and +1 one from a scalar */
        S_DIM_ = STATE_DIM * (STATE_DIM + 1) / 2 + STATE_DIM + 1
  };

  typedef ODE_Base <S_DIM_> BASE;

  typedef Dimensions <STATE_DIM, INPUT_DIM> DIMENSIONS;
  typedef typename DIMENSIONS::scalar_t scalar_t;
  typedef typename DIMENSIONS::scalar_array_t scalar_array_t;
  typedef typename DIMENSIONS::size_array_t size_array_t;
  typedef typename DIMENSIONS::eigen_scalar_t eigen_scalar_t;
  typedef typename DIMENSIONS::eigen_scalar_array_t eigen_scalar_array_t;
  typedef typename DIMENSIONS::state_vector_t state_vector_t;
  typedef typename DIMENSIONS::state_vector_array_t state_vector_array_t;
  typedef typename DIMENSIONS::input_vector_t input_vector_t;
  typedef typename DIMENSIONS::input_vector_array_t input_vector_array_t;
  typedef typename DIMENSIONS::input_state_matrix_t input_state_matrix_t;
  typedef typename DIMENSIONS::input_state_matrix_array_t input_state_matrix_array_t;
  typedef typename DIMENSIONS::state_matrix_t state_matrix_t;
  typedef typename DIMENSIONS::state_matrix_array_t state_matrix_array_t;
  typedef typename DIMENSIONS::input_matrix_t input_matrix_t;
  typedef typename DIMENSIONS::input_matrix_array_t input_matrix_array_t;
  typedef typename DIMENSIONS::state_input_matrix_t state_input_matrix_t;
  typedef typename DIMENSIONS::state_input_matrix_array_t state_input_matrix_array_t;
  typedef typename DIMENSIONS::dynamic_vector_t dynamic_vector_t;
  typedef typename DIMENSIONS::dynamic_matrix_t dynamic_matrix_t;
  typedef typename DIMENSIONS::dynamic_matrix_array_t dynamic_matrix_array_t;

  typedef Eigen::Matrix<scalar_t, S_DIM_, 1> s_vector_t;
  typedef std::vector<s_vector_t, Eigen::aligned_allocator<s_vector_t> > s_vector_array_t;

  /**
   * Constructor.
   */
  SequentialRiccatiEquationsNormalized(
      const bool &useMakePSD,
      const scalar_t &addedRiccatiDiagonal,
      bool normalizeTime,
      bool preComputeRiccatiTerms = true)
      : useMakePSD_(useMakePSD)
      , addedRiccatiDiagonal_(addedRiccatiDiagonal)
      , switchingTimeStart_(0.0)
      , switchingTimeFinal_(1.0)
      , scalingFactor_(1.0)
      , normalizeTime_(normalizeTime)
      , preComputeRiccatiTerms_(preComputeRiccatiTerms)
      , Sm_(state_matrix_t::Zero())
      , Sv_(state_vector_t::Zero())
      , s_(eigen_scalar_t::Zero())
      , q_(eigen_scalar_t::Zero())
      , Qv_(state_vector_t::Zero())
      , Qm_(state_matrix_t::Zero())
      , AmT_minus_P_Rinv_R_Rinv_Bm_(state_matrix_t::Zero())
      , B_Rinv_R_Rinv_Rv_(state_vector_t::Zero())
      , Am_transposeSm_(state_matrix_t::Zero())
      , Am_(state_matrix_t::Zero())
      , Bm_(state_input_matrix_t::Zero())
      , Rv_(input_vector_t::Zero())
      , RmInv_(input_matrix_t::Zero())
      , Rm_(input_matrix_t::Zero())
      , Pm_(input_state_matrix_t::Zero())
      , Lm_(input_state_matrix_t::Zero())
      , Lv_(input_vector_t::Zero())
      , Lm_transposeRm_(state_input_matrix_t::Zero()) {}

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
  static void convert2Vector(
      const state_matrix_t &Sm,
      const state_vector_t &Sv,
      const eigen_scalar_t &s,
      s_vector_t &allSs) {

    /* Sm is symmetric. Here, we only extract the upper triangular part and transcribe it in column-wise fashion into allSs*/
    size_t count = 0;    // count the total number of scalar entries covered
    size_t nRows = 0;
    for (size_t nCols = 0; nCols < STATE_DIM; nCols++) {
      nRows = nCols + 1;
      allSs.segment(count, nRows) <<
                                  Eigen::Map<const dynamic_vector_t>(Sm.data() + nCols * STATE_DIM, nRows);
      count += nRows;
    }

    /* add data from Sv on top*/
    allSs.template segment<STATE_DIM>((STATE_DIM * (STATE_DIM + 1)) / 2) <<
                                                                         Eigen::Map<const dynamic_vector_t>(Sv.data(),
                                                                                                            STATE_DIM);

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
  static void convert2Matrix(
      const s_vector_t &allSs,
      state_matrix_t &Sm,
      state_vector_t &Sv,
      eigen_scalar_t &s) {

    /* Sm is symmetric. Here, we map the first entries from allSs onto the respective elements in the symmetric matrix*/
    size_t count = 0;
    size_t nCols = 0;
    for (size_t rows = 0; rows < STATE_DIM; rows++) {
      nCols = rows + 1;
      Sm.block(rows, 0, 1, nCols) <<
                                  Eigen::Map<const dynamic_vector_t>(allSs.data() + count, nCols).transpose();
      // "nCols-1" because diagonal elements have already been covered
      Sm.block(0, rows, nCols - 1, 1) <<
                                      Eigen::Map<const dynamic_vector_t>(allSs.data() + count, nCols - 1);
      count += nCols;
    }

    /* extract the vector Sv*/
    Sv = Eigen::Map<const dynamic_vector_t>(
        allSs.data() + (STATE_DIM * (STATE_DIM + 1)) / 2, STATE_DIM);

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
  void setData(
      const scalar_t &switchingTimeStart,
      const scalar_t &switchingTimeFinal,
      const scalar_array_t *timeStampPtr,
      const state_matrix_array_t *AmPtr,
      const state_input_matrix_array_t *BmPtr,
      const eigen_scalar_array_t *qPtr,
      const state_vector_array_t *QvPtr,
      const state_matrix_array_t *QmPtr,
      const input_vector_array_t *RvPtr,
      const input_matrix_array_t *RmInversePtr,
      const input_matrix_array_t *RmPtr,
      const dynamic_matrix_array_t *RmCholPtr,
      const input_state_matrix_array_t *PmPtr,
      const size_array_t *eventsPastTheEndIndecesPtr,
      const eigen_scalar_array_t *qFinalPtr,
      const state_vector_array_t *QvFinalPtr,
      const state_matrix_array_t *QmFianlPtr) {

    BASE::resetNumFunctionCalls();

    switchingTimeStart_ = switchingTimeStart;
    switchingTimeFinal_ = switchingTimeFinal;

    if (normalizeTime_){
      scalingFactor_ = switchingTimeFinal - switchingTimeStart;
    } else {
      scalingFactor_ = 1.0;
    }

    eventTimes_.clear();
    eventTimes_.reserve(eventsPastTheEndIndecesPtr->size());

    for (const size_t &pastTheEndIndex : *eventsPastTheEndIndecesPtr)
      eventTimes_.push_back(timeStampPtr->at(pastTheEndIndex - 1));

    qFinalPtr_ = qFinalPtr;
    QvFinalPtr_ = QvFinalPtr;
    QmFianlPtr_ = QmFianlPtr;

    if (preComputeRiccatiTerms_) {
      Qm_minus_P_Rinv_R_Rinv_P_array_.resize(RmPtr->size());
      AmT_minus_P_Rinv_R_Rinv_B_array_.resize(RmPtr->size());
      B_Rinv_R_Rinv_Bm_LinvT_array_.resize(RmPtr->size());
      Qv_minus_P_Rinv_R_Rinv_Rv_array_.resize(RmPtr->size());
      q_minus_half_Rv_Rinv_R_Rinv_Rv_array_.resize(RmPtr->size());
      B_Rinv_R_Rinv_Rv_array_.resize(RmPtr->size());

      dynamic_matrix_t P_R_Chol, B_R_Chol;
      dynamic_vector_t R_Chol_Rv;
      for (size_t i = 0; i < RmPtr->size(); i++) {
        // Temporary terms
        P_R_Chol.noalias() = (*PmPtr)[i].transpose() * (*RmCholPtr)[i];
        B_R_Chol.noalias() = (*BmPtr)[i] * (*RmCholPtr)[i];
        R_Chol_Rv.noalias() = (*RmCholPtr)[i].transpose() * (*RvPtr)[i];

        // Precomputed terms
        Qm_minus_P_Rinv_R_Rinv_P_array_[i] = (*QmPtr)[i];
        Qm_minus_P_Rinv_R_Rinv_P_array_[i].noalias() -= P_R_Chol * P_R_Chol.transpose();

        AmT_minus_P_Rinv_R_Rinv_B_array_[i] = (*AmPtr)[i].transpose();
        AmT_minus_P_Rinv_R_Rinv_B_array_[i].noalias() -= P_R_Chol * B_R_Chol.transpose();

        B_Rinv_R_Rinv_Bm_LinvT_array_[i] = B_R_Chol;

        Qv_minus_P_Rinv_R_Rinv_Rv_array_[i] = (*QvPtr)[i];
        Qv_minus_P_Rinv_R_Rinv_Rv_array_[i].noalias() -= P_R_Chol * R_Chol_Rv;

        q_minus_half_Rv_Rinv_R_Rinv_Rv_array_[i] = (*qPtr)[i];
        q_minus_half_Rv_Rinv_R_Rinv_Rv_array_[i].noalias() -= 0.5 * R_Chol_Rv.transpose() * R_Chol_Rv;

        B_Rinv_R_Rinv_Rv_array_[i] = B_R_Chol * R_Chol_Rv;
      }

      Qm_minus_P_Rinv_R_Rinv_P_func_.setData(timeStampPtr, &Qm_minus_P_Rinv_R_Rinv_P_array_);
      AmT_minus_P_Rinv_R_Rinv_B_func_.setData(timeStampPtr, &AmT_minus_P_Rinv_R_Rinv_B_array_);
      B_Rinv_R_Rinv_B_LinvT_func_.setData(timeStampPtr, &B_Rinv_R_Rinv_Bm_LinvT_array_);
      Qv_minus_P_Rinv_R_Rinv_Rv_func_.setData(timeStampPtr, &Qv_minus_P_Rinv_R_Rinv_Rv_array_);
      q_minus_half_Rv_Rinv_R_Rinv_Rv_func_.setData(timeStampPtr, &q_minus_half_Rv_Rinv_R_Rinv_Rv_array_);
      B_Rinv_R_Rinv_Rv_func_.setData(timeStampPtr, &B_Rinv_R_Rinv_Rv_array_);
    } else {
      AmFunc_.setData(timeStampPtr, AmPtr);
      BmFunc_.setData(timeStampPtr, BmPtr);

      qFunc_.setData(timeStampPtr, qPtr);
      QvFunc_.setData(timeStampPtr, QvPtr);
      QmFunc_.setData(timeStampPtr, QmPtr);
      RvFunc_.setData(timeStampPtr, RvPtr);
      RmInverseFunc_.setData(timeStampPtr, RmInversePtr);
      RmFunc_.setData(timeStampPtr, RmPtr);
      Rm_LinvTFunc_.setData(timeStampPtr, RmCholPtr);
      PmFunc_.setData(timeStampPtr, PmPtr);
    }
  }

  /**
   * Reset the Riccati equation
   */
  void reset() {
    // TODO (Ruben) remove this function and its callers
  }

  /**
   * Riccati jump map at switching moments
   *
   * @param [in] time: Normalized transition time
   * @param [in] state: transition state
   * @param [out] mappedState: mapped state after transition
   */
  void computeJumpMap(
      const scalar_t &z,
      const s_vector_t &state,
      s_vector_t &mappedState) override {

    scalar_t time = switchingTimeFinal_ - scalingFactor_ * z;

    size_t index = find(eventTimes_, time);

    if (index == eventTimes_.size())
      throw std::runtime_error("The Riccati state jump time is not defined.");

    s_vector_t allSsJump;
    convert2Vector(QmFianlPtr_->at(index), QvFinalPtr_->at(index), qFinalPtr_->at(index), allSsJump);

    mappedState = state + allSsJump;
  }

  /**
   * Computes derivatives.
   *
   * @param [in] z: Normalized time.
   * @param [in] allSs: Single vector constructed by concatenating Sm, Sv and s.
   * @param [out] derivatives: d(allSs)/dz.
   */
  void computeFlowMap(
      const scalar_t &z,
      const s_vector_t &allSs,
      s_vector_t &derivatives) override {
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
      bool hasNegativeEigenValue = makePSD(Sm_);
    }

    if (preComputeRiccatiTerms_) {
      const auto greatestLessTimeStampIndex = Qm_minus_P_Rinv_R_Rinv_P_func_.interpolate(t, Qm_);
      Qv_minus_P_Rinv_R_Rinv_Rv_func_.interpolate(t, Qv_, greatestLessTimeStampIndex);
      q_minus_half_Rv_Rinv_R_Rinv_Rv_func_.interpolate(t, q_, greatestLessTimeStampIndex);
      AmT_minus_P_Rinv_R_Rinv_B_func_.interpolate(t, AmT_minus_P_Rinv_R_Rinv_Bm_, greatestLessTimeStampIndex);
      B_Rinv_R_Rinv_Rv_func_.interpolate(t, B_Rinv_R_Rinv_Rv_, greatestLessTimeStampIndex);
      B_Rinv_R_Rinv_B_LinvT_func_.interpolate(t, B_Rinv_R_Rinv_Bm_LinvT_, greatestLessTimeStampIndex);

      Am_transposeSm_.noalias() = AmT_minus_P_Rinv_R_Rinv_Bm_ * Sm_;

      // dSmdt,  Qm_ used instead of temporary
      Qm_ += Am_transposeSm_ + Am_transposeSm_.transpose();
      dynamic_matrix_t Sm_T_B_Rinv_R_Rinv_Bm_LinvT_ = Sm_.transpose() * B_Rinv_R_Rinv_Bm_LinvT_;
      Qm_.noalias() -= Sm_T_B_Rinv_R_Rinv_Bm_LinvT_ * Sm_T_B_Rinv_R_Rinv_Bm_LinvT_.transpose();

      // dSvdt,  Qv_ used instead of temporary
      Qv_.noalias() += AmT_minus_P_Rinv_R_Rinv_Bm_ * Sv_;
      dynamic_vector_t B_Rinv_R_Rinv_Bm_Sv_ = B_Rinv_R_Rinv_Bm_LinvT_ * (B_Rinv_R_Rinv_Bm_LinvT_.transpose() * Sv_);
      Qv_.noalias() -= Sm_.transpose() * (B_Rinv_R_Rinv_Rv_ + B_Rinv_R_Rinv_Bm_Sv_);

      // dsdt,   q_ used instead of temporary
      q_.noalias() -= Sv_.transpose() * B_Rinv_R_Rinv_Rv_;
      q_.noalias() -= 0.5 * Sv_.transpose() * B_Rinv_R_Rinv_Bm_Sv_;
    } else {
      const auto greatestLessTimeStampIndex = qFunc_.interpolate(t, q_);
      QvFunc_.interpolate(t, Qv_, greatestLessTimeStampIndex);
      QmFunc_.interpolate(t, Qm_, greatestLessTimeStampIndex);
      AmFunc_.interpolate(t, Am_, greatestLessTimeStampIndex);
      BmFunc_.interpolate(t, Bm_, greatestLessTimeStampIndex);
      RvFunc_.interpolate(t, Rv_, greatestLessTimeStampIndex);
      RmInverseFunc_.interpolate(t, RmInv_, greatestLessTimeStampIndex);
      RmFunc_.interpolate(t, Rm_, greatestLessTimeStampIndex);
      PmFunc_.interpolate(t, Pm_, greatestLessTimeStampIndex);

      Pm_.noalias() += Bm_.transpose() * Sm_; // ! Pm is changed to avoid an extra temporary
      Lm_.noalias() = RmInv_ * Pm_;
      Rv_.noalias() += Bm_.transpose() * Sv_; // ! Rv is changed to avoid an extra temporary
      Lv_.noalias() = RmInv_ * Rv_;

      Am_transposeSm_.noalias() = Am_.transpose() * Sm_.transpose();
      Lm_transposeRm_.noalias() = Lm_.transpose() * Rm_.transpose();

      // dSmdt,  Qm_ used instead of temporary
      Qm_ += Am_transposeSm_ + Am_transposeSm_.transpose();
      Qm_.noalias() -= Lm_transposeRm_ * Lm_;

      // dSvdt,  Qv_ used instead of temporary
      Qv_.noalias() += Am_.transpose() * Sv_;
      Qv_.noalias() -= Lm_transposeRm_ * Lv_;

      // dsdt,   q_ used instead of temporary
      q_.noalias() -= 0.5 * Lv_.transpose() * Rm_ * Lv_;
    }

    if (!useMakePSD_) {
      Qm_.diagonal().array() += addedRiccatiDiagonal_;
    }

    Qm_ *= scalingFactor_;
    Qv_ *= scalingFactor_;
    q_ *= scalingFactor_;
    convert2Vector(Qm_, Qv_, q_, derivatives);
  }

 protected:
  /**
   * Makes the matrix PSD.
   * @tparam Derived type.
   * @param [out] squareMatrix: The matrix to become PSD.
   * @return boolean
   */
  template<typename Derived>
  static bool makePSD(Eigen::MatrixBase<Derived> &squareMatrix) {

    if (squareMatrix.rows() != squareMatrix.cols())
      throw std::runtime_error("Not a square matrix: makePSD() method is for square matrix.");

    Eigen::SelfAdjointEigenSolver<Derived> eig(squareMatrix, Eigen::EigenvaluesOnly);
    Eigen::VectorXd lambda = eig.eigenvalues();

    bool hasNegativeEigenValue = false;
    for (size_t j = 0; j < lambda.size(); j++)
      if (lambda(j) < 0.0) {
        hasNegativeEigenValue = true;
        lambda(j) = 1e-6;
      }

    if (hasNegativeEigenValue) {
      eig.compute(squareMatrix, Eigen::ComputeEigenvectors);
      squareMatrix = eig.eigenvectors() * lambda.asDiagonal() * eig.eigenvectors().inverse();
    } else {
      squareMatrix = 0.5 * (squareMatrix + squareMatrix.transpose()).eval();
    }

    return hasNegativeEigenValue;
  }

  template<class InputIterator, class T>
  InputIterator find(InputIterator first, InputIterator last, const T &val) {
    while (first != last) {
      if (*first == val) return first;
      ++first;
    }
    return last;
  }

  /**
   * finds the index of an element in dataArray which is equal to value (epsilone distance)
   * @param [in] dataArray: data array
   * @param [in] value: enquiry value
   * @return: index
   */
  size_t find(const std::vector<double> &dataArray, const double &value) {

    size_t index = dataArray.size();

    for (size_t i = 0; i < dataArray.size(); i++)
      if (std::abs(dataArray[i] - value) < 1e-5) {
        index = i;
        break;
      }

    return index;
  }

 private:
  bool useMakePSD_;
  scalar_t addedRiccatiDiagonal_;
  scalar_t switchingTimeStart_;
  scalar_t switchingTimeFinal_;
  scalar_t scalingFactor_;
  bool normalizeTime_;
  bool preComputeRiccatiTerms_;

  EigenLinearInterpolation <state_matrix_t> AmFunc_;
  EigenLinearInterpolation <state_input_matrix_t> BmFunc_;
  EigenLinearInterpolation <eigen_scalar_t> qFunc_;
  EigenLinearInterpolation <state_vector_t> QvFunc_;
  EigenLinearInterpolation <state_matrix_t> QmFunc_;
  EigenLinearInterpolation <input_vector_t> RvFunc_;
  EigenLinearInterpolation <input_matrix_t> RmInverseFunc_;
  EigenLinearInterpolation <input_matrix_t> RmFunc_;
  EigenLinearInterpolation <dynamic_matrix_t> Rm_LinvTFunc_;
  EigenLinearInterpolation <input_state_matrix_t> PmFunc_;

  state_matrix_array_t Qm_minus_P_Rinv_R_Rinv_P_array_;
  state_matrix_array_t AmT_minus_P_Rinv_R_Rinv_B_array_;
  dynamic_matrix_array_t B_Rinv_R_Rinv_Bm_LinvT_array_;
  state_vector_array_t Qv_minus_P_Rinv_R_Rinv_Rv_array_;
  state_vector_array_t B_Rinv_R_Rinv_Rv_array_;
  eigen_scalar_array_t q_minus_half_Rv_Rinv_R_Rinv_Rv_array_;
  EigenLinearInterpolation <state_matrix_t> Qm_minus_P_Rinv_R_Rinv_P_func_;
  EigenLinearInterpolation <state_matrix_t> AmT_minus_P_Rinv_R_Rinv_B_func_;
  EigenLinearInterpolation <dynamic_matrix_t> B_Rinv_R_Rinv_B_LinvT_func_;
  EigenLinearInterpolation <state_vector_t> Qv_minus_P_Rinv_R_Rinv_Rv_func_;
  EigenLinearInterpolation <state_vector_t> B_Rinv_R_Rinv_Rv_func_;
  EigenLinearInterpolation <eigen_scalar_t> q_minus_half_Rv_Rinv_R_Rinv_Rv_func_;

  // members required only in computeFlowMap()
  state_matrix_t Sm_;
  state_vector_t Sv_;
  eigen_scalar_t s_;
  eigen_scalar_t q_;
  state_vector_t Qv_;
  state_matrix_t Qm_;
  state_matrix_t AmT_minus_P_Rinv_R_Rinv_Bm_;
  dynamic_matrix_t B_Rinv_R_Rinv_Bm_LinvT_;
  state_vector_t B_Rinv_R_Rinv_Rv_;
  state_matrix_t Am_transposeSm_;
  state_matrix_t Am_;
  state_input_matrix_t Bm_;
  input_vector_t Rv_;
  input_matrix_t RmInv_;
  input_matrix_t Rm_;
  dynamic_matrix_t Rm_LinvT_;
  input_state_matrix_t Pm_;
  input_state_matrix_t Lm_;
  input_vector_t Lv_;
  state_input_matrix_t Lm_transposeRm_;

  scalar_array_t eventTimes_;
  const eigen_scalar_array_t *qFinalPtr_;
  const state_vector_array_t *QvFinalPtr_;
  const state_matrix_array_t *QmFianlPtr_;
};

}

#endif /* SEQUENTIALRICCATIEQUATIONSNORMALIZED_OCS2_H_ */
