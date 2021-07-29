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

#include <array>
#include <limits>

#include <ocs2_core/Dimensions.h>
#include <ocs2_core/integration/OdeBase.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_core/misc/Numerics.h>

namespace ocs2 {

/**
 * Sensitivity sequential Riccati equations class
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class SensitivitySequentialRiccatiEquations final : public OdeBase<STATE_DIM*(STATE_DIM + 1) / 2 + STATE_DIM + 1> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum {
    /** If STATE_DIM=n, Then: n(n+1)/2 entries from triangular matrix Sm, n entries from vector Sv and +1 one from a scalar */
    S_DIM_ = STATE_DIM * (STATE_DIM + 1) / 2 + STATE_DIM + 1
  };

  using BASE = OdeBase<S_DIM_>;

  using DIMENSIONS = Dimensions<STATE_DIM, INPUT_DIM>;
  using scalar_t = typename DIMENSIONS::scalar_t;
  using scalar_array_t = typename DIMENSIONS::scalar_array_t;
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

  using s_vector_t = Eigen::Matrix<scalar_t, S_DIM_, 1>;
  using s_vector_array_t = std::vector<s_vector_t, Eigen::aligned_allocator<s_vector_t> >;

  /**
   * Default constructor.
   */
  SensitivitySequentialRiccatiEquations() = default;

  /**
   * Default destructor.
   */
  ~SensitivitySequentialRiccatiEquations() override = default;

  /**
   * Returns pointer to the class.
   *
   * @return A raw pointer to the class.
   */
  SensitivitySequentialRiccatiEquations<STATE_DIM, INPUT_DIM>* clone() const {
    return new SensitivitySequentialRiccatiEquations<STATE_DIM, INPUT_DIM>(*this);
  }

  /**
   * Transcribe symmetric matrix nabla_Sm, vector nabla_Sv and
   * scalar nabla_s into a single vector.
   *
   * @param [in] nabla_Sm: \f$ \partial S_m \f$
   * @param [in] nabla_Sv: \f$ \partial S_v \f$
   * @param [in] nabla_s: \f$ \partial s \f$
   * @param [out] allSs: Single vector constructed by concatenating
   * nabla_Sm, nabla_Sv and nabla_s.
   */
  static void convert2Vector(const state_matrix_t& nabla_Sm, const state_vector_t& nabla_Sv, const scalar_t& nabla_s, s_vector_t& allSs) {
    /* nabla_Sm is symmetric. Here, we only extract the upper triangular part and transcribe it in column-wise fashion into allSs*/
    size_t count = 0;  // count the total number of scalar entries covered
    size_t nRows = 0;
    for (size_t nCols = 0; nCols < STATE_DIM; nCols++) {
      nRows = nCols + 1;
      allSs.segment(count, nRows) << Eigen::Map<const dynamic_vector_t>(nabla_Sm.data() + nCols * STATE_DIM, nRows);
      count += nRows;
    }

    /* add data from nabla_Sv on top*/
    allSs.template segment<STATE_DIM>((STATE_DIM * (STATE_DIM + 1)) / 2) << Eigen::Map<const dynamic_vector_t>(nabla_Sv.data(), STATE_DIM);

    /* add nabla_s as last element*/
    allSs.template tail<1>() << nabla_s;
  }

  /**
   * Transcribes the stacked vector allSs into a symmetric matrix, nabla_Sm,
   * a vector, nabla_Sv and a single scalar, nabla_s.
   *
   * @param [in] allSs: Single vector constructed by concatenating nabla_Sm,
   * nabla_Sv and nabla_s.
   * @param [out] nabla_Sm: \f$ \partial S_m \f$
   * @param [out] nabla_Sv: \f$ \partial S_v \f$
   * @param [out] nabla_s: \f$ \partial s \f$
   */
  static void convert2Matrix(const s_vector_t& allSs, state_matrix_t& nabla_Sm, state_vector_t& nabla_Sv, scalar_t& nabla_s) {
    /* Sm is symmetric. Here, we map the first entries from allSs onto the respective elements in the symmetric matrix*/
    size_t count = 0;
    size_t nCols = 0;
    for (size_t rows = 0; rows < STATE_DIM; rows++) {
      nCols = rows + 1;
      nabla_Sm.block(rows, 0, 1, nCols) << Eigen::Map<const dynamic_vector_t>(allSs.data() + count, nCols).transpose();
      // "nCols-1" because diagonal elements have already been covered
      nabla_Sm.block(0, rows, nCols - 1, 1) << Eigen::Map<const dynamic_vector_t>(allSs.data() + count, nCols - 1);
      count += nCols;
    }

    /* extract the vector Sv*/
    nabla_Sv = Eigen::Map<const dynamic_vector_t>(allSs.data() + (STATE_DIM * (STATE_DIM + 1)) / 2, STATE_DIM);

    /* extract s as the last element */
    nabla_s = allSs.template tail<1>()(0);
  }

  /**
   * Sets data
   */
  void setData(const scalar_t& learningRate, const scalar_array_t* SsTimePtr, const state_matrix_array_t* SmPtr,
               const state_vector_array_t* SvPtr, const scalar_array_t* timeStampPtr, const ModelData::array_t* modelDataPtr,
               const dynamic_matrix_array_t* RmInversePtr, const scalar_array_t* nablaqPtr, const state_vector_array_t* nablaQvPtr,
               const input_vector_array_t* nablaRvPtr) {
    alpha_ = learningRate;

    SsTimePtr_ = SsTimePtr;
    SmPtr_ = SmPtr;
    SvPtr_ = SvPtr;

    timeStampPtr_ = timeStampPtr;
    modelDataPtr_ = modelDataPtr;
    RmInversePtr_ = RmInversePtr;
    nablaqPtr_ = nablaqPtr;
    nablaQvPtr_ = nablaQvPtr;
    nablaRvPtr_ = nablaRvPtr;
  }

  /**
   * Sets the multiplier of exogenous part of the equation. It is either zero
   * or plus-minus 1/(s_{i+1}-s_{i})
   *
   * @param [in] multiplier: the multiplier of exogenous part of the equation.
   */
  void setMultiplier(const scalar_t& multiplier) { multiplier_ = multiplier; }

  /**
   * Computes derivative
   * @param [in] z
   * @param [in] allSs
   * @param [out] derivatives
   */
  void computeFlowMap(const scalar_t& z, const s_vector_t& allSs, s_vector_t& derivatives) override {
    // denormalized time
    const scalar_t t = -z;

    convert2Matrix(allSs, nabla_Sm_, nabla_Sv_, nabla_s_);

    auto indexAlpha = LinearInterpolation::timeSegment(t, SsTimePtr_);
    LinearInterpolation::interpolate(indexAlpha, Sv_, SvPtr_);
    LinearInterpolation::interpolate(indexAlpha, Sm_, SmPtr_);

    indexAlpha = LinearInterpolation::timeSegment(t, timeStampPtr_);
    ModelData::interpolate(indexAlpha, Am_, modelDataPtr_, ModelData::dynamicsStateDerivative);
    ModelData::interpolate(indexAlpha, Bm_, modelDataPtr_, ModelData::dynamicsInputDerivative);
    ModelData::interpolate(indexAlpha, q_, modelDataPtr_, ModelData::cost);
    ModelData::interpolate(indexAlpha, Qv_, modelDataPtr_, ModelData::costStateDerivative);
    ModelData::interpolate(indexAlpha, Rv_, modelDataPtr_, ModelData::costInputDerivative);
    ModelData::interpolate(indexAlpha, Qm_, modelDataPtr_, ModelData::costStateSecondDerivative);
    ModelData::interpolate(indexAlpha, Rm_, modelDataPtr_, ModelData::costInputSecondDerivative);
    ModelData::interpolate(indexAlpha, Pm_, modelDataPtr_, ModelData::costInputStateDerivative);
    LinearInterpolation::interpolate(indexAlpha, invRm_, RmInversePtr_);
    LinearInterpolation::interpolate(indexAlpha, nabla_q_, nablaqPtr_);
    LinearInterpolation::interpolate(indexAlpha, nabla_Qv_, nablaQvPtr_);
    LinearInterpolation::interpolate(indexAlpha, nabla_Rv_, nablaRvPtr_);

    Lv_ = invRm_ * (Rv_ + Bm_.transpose() * Sv_);
    Lm_ = invRm_ * (Pm_ + Bm_.transpose() * Sm_);

    nabla_Lv_ = invRm_ * (nabla_Rv_ + Bm_.transpose() * nabla_Sv_);
    nabla_Lm_ = invRm_ * Bm_.transpose() * nabla_Sm_;

    // Riccati equations
    if (!numerics::almost_eq(multiplier_, 0.0)) {
      dSmdt_ = Qm_ + Am_.transpose() * Sm_ + Sm_.transpose() * Am_ - Lm_.transpose() * Rm_ * Lm_;
      dSmdt_ = 0.5 * (dSmdt_ + dSmdt_.transpose()).eval();
      dSvdt_ = Qv_ + Am_.transpose() * Sv_ - Lm_.transpose() * Rm_ * Lv_;
      dsdt_ = q_ - 0.5 * alpha_ * (2.0 - alpha_) * Lv_.dot(Rm_ * Lv_);

    } else {
      dSmdt_.setZero();
      dSvdt_.setZero();
      dsdt_ = 0.0;
    }

    // derivatives of Riccati equations
    nabla_dSmdt_ =
        Am_.transpose() * nabla_Sm_ + nabla_Sm_.transpose() * Am_ - nabla_Lm_.transpose() * Rm_ * Lm_ - Lm_.transpose() * Rm_ * nabla_Lm_;
    nabla_dSmdt_ = 0.5 * (nabla_dSmdt_ + nabla_dSmdt_.transpose()).eval();
    nabla_dSvdt_ = nabla_Qv_ + Am_.transpose() * nabla_Sv_ - nabla_Lm_.transpose() * Rm_ * Lv_ - Lm_.transpose() * Rm_ * nabla_Lv_;
    nabla_dsdt_ = nabla_q_ - 0.5 * alpha_ * (2.0 - alpha_) * (nabla_Lv_.dot(Rm_ * Lv_) + Lv_.dot(Rm_ * nabla_Lv_));

    // switching time gradient for the equivalent system
    nabla_dSmdz_ = nabla_dSmdt_ + multiplier_ * dSmdt_;
    nabla_dSvdz_ = nabla_dSvdt_ + multiplier_ * dSvdt_;
    nabla_dsdz_ = nabla_dsdt_ + multiplier_ * dsdt_;

    convert2Vector(nabla_dSmdz_, nabla_dSvdz_, nabla_dsdz_, derivatives);
  }

 private:
  scalar_t alpha_ = 0.0;
  scalar_t multiplier_ = 0.0;

  const scalar_array_t* SsTimePtr_;
  const state_matrix_array_t* SmPtr_;
  const state_vector_array_t* SvPtr_;
  const scalar_array_t* timeStampPtr_;
  const ModelData::array_t* modelDataPtr_;
  const dynamic_matrix_array_t* RmInversePtr_;
  const scalar_array_t* nablaqPtr_;
  const state_vector_array_t* nablaQvPtr_;
  const input_vector_array_t* nablaRvPtr_;

  state_matrix_t nabla_Sm_;
  state_vector_t nabla_Sv_;
  scalar_t nabla_s_;
  state_vector_t Sv_;
  state_matrix_t Sm_;
  dynamic_matrix_t Am_;
  dynamic_matrix_t Bm_;
  scalar_t q_;
  dynamic_vector_t Qv_;
  dynamic_vector_t Rv_;
  dynamic_matrix_t Qm_;
  dynamic_matrix_t Rm_;
  dynamic_matrix_t Pm_;
  dynamic_matrix_t invRm_;
  scalar_t nabla_q_;
  state_vector_t nabla_Qv_;
  input_vector_t nabla_Rv_;

  input_vector_t Lv_;
  input_state_matrix_t Lm_;
  input_vector_t nabla_Lv_;
  input_state_matrix_t nabla_Lm_;

  scalar_t dsdt_;
  state_vector_t dSvdt_;
  state_matrix_t dSmdt_;

  // normalized derivatives of Riccati equations
  scalar_t nabla_dsdz_;
  state_vector_t nabla_dSvdz_;
  state_matrix_t nabla_dSmdz_;

  // derivatives of Riccati equations
  scalar_t nabla_dsdt_;
  state_vector_t nabla_dSvdt_;
  state_matrix_t nabla_dSmdt_;
};

}  // namespace ocs2
