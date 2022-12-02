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

#ifndef BVPEQUATIONS_OCS2_H_
#define BVPEQUATIONS_OCS2_H_

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <vector>

#include <ocs2_core/integration/OdeBase.h>
#include <ocs2_core/misc/LinearAlgebra.h>
#include <ocs2_core/misc/LinearInterpolation.h>

namespace ocs2 {

/**
 * This class contains the general BVP equations.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class BVPEquations : public OdeBase<STATE_DIM * STATE_DIM + STATE_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum {
    /** If STATE_DIM=n, Then: n(n+1)/2 entries from triangular matrix Sm, n entries from vector Sv_ and +1 one from a scalar */
    Full_ODE_VECTOR_DIM = STATE_DIM * STATE_DIM + STATE_DIM
  };

  using scalar_t = double;
  using full_ode_vector_t = Eigen::Matrix<scalar_t, Full_ODE_VECTOR_DIM, 1>;
  using full_ode_vector_array_t = std::vector<full_ode_vector_t, Eigen::aligned_allocator<full_ode_vector_t> >;

  using state_vector_t = Eigen::Matrix<scalar_t, STATE_DIM, 1>;
  using input_vector_t = Eigen::Matrix<scalar_t, INPUT_DIM, 1>;
  using state_state_matrix_t = Eigen::Matrix<scalar_t, STATE_DIM, STATE_DIM>;
  using input_input_matrix_t = Eigen::Matrix<scalar_t, INPUT_DIM, INPUT_DIM>;
  using input_state_matrix_t = Eigen::Matrix<scalar_t, INPUT_DIM, STATE_DIM>;
  using state_input_matrix_t = Eigen::Matrix<scalar_t, STATE_DIM, INPUT_DIM>;

  using scalar_array_t = std::vector<scalar_t>;
  using state_vector_array_t = std::vector<state_vector_t, Eigen::aligned_allocator<state_vector_t> >;
  using input_vector_array_t = std::vector<input_vector_t, Eigen::aligned_allocator<input_vector_t> >;
  using state_state_matrix_array_t = std::vector<state_state_matrix_t, Eigen::aligned_allocator<state_state_matrix_t> >;
  using input_input_matrix_array_t = std::vector<input_input_matrix_t, Eigen::aligned_allocator<input_input_matrix_t> >;
  using input_state_matrix_array_t = std::vector<input_state_matrix_t, Eigen::aligned_allocator<input_state_matrix_t> >;
  using state_input_matrix_array_t = std::vector<state_input_matrix_t, Eigen::aligned_allocator<state_input_matrix_t> >;

  /**
   * Default constructor.
   */
  BVPEquations(const bool& useMakePSD, const scalar_t& addedRiccatiDiagonal)

      : useMakePSD_(useMakePSD), addedRiccatiDiagonal_(addedRiccatiDiagonal) {}

  /**
   * Default destructor.
   */
  ~BVPEquations() = default;

  /**
   * Transcribe symmetric matrix Mm_ and vector Sv_ into a single vector.
   *
   * @param [in] Mm_: \f$ M_m \f$
   * @param [in] Sv_: \f$ S_v \f$
   * @param [out] MSv: Single vector constructed by concatenating Mm_ and Sv_.
   */
  static void convert2Vector(const state_state_matrix_t& Mm_, const state_vector_t& Sv_, full_ode_vector_t& MSv) {
    MSv << Eigen::Map<const Eigen::VectorXd>(Mm_.data(), STATE_DIM * STATE_DIM), Eigen::Map<const Eigen::VectorXd>(Sv_.data(), STATE_DIM);
  }

  /**
   * Transcribes the stacked vector allSs into a symmetric matrix, Mm_ and a vector, Sv_.
   *
   * @param [in] MSv: Single vector constructed by concatenating Mm_ and Sv_.
   * @param [out] Mm_: \f$ M_m \f$
   * @param [out] Sv_: \f$ S_v \f$
   */
  static void convert2Matrix(const full_ode_vector_t& MSv, state_state_matrix_t& Mm_, state_vector_t& Sv_) {
    Mm_ = Eigen::Map<const state_state_matrix_t>(MSv.data(), STATE_DIM, STATE_DIM);
    Sv_ = Eigen::Map<const state_vector_t>(MSv.data() + STATE_DIM * STATE_DIM, STATE_DIM);
  }

  /**
   * Sets coefficients of the model.
   *
   * @param [in] timeStampPtr: A pointer to the time stamp trajectory.
   * @param [in] AmPtr: A pointer to the trajectory of \f$ A_m(t) \f$ .
   * @param [in] OmPtr: A pointer to the trajectory of \f$ O_m(t) \f$ .
   * @param [in] BmPtr: A pointer to the trajectory of \f$ B_m(t) \f$ .
   * @param [in] GvPtr: A pointer to the trajectory of \f$ G_v(t) \f$ .
   * @param [in] QvPtr: A pointer to the trajectory of \f$ Q_v(t) \f$ .
   * @param [in] QmPtr: A pointer to the trajectory of \f$ Q_m(t) \f$ .
   * @param [in] PmPtr: A pointer to the trajectory of \f$ P_m(t) \f$ .
   * @param [in] RvPtr: A pointer to the trajectory of \f$ R_v(t) \f$ .
   * @param [in] RmPtr: A pointer to the trajectory of \f$ R_m(t) \f$ .
   * @param [in] RmInversePtr: A pointer to the trajectory of \f$ R_m^{-1}(t) \f$ .
   */
  void setData(const scalar_array_t* timeStampPtr, const state_state_matrix_array_t* AmPtr, const state_state_matrix_array_t* OmPtr,
               const state_input_matrix_array_t* BmPtr, const state_vector_array_t* GvPtr, const state_vector_array_t* QvPtr,
               const state_state_matrix_array_t* QmPtr, const input_state_matrix_array_t* PmPtr, const input_vector_array_t* RvPtr,
               const input_input_matrix_array_t* RmPtr, const input_input_matrix_array_t* RmInversePtr) {
    AmFunc_.setTimeStamp(timeStampPtr);
    OmFunc_.setTimeStamp(timeStampPtr);
    BmFunc_.setTimeStamp(timeStampPtr);
    GvFunc_.setTimeStamp(timeStampPtr);

    QvFunc_.setTimeStamp(timeStampPtr);
    QmFunc_.setTimeStamp(timeStampPtr);
    PmFunc_.setTimeStamp(timeStampPtr);

    RvFunc_.setTimeStamp(timeStampPtr);
    RmFunc_.setTimeStamp(timeStampPtr);
    RmInverseFunc_.setTimeStamp(timeStampPtr);

    if (AmPtr)
      AmFunc_.setData(AmPtr);
    else
      throw std::runtime_error("Am_ is not set.");
    if (OmPtr)
      OmFunc_.setData(OmPtr);
    else
      OmFunc_.setZero();
    if (BmPtr)
      BmFunc_.setData(BmPtr);
    else
      throw std::runtime_error("Bm_ is not set.");
    if (GvPtr)
      GvFunc_.setData(GvPtr);
    else
      GvFunc_.setZero();

    if (QvPtr)
      QvFunc_.setData(QvPtr);
    else
      QvFunc_.setZero();
    if (QmPtr)
      QmFunc_.setData(QmPtr);
    else
      QmFunc_.setZero();
    if (PmPtr)
      PmFunc_.setData(PmPtr);
    else
      PmFunc_.setZero();

    if (RvPtr)
      RvFunc_.setData(RvPtr);
    else
      RvFunc_.setZero();
    if (RmPtr)
      RmFunc_.setData(RmPtr);
    else
      throw std::runtime_error("Rm_ is not set.");
    if (RmInversePtr)
      RmInverseFunc_.setData(RmInversePtr);
    else
      throw std::runtime_error("RmInverse_ is not set.");

    startTime_ = timeStampPtr->front();
    finalTime_ = timeStampPtr->back();
  }

  /**
   * Computes derivative
   * @param [in] z: Normalized time.
   * @param [in] MSv: Single vector constructed by concatenating Mm_, Sv_.
   * @param [out] derivatives: derivatives: d(MSv)/dz.
   */
  void computeFlowMap(const scalar_t& z, const full_ode_vector_t& MSv, full_ode_vector_t& derivatives) override {
    // denormalized time
    if (z > 1 || z < 0) throw std::runtime_error("The normalized time should be between zero and one.");

    scalar_t t = finalTime_ - (finalTime_ - startTime_) * z;

    convert2Matrix(MSv, Mm_, Sv_);

    // numerical consideration
    if (useMakePSD_ == true)
      LinearAlgebra::makePSD(Mm_);
    else
      Mm_ += state_state_matrix_t::Identity() * (addedRiccatiDiagonal_);

    const auto indexAlpha = AmFunc_.interpolate(t, Am_);
    OmFunc_.interpolate(indexAlpha, Om_);
    BmFunc_.interpolate(indexAlpha, Bm_);
    GvFunc_.interpolate(indexAlpha, Gv_);

    QvFunc_.interpolate(indexAlpha, Qv_);
    QmFunc_.interpolate(indexAlpha, Qm_);
    PmFunc_.interpolate(indexAlpha, Pm_);
    RvFunc_.interpolate(indexAlpha, Rv_);
    RmFunc_.interpolate(indexAlpha, Rm_);
    RmInverseFunc_.interpolate(indexAlpha, RmInverse_);

    // Uv = -Lv_ - Km_*x
    Lv_ = RmInverse_ * (Rv_ + Bm_.transpose() * Sv_);
    Km_ = RmInverse_ * (Pm_ + Bm_.transpose() * Mm_);

    // Riccati equations for the original system
    dMmdt_ = Qm_ + Am_.transpose() * Mm_ + Mm_.transpose() * Am_ + Mm_.transpose() * Om_ * Mm_ - Km_.transpose() * Rm_ * Km_;
    dSvdt_ = (Qv_ + Mm_ * Gv_) + Am_.transpose() * Sv_ + Mm_.transpose() * Om_ * Sv_ - Km_.transpose() * Rm_ * Lv_;

    // Riccati equations for the equivalent system
    dMmdz_ = (finalTime_ - startTime_) * dMmdt_;
    dSvdz_ = (finalTime_ - startTime_) * dSvdt_;

    convert2Vector(dMmdz_, dSvdz_, derivatives);
  }

 private:
  bool useMakePSD_;
  scalar_t addedRiccatiDiagonal_;
  scalar_t startTime_;
  scalar_t finalTime_;

  LinearInterpolation<state_state_matrix_t, Eigen::aligned_allocator<state_state_matrix_t> > AmFunc_;
  LinearInterpolation<state_state_matrix_t, Eigen::aligned_allocator<state_state_matrix_t> > OmFunc_;
  LinearInterpolation<state_input_matrix_t, Eigen::aligned_allocator<state_input_matrix_t> > BmFunc_;
  LinearInterpolation<state_vector_t, Eigen::aligned_allocator<state_vector_t> > GvFunc_;

  LinearInterpolation<state_vector_t, Eigen::aligned_allocator<state_vector_t> > QvFunc_;
  LinearInterpolation<input_vector_t, Eigen::aligned_allocator<input_vector_t> > RvFunc_;

  LinearInterpolation<state_state_matrix_t, Eigen::aligned_allocator<state_state_matrix_t> > QmFunc_;
  LinearInterpolation<input_state_matrix_t, Eigen::aligned_allocator<input_state_matrix_t> > PmFunc_;
  LinearInterpolation<input_input_matrix_t, Eigen::aligned_allocator<input_input_matrix_t> > RmFunc_;
  LinearInterpolation<input_input_matrix_t, Eigen::aligned_allocator<input_input_matrix_t> > RmInverseFunc_;

  state_state_matrix_t Mm_;
  state_vector_t Sv_;
  state_state_matrix_t Am_;
  state_state_matrix_t Om_;
  state_input_matrix_t Bm_;
  state_vector_t Gv_;
  state_vector_t Qv_;
  state_state_matrix_t Qm_;
  input_state_matrix_t Pm_;
  input_vector_t Rv_;
  input_input_matrix_t Rm_;
  input_input_matrix_t RmInverse_;
  state_state_matrix_t dMmdt_;
  state_state_matrix_t dMmdz_;
  state_vector_t dSvdt_;
  state_vector_t dSvdz_;
  input_vector_t Lv_;
  input_state_matrix_t Km_;
};

}  // namespace ocs2

#endif /* BVPEQUATIONS_H_OCS2_ */
