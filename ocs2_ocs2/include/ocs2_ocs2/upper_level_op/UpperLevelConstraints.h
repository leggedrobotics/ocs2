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

#include <ocs2_frank_wolfe/NLP_Constraints.h>

namespace ocs2 {

/**
 * This class is an interface to a NLP constraints.
 */
class UpperLevelConstraints final : public NLP_Constraints {
 public:
  /**
   * Default constructor.
   */
  UpperLevelConstraints() = default;

  /**
   * Default destructor.
   */
  ~UpperLevelConstraints() override = default;

  /**
   * Sets the initial time, and final time.
   *
   * @param [in] initTime: The initial time.
   * @param [in] finalTime: The final time.
   */
  void set(const scalar_t& initTime, const scalar_t& finalTime) {
    initTime_ = initTime;
    finalTime_ = finalTime;
  }

  void setCurrentParameter(const dynamic_vector_t& x) override {
    eventTime_ = x;
    linearInequalityCoefficients(x.size(), initTime_, finalTime_, Cm_, Dv_);
  }

  void getLinearInequalityConstraint(dynamic_vector_t& h) override { h = Cm_ * eventTime_ + Dv_; }

  void getLinearInequalityConstraintDerivative(dynamic_matrix_t& dhdx) override { dhdx = Cm_; }

  /**
   * Computes the inequality constraints coefficients for the following conditions:
   *
   * \f$t_0 < s_0 < s_1 < ... < s_{n-1} < t_f\f$,
   *
   * where \f$t_0\f$ is the initial time and \f$t_f\f$ is the final time.
   * \f$n\f$ is the number of event times and \f$s_i\f$ is the ith event time.
   * The inequality constraints have the following format: \f$C_m [s_i] + D_v > 0\f$.
   *
   * @param [in] numEventTimes: The number of event times, \f$n\f$.
   * @param [in] initTime: The initial time, \f$t_0\f$.
   * @param [in] finalTime: The final time, \f$t_f\f$.
   * @param [out] Cm: \f$C_m\f$ matrix of the inequality constraints.
   * @param [out] Dv: \f$D_v\f$ vector of the inequality constraints.
   *
   */
  static void linearInequalityCoefficients(size_t numEventTimes, scalar_t initTime, scalar_t finalTime, dynamic_matrix_t& Cm,
                                           dynamic_vector_t& Dv) {
    Cm = dynamic_matrix_t::Zero(numEventTimes + 1, numEventTimes);
    for (size_t i = 0; i < numEventTimes + 1; i++) {
      if (i < numEventTimes) {
        Cm(i, i) = +1.0;
      }
      if (i > 0) {
        Cm(i, i - 1) = -1.0;
      }
    }

    Dv = dynamic_vector_t::Zero(numEventTimes + 1);
    Dv(0) = -initTime;
    Dv(numEventTimes) = finalTime;
  }

 private:
  scalar_t initTime_;
  scalar_t finalTime_;

  dynamic_vector_t eventTime_;

  dynamic_matrix_t Cm_;
  dynamic_vector_t Dv_;
};

}  // namespace ocs2
