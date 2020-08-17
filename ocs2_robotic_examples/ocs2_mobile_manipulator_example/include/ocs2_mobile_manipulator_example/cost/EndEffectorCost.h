/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#include <ocs2_mobile_manipulator_example/PinocchioInterface.h>
#include <ocs2_mobile_manipulator_example/definitions.h>

#include <ocs2_core/cost/QuadraticGaussNewtonCostBaseAD.h>

namespace mobile_manipulator {

class EndEffectorCost final : public ocs2::QuadraticGaussNewtonCostBaseAD {
 public:
  EndEffectorCost(const PinocchioInterface<ad_scalar_t>& pinocchioInterface, matrix_t Q, matrix_t R, matrix_t Qf);
  ~EndEffectorCost() override = default;

  /* Copy constructor */
  EndEffectorCost(const EndEffectorCost& rhs) : ocs2::QuadraticGaussNewtonCostBaseAD(rhs), Q_(rhs.Q_), R_(rhs.R_), Qf_(rhs.Qf_) {
    pinocchioInterface_.reset(new PinocchioInterface<ad_scalar_t>(*rhs.pinocchioInterface_));
  }

  EndEffectorCost* clone() const override { return new EndEffectorCost(*this); }

 protected:
  ad_vector_t intermediateCostFunction(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                                       const ad_vector_t& parameters) const override;
  ad_vector_t finalCostFunction(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& parameters) const override;

  /* Set num parameters to size of desired trajectory state */
  size_t getNumIntermediateParameters() const override;
  vector_t getIntermediateParameters(scalar_t time) const override;

  size_t getNumFinalParameters() const override;
  vector_t getFinalParameters(scalar_t time) const override;

 private:
  std::unique_ptr<PinocchioInterface<ad_scalar_t>> pinocchioInterface_;

  /* Quadratic cost */
  matrix_t Q_;
  matrix_t R_;
  matrix_t Qf_;
};

}  // namespace mobile_manipulator
