/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

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

#include <ocs2_mobile_manipulator/definitions.h>
#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>

namespace ocs2 {
namespace mobile_manipulator {

template <typename SCALAR>
class MobileManipulatorPinocchioMapping final : public PinocchioStateInputMapping<SCALAR> {
 public:
  using Base = PinocchioStateInputMapping<SCALAR>;
  using typename Base::matrix_t;
  using typename Base::vector_t;

  MobileManipulatorPinocchioMapping() = default;
  ~MobileManipulatorPinocchioMapping() override = default;
  MobileManipulatorPinocchioMapping<SCALAR>* clone() const override { return new MobileManipulatorPinocchioMapping<SCALAR>(*this); }

  vector_t getPinocchioJointPosition(const vector_t& state) const override { return state; }

  vector_t getPinocchioJointVelocity(const vector_t& state, const vector_t& input) const override {
    vector_t dxdt(STATE_DIM);
    const auto theta = state(2);
    const auto v = input(0);  // forward velocity in base frame
    dxdt << cos(theta) * v, sin(theta) * v, input(1), input.tail(6);
    return dxdt;
  }

  std::pair<matrix_t, matrix_t> getOcs2Jacobian(const vector_t& state, const matrix_t& Jq, const matrix_t& Jv) const override {
    matrix_t dfdu(Jv.rows(), INPUT_DIM);
    Eigen::Matrix<SCALAR, 3, 2> dvdu_base;
    const SCALAR theta = state(2);
    // clang-format off
    dvdu_base << cos(theta), 0,
                 sin(theta), 0,
                 0, 1.0;
    // clang-format on
    dfdu.template leftCols<2>() = Jv.template leftCols<3>() * dvdu_base;
    dfdu.template rightCols<6>() = Jv.template rightCols<6>();
    return {Jq, dfdu};
  }
};

}  // namespace mobile_manipulator
}  // namespace ocs2
