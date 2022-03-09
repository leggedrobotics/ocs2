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

#include "ocs2_mobile_manipulator/MobileManipulatorPinocchioMapping.h"

namespace ocs2 {
namespace mobile_manipulator {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
MobileManipulatorPinocchioMappingTpl<SCALAR>::MobileManipulatorPinocchioMappingTpl(ManipulatorModelInfo info)
    : modelInfo_(std::move(info)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
MobileManipulatorPinocchioMappingTpl<SCALAR>* MobileManipulatorPinocchioMappingTpl<SCALAR>::clone() const {
  return new MobileManipulatorPinocchioMappingTpl<SCALAR>(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto MobileManipulatorPinocchioMappingTpl<SCALAR>::getPinocchioJointPosition(const vector_t& state) const -> vector_t {
  return state;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto MobileManipulatorPinocchioMappingTpl<SCALAR>::getPinocchioJointVelocity(const vector_t& state, const vector_t& input) const
    -> vector_t {
  vector_t vPinocchio = vector_t::Zero(modelInfo_.stateDim);
  // set velocity model based on model type
  switch (modelInfo_.manipulatorModelType) {
    case ManipulatorModelType::DefaultManipulator: {
      vPinocchio = input;
      break;
    }
    case ManipulatorModelType::FloatingArmManipulator: {
      vPinocchio.tail(modelInfo_.armDim) = input;
      break;
    }
    case ManipulatorModelType::FullyActuatedFloatingArmManipulator: {
      vPinocchio << input;
      break;
    }
    case ManipulatorModelType::WheelBasedMobileManipulator: {
      const auto theta = state(2);
      const auto v = input(0);  // forward velocity in base frame
      vPinocchio << cos(theta) * v, sin(theta) * v, input(1), input.tail(modelInfo_.armDim);
      break;
    }
    default: {
      throw std::runtime_error("The chosen manipulator model type is not supported!");
    }
  }  // end of switch-case

  return vPinocchio;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto MobileManipulatorPinocchioMappingTpl<SCALAR>::getOcs2Jacobian(const vector_t& state, const matrix_t& Jq, const matrix_t& Jv) const
    -> std::pair<matrix_t, matrix_t> {
  // set jacobian model based on model type
  switch (modelInfo_.manipulatorModelType) {
    case ManipulatorModelType::DefaultManipulator: {
      return {Jq, Jv};
    }
    case ManipulatorModelType::FloatingArmManipulator: {
      matrix_t dfdu(Jv.rows(), modelInfo_.inputDim);
      dfdu = Jv.template rightCols(modelInfo_.armDim);
      return {Jq, dfdu};
    }
    case ManipulatorModelType::FullyActuatedFloatingArmManipulator: {
      return {Jq, Jv};
    }
    case ManipulatorModelType::WheelBasedMobileManipulator: {
      matrix_t dfdu(Jv.rows(), modelInfo_.inputDim);
      Eigen::Matrix<SCALAR, 3, 2> dvdu_base;
      const SCALAR theta = state(2);
      // clang-format off
      dvdu_base << cos(theta), SCALAR(0),
                   sin(theta), SCALAR(0),
                   SCALAR(0), SCALAR(1.0);
      // clang-format on
      dfdu.template leftCols<2>() = Jv.template leftCols<3>() * dvdu_base;
      dfdu.template rightCols(modelInfo_.armDim) = Jv.template rightCols(modelInfo_.armDim);
      return {Jq, dfdu};
    }
    default: {
      throw std::runtime_error("The chosen manipulator model type is not supported!");
    }
  }  // end of switch-case
}

// explicit template instantiation
template class ocs2::mobile_manipulator::MobileManipulatorPinocchioMappingTpl<ocs2::scalar_t>;
template class ocs2::mobile_manipulator::MobileManipulatorPinocchioMappingTpl<ocs2::ad_scalar_t>;

}  // namespace mobile_manipulator
}  // namespace ocs2
