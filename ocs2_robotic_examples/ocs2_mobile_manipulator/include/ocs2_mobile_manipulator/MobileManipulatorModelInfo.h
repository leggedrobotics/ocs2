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

#include <type_traits>

#include <ocs2_core/Types.h>
#include <ocs2_core/automatic_differentiation/Types.h>

namespace ocs2 {
namespace mobile_manipulator {

template <typename SCALAR>
class MobileManipulatorModelInfoTpl;

using MobileManipulatorModelInfo = MobileManipulatorModelInfoTpl<scalar_t>;
using MobileManipulatorModelInfoCppAd = MobileManipulatorModelInfoTpl<ad_scalar_t>;

enum class ManipulatorModelType {
  DefaultManipulator = 0,           // default model from the parsed URDF
  WheelBasedMobileManipulator = 1,  // adds dummy XY-Y joints to the model parsed from URDF
  FloatingArmManipulator = 2,       // adds dummy XYZ-RPY joints to the model parsed from URDF
};

template <typename SCALAR>
struct MobileManipulatorModelInfoTpl {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using scalar_t = SCALAR;
  using vector_t = Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>;
  using vector3_t = Eigen::Matrix<SCALAR, 3, 1>;
  using matrix3_t = Eigen::Matrix<SCALAR, 3, 3>;

  template <typename T>  // Template for conditional compilation using SFINAE
  using EnableIfScalar_t = typename std::enable_if<std::is_same<T, scalar_t>::value, bool>::type;

  ManipulatorModelType manipulatorModelType;  // type of manipulator: floating-base, wheel-base, default
  size_t stateDim;                            // number of states needed to define the system flow map
  size_t inputDim;                            // number of inputs needed to define the system flow map

  /** Casts MobileManipulatorModelInfo to MobileManipulatorModelInfoCppAD. */
  template <typename T = SCALAR, EnableIfScalar_t<T> = true>
  MobileManipulatorModelInfoCppAd toCppAd() const;
};

/* Explicit template instantiation for scalar_t and ad_scalar_t */
extern template struct MobileManipulatorModelInfoTpl<scalar_t>;
extern template struct MobileManipulatorModelInfoTpl<ad_scalar_t>;

}  // namespace mobile_manipulator
}  // namespace ocs2

// EOF
