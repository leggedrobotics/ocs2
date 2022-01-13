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

#include <ostream>
#include <string>
#include <type_traits>

#include <ocs2_core/Types.h>
#include <ocs2_core/automatic_differentiation/Types.h>

namespace ocs2 {

template <typename SCALAR>
class CentroidalModelInfoTpl;

using CentroidalModelInfo = CentroidalModelInfoTpl<scalar_t>;
using CentroidalModelInfoCppAd = CentroidalModelInfoTpl<ad_scalar_t>;

enum class CentroidalModelType { FullCentroidalDynamics, SingleRigidBodyDynamics };

std::string toString(CentroidalModelType type);
std::ostream& operator<<(std::ostream& os, CentroidalModelType type);

template <typename SCALAR>
struct CentroidalModelInfoTpl {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using scalar_t = SCALAR;
  using vector_t = Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>;
  using vector3_t = Eigen::Matrix<SCALAR, 3, 1>;
  using matrix3_t = Eigen::Matrix<SCALAR, 3, 3>;

  template <typename T>  // Template for conditional compilation using SFINAE
  using EnableIfScalar_t = typename std::enable_if<std::is_same<T, scalar_t>::value, bool>::type;

  CentroidalModelType centroidalModelType;      // full centroidal dynamics OR single rigid body dynamics (SRBD)
  size_t numThreeDofContacts;                   // 3DOF contacts, force only
  size_t numSixDofContacts;                     // 6DOF contacts, force and torque
  std::vector<size_t> endEffectorFrameIndices;  // indices of end-effector frames [3DOF contacts, 6DOF contacts]
  size_t generalizedCoordinatesNum;             // number of generalized coordinates in the pinocchio model
  size_t actuatedDofNum;                        // number of actuated degrees of freedom
  size_t stateDim;                              // number of states needed to define the system flow map
  size_t inputDim;                              // number of inputs needed to define the system flow map
  scalar_t robotMass;                           // total robot mass
  vector_t qPinocchioNominal;                   // nominal robot configuration used in the SRBD model
  matrix3_t centroidalInertiaNominal;           // nominal robot centroidal inertia used in the SRBD model (expressed in nominal base frame)
  vector3_t comToBasePositionNominal;           // nominal CoM to base position used in the SRBD model (expressed in nominal base frame)

  /** Casts CentroidalModelInfo to CentroidalModelInfoCppAD. */
  template <typename T = SCALAR, EnableIfScalar_t<T> = true>
  CentroidalModelInfoCppAd toCppAd() const;
};

/* Explicit template instantiation for scalar_t and ad_scalar_t */
extern template struct CentroidalModelInfoTpl<scalar_t>;
extern template struct CentroidalModelInfoTpl<ad_scalar_t>;
}  // namespace ocs2
