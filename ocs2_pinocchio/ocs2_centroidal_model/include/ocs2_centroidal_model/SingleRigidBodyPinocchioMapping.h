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

#include "ocs2_centroidal_model/utils.h"

#include <ocs2_core/Types.h>
#include <ocs2_core/automatic_differentiation/Types.h>

#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_robotic_tools/common/SkewSymmetricMatrix.h>

#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/centroidal.hpp>

namespace ocs2 {

template <typename SCALAR>
class SingleRigidBodyPinocchioMapping final : public PinocchioStateInputMapping<SCALAR> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using vector_t = Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>;
  using matrix_t = Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic>;

  using vector6_t = Eigen::Matrix<SCALAR, 6, 1>;
  using vector3_t = Eigen::Matrix<SCALAR, 3, 1>;
  using matrix3x_t = Eigen::Matrix<SCALAR, 3, Eigen::Dynamic>;
  using matrix6x_t = Eigen::Matrix<SCALAR, 6, Eigen::Dynamic>;
  using matrix3_t = Eigen::Matrix<SCALAR, 3, 3>;
  using matrix6_t = Eigen::Matrix<SCALAR, 6, 6>;

  using Model = pinocchio::ModelTpl<SCALAR>;
  using Data = typename Model::Data;

  SingleRigidBodyPinocchioMapping(size_t stateDim, size_t inputDim, const vector_t& qPinocchioNominal,
                                  const PinocchioInterfaceTpl<SCALAR>& pinocchioInterface);

  ~SingleRigidBodyPinocchioMapping() override = default;
  SingleRigidBodyPinocchioMapping<SCALAR>* clone() const override { return new SingleRigidBodyPinocchioMapping<SCALAR>(*this); }

  vector_t getPinocchioJointPosition(const vector_t& state) const override;

  vector_t getPinocchioJointVelocity(const vector_t& state, const vector_t& input) const override;

  std::pair<matrix_t, matrix_t> getOcs2Jacobian(const vector_t& state, const matrix_t& Jq, const matrix_t& Jv) const override;

  matrix6x_t getCentroidalMomentumMatrix(const vector_t& state) const;

  vector3_t getComPositionInWorld(const vector_t& state) const;

  matrix3x_t getComJacobianInWorld(const vector_t& state) const;

 private:
  PinocchioInterfaceTpl<SCALAR> pinocchioInterface_;
  SCALAR robotMass_;              // total robot mass
  vector_t qPinocchioNominal_;    // nominal robot configuration used in the SRBD model
  matrix3_t I_com_nominal_;       // nominal robot centroidal inertia used in the SRBD model (expressed in nominal base frame)
  vector3_t r_com_base_nominal_;  // nominal CoM to base position used in the SRBD model (expressed in nominal base frame)
  const size_t stateDim_;
  const size_t inputDim_;
};

/* Explicit template instantiation for scalar_t and ad_scalar_t */
extern template class SingleRigidBodyPinocchioMapping<scalar_t>;
extern template class SingleRigidBodyPinocchioMapping<ad_scalar_t>;
}  // namespace ocs2
