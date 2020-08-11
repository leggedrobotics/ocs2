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

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <string>

#include <ocs2_mobile_manipulator_example/definitions.h>

namespace mobile_manipulator {

/**
 * Pinocchio interface class contatining robot model and data.
 * The robot model can be shared between interface instances.
 */
template <typename SCALAR>
class PinocchioInterface {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using PinocchioModel = pinocchio::ModelTpl<SCALAR>;
  using PinocchioData = typename PinocchioModel::Data;

  using AffineType = Eigen::Transform<SCALAR, 3, Eigen::Affine>;

  /**
   * Load pinocchio model from URDF
   * @param[in] urdfFile Path to URDF
   * @param[out] model pinocchio model
   */
  explicit PinocchioInterface(const std::string& urdfPath);

  /**
   * Copy constructor
   * Keeps a pointer to the shared robot model.
   */
  PinocchioInterface(const PinocchioInterface& other);

  /**
   * Copy assignment operator
   * Keeps a pointer to the shared robot model.
   */
  PinocchioInterface& operator=(const PinocchioInterface& rhs);

  const PinocchioModel& getModel() const { return *robotModel_; }
  PinocchioData& getData() { return robotData_; }

  /**
   * Gets the pose of a body in the (pinocchio) world frame
   * @param[in] name of the body (corresponds to the pinocchio name, which is usually the URDF link name)
   * @param[out] the body pose
   * TODO(perry) make this const by caching or mutabling the robotData_
   */
  AffineType getBodyPoseInWorldFrame(const std::string bodyName, const Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>& q);

 private:
  std::shared_ptr<const PinocchioModel> robotModel_;
  PinocchioData robotData_;
};

extern template class PinocchioInterface<scalar_t>;
extern template class PinocchioInterface<ad_scalar_t>;

}  // namespace mobile_manipulator
