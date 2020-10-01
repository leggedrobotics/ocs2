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

#include <Eigen/Dense>
#include <iosfwd>
#include <memory>
#include <string>

#include <ocs2_core/Types.h>
#include <ocs2_core/automatic_differentiation/Types.h>
#include <cppad/cg.hpp>

/* Forward declaration of main pinocchio types */
namespace pinocchio {
template <typename Scalar, int Options>
struct JointCollectionDefaultTpl;
template <typename Scalar, int Options, template <typename S, int O> class JointCollectionTpl>
struct ModelTpl;
template <typename Scalar, int Options, template <typename S, int O> class JointCollectionTpl>
struct DataTpl;
template <typename Scalar, int Options, template <typename S, int O> class JointCollectionTpl>
struct JointModelTpl;
}  // namespace pinocchio

namespace ocs2 {

template <typename SCALAR>
class PinocchioInterfaceTpl;

using PinocchioInterface = PinocchioInterfaceTpl<scalar_t>;
using PinocchioInterfaceCppAd = PinocchioInterfaceTpl<ad_scalar_t>;

/**
 * Pose conatining position vector and orientation quaternion
 * @tparam SCALAR: scalar type
 */
template <typename SCALAR>
struct Pose {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Matrix<SCALAR, 3, 1> position;
  Eigen::Quaternion<SCALAR> orientation;
};

/**
 * Pinocchio interface class contatining robot model and data.
 * The robot model is shared between interface instances.
 */
template <typename SCALAR>
class PinocchioInterfaceTpl final {
 public:
  using Model = pinocchio::ModelTpl<SCALAR, 0, pinocchio::JointCollectionDefaultTpl>;
  using Data = typename pinocchio::DataTpl<SCALAR, 0, pinocchio::JointCollectionDefaultTpl>;
  using JointModel = pinocchio::JointModelTpl<scalar_t, 0, pinocchio::JointCollectionDefaultTpl>;

  using MatrixX = Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic>;

  /**
   * Construct from given pinocchio model
   * @param[in] model pinocchio model
   */
  explicit PinocchioInterfaceTpl(const Model& model);

  /** Destructor */
  ~PinocchioInterfaceTpl();

  /** Copy constructor */
  PinocchioInterfaceTpl(const PinocchioInterfaceTpl& rhs);

  /** Move constructor */
  PinocchioInterfaceTpl(PinocchioInterfaceTpl&& rhs);

  /** Copy assignment operator */
  PinocchioInterfaceTpl& operator=(const PinocchioInterfaceTpl& rhs);

  /** Move assignment */
  PinocchioInterfaceTpl<SCALAR>& operator=(PinocchioInterfaceTpl&& rhs);

  /** Get the pinocchio model */
  const Model& getModel() const { return *robotModelPtr_; }

  /** Get the pinocchio data */
  Data& getData() { return *robotDataPtr_; }

  /**
   * Gets the pose of a body in the (pinocchio) world frame
   *
   * @param[in] bodyName name of the body (corresponds to the pinocchio name, which is usually the URDF link name)
   * @param[in] q joint configuration
   * @return the body pose
   * TODO(perry) make this const by caching or mutabling the robotDataPtr_
   */
  Pose<SCALAR> getBodyPoseInWorldFrame(const std::string bodyName, const Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>& q);

  void computeAllJacobians(const Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>& q);

  MatrixX getJacobianOfJoint(size_t jointIndex);

  Pose<SCALAR> getJointPose(size_t jointIndex, const Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>& q);

  friend std::ostream& operator<<(std::ostream& os, const PinocchioInterfaceTpl<scalar_t>& p);

 private:
  std::shared_ptr<const Model> robotModelPtr_;
  std::unique_ptr<Data> robotDataPtr_;
};

/** Print PinocchioInterfaceTpl info to stream */
std::ostream& operator<<(std::ostream& os, const PinocchioInterface& p);

/** Factory function from URDF file */
PinocchioInterface getPinocchioInterfaceFromUrdfFile(const std::string& urdfFile);

/** Factory function from URDF file with root joint */
PinocchioInterface getPinocchioInterfaceFromUrdfFile(const std::string& urdfFile, const PinocchioInterface::JointModel& rootJoint);

/** Factory function from URDF string */
PinocchioInterface getPinocchioInterfaceFromUrdfString(const std::string& xmlString);

/** Factory function from URDF string with root joint */
PinocchioInterface getPinocchioInterfaceFromUrdfString(const std::string& xmlString, const PinocchioInterface::JointModel& rootJoint);

/** Cast pinocchio interface to CppAD scalar type. */
PinocchioInterfaceCppAd castToCppAd(const PinocchioInterface& interface);

/* Explicit template instantiation for scalar_t and ad_scalar_t */
extern template class PinocchioInterfaceTpl<scalar_t>;
extern template class PinocchioInterfaceTpl<ad_scalar_t>;

}  // namespace ocs2
