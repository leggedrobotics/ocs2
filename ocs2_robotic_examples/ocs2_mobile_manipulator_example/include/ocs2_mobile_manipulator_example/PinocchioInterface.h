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
// #include <Eigen/Geometry>
#include <memory>
#include <string>

#include <ocs2_core/Types.h>
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

using ad_base_t = CppAD::cg::CG<scalar_t>;
using ad_scalar_t = CppAD::AD<ad_base_t>;

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
class PinocchioInterface final {
 public:
  using PinocchioModel = pinocchio::ModelTpl<SCALAR, 0, pinocchio::JointCollectionDefaultTpl>;
  using PinocchioData = typename pinocchio::DataTpl<SCALAR, 0, pinocchio::JointCollectionDefaultTpl>;
  using PinocchioJointModel = pinocchio::JointModelTpl<scalar_t, 0, pinocchio::JointCollectionDefaultTpl>;

  using MatrixX = Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic>;

  /**
   * Construct from given pinocchio model
   * @param[in] model pinocchio model
   */
  explicit PinocchioInterface(const PinocchioModel& model);

  /** Destructor */
  ~PinocchioInterface();

  /** Copy constructor */
  PinocchioInterface(const PinocchioInterface& rhs);

  /** Move constructor */
  PinocchioInterface(PinocchioInterface&& rhs);

  /** Copy assignment operator */
  PinocchioInterface& operator=(const PinocchioInterface& rhs);

  /** Move assignment */
  PinocchioInterface<SCALAR>& operator=(PinocchioInterface&& rhs);

  /** Get the pinocchio model */
  const PinocchioModel& getModel() const { return *robotModelPtr_; }

  /** Get the pinocchio data */
  PinocchioData& getData() { return *robotDataPtr_; }

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

  /** Cast pinocchio interface to CppAD scalar type. */
  static PinocchioInterface<ad_scalar_t> castToCppAd(const PinocchioInterface<scalar_t>& interface);

  /** Factory function from URDF */
  static PinocchioInterface<scalar_t> buildFromUrdf(const std::string& urdfPath);
  static PinocchioInterface<scalar_t> buildFromUrdf(const std::string& urdfPath, const PinocchioJointModel& rootJoint);

  /** Factory function from URDF XML stream */
  static PinocchioInterface<scalar_t> buildFromXml(const std::string& xmlStream);
  static PinocchioInterface<scalar_t> buildFromXml(const std::string& xmlStream, const PinocchioJointModel& rootJoint);

  /** Prints some debug info of the pinocchio model. */
  void display();

 private:
  std::shared_ptr<const PinocchioModel> robotModelPtr_;
  std::unique_ptr<PinocchioData> robotDataPtr_;
};

extern template class PinocchioInterface<scalar_t>;
extern template class PinocchioInterface<ad_scalar_t>;

}  // namespace ocs2
