#include <ocs2_mobile_manipulator_example/PinocchioInterface.h>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace mobile_manipulator {

/**
 * Pinocchio interface class contatining robot model and data.
 * The robot model can be shared between interface instances.
 */
template <typename SCALAR>
PinocchioInterface<SCALAR>::PinocchioInterface(const std::string& urdfPath) {
  pinocchio::ModelTpl<double> tempModel;

  // build robot model and robot data
  pinocchio::urdf::buildModel(urdfPath, tempModel);

  robotModel_ = std::make_shared<const PinocchioModel>(tempModel.cast<SCALAR>());
  robotData_ = PinocchioData(*robotModel_);
}

/**
 * Copy constructor
 * Keeps a pointer to the shared robot model.
 */
template <typename SCALAR>
PinocchioInterface<SCALAR>::PinocchioInterface(const PinocchioInterface& other) {
  robotModel_ = other.robotModel_;
  robotData_ = other.robotData_;
}

/**
 * Copy assignment operator
 * Keeps a pointer to the shared robot model.
 */
template <typename SCALAR>
PinocchioInterface<SCALAR>& PinocchioInterface<SCALAR>::operator=(const PinocchioInterface& rhs) {
  robotModel_ = rhs.robotModel_;
  robotData_ = rhs.robotData_;
}

/**
 * Gets the pose of a body in the (pinocchio) world frame
 * @param[in] name of the body (corresponds to the pinocchio name, which is usually the URDF link name)
 * @param[out] the body pose
 * TODO(perry) make this const by caching or mutabling the robotData_
 */
template <typename SCALAR>
typename PinocchioInterface<SCALAR>::AffineType PinocchioInterface<SCALAR>::getBodyPoseInWorldFrame(
    const std::string bodyName, const Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>& q) {
  pinocchio::JointIndex bodyId = this->robotModel_->getBodyId(bodyName);

  pinocchio::forwardKinematics(*robotModel_, robotData_, q);
  pinocchio::updateFramePlacements(*this->robotModel_, this->robotData_);

  AffineType bodyPose;
  bodyPose.linear() = this->robotData_.oMf[bodyId].rotation();
  bodyPose.translation() = this->robotData_.oMf[bodyId].translation();

  return bodyPose;
}

}  // namespace mobile_manipulator
