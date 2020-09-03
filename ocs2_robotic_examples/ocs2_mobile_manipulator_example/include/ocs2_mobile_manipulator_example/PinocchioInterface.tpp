#include <iomanip>

#include <pinocchio/codegen/cppadcg.hpp>
#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "CppAdHelpers.h"

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
PinocchioInterface<SCALAR>::PinocchioInterface(const PinocchioModel& model) {
  robotModelPtr_ = std::make_shared<const PinocchioModel>(model);
  robotDataPtr_ = std::unique_ptr<PinocchioData>(new PinocchioData(*robotModelPtr_));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
PinocchioInterface<SCALAR>::PinocchioInterface(const std::string& urdfPath) {
  pinocchio::ModelTpl<scalar_t> tempModel;

  // add 3 DOF for wheelbase
  pinocchio::JointModelComposite jointComposite(3);
  jointComposite.addJoint(pinocchio::JointModelPX());
  jointComposite.addJoint(pinocchio::JointModelPY());
  jointComposite.addJoint(pinocchio::JointModelRZ());

  // build robot model and robot data
  pinocchio::urdf::buildModel(urdfPath, jointComposite, tempModel);

  robotModelPtr_ = std::make_shared<const PinocchioModel>(tempModel.cast<SCALAR>());
  robotDataPtr_ = std::unique_ptr<PinocchioData>(new PinocchioData(*robotModelPtr_));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
PinocchioInterface<SCALAR>::~PinocchioInterface() = default;

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
PinocchioInterface<SCALAR>::PinocchioInterface(const PinocchioInterface& other) {
  robotModelPtr_ = other.robotModelPtr_;
  robotDataPtr_.reset(new PinocchioData(*other.robotDataPtr_));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
PinocchioInterface<SCALAR>& PinocchioInterface<SCALAR>::operator=(const PinocchioInterface& rhs) {
  robotModelPtr_ = rhs.robotModelPtr_;
  robotDataPtr_.reset(new PinocchioData(*rhs.robotDataPtr_));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
Pose<SCALAR> PinocchioInterface<SCALAR>::getBodyPoseInWorldFrame(const std::string bodyName,
                                                                 const Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>& q) {
  const pinocchio::FrameIndex bodyId = robotModelPtr_->getBodyId(bodyName);

  pinocchio::forwardKinematics(*robotModelPtr_, *robotDataPtr_, q);
  pinocchio::updateFramePlacements(*robotModelPtr_, *robotDataPtr_);

  Pose<SCALAR> pose;
  pose.position = robotDataPtr_->oMf[bodyId].translation();
  pose.orientation = matrixToQuaternion(robotDataPtr_->oMf[bodyId].rotation());

  return pose;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
PinocchioInterface<ad_scalar_t> PinocchioInterface<SCALAR>::castToCppAd(const PinocchioInterface<scalar_t>& interface) {
  return PinocchioInterface<ad_scalar_t>(interface.getModel().template cast<ad_scalar_t>());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
void PinocchioInterface<SCALAR>::display() {
  const auto& model = getModel();
  std::cout << "model.nv = " << model.nv << '\n';
  std::cout << "model.nq = " << model.nq << '\n';
  std::cout << "model.njoints = " << model.njoints << '\n';
  std::cout << "model.nbodies = " << model.nbodies << '\n';
  std::cout << "model.nframes = " << model.nframes << '\n';

  std::cout << "\nJoints:\n";
  for (int k = 0; k < model.njoints; ++k) {
    std::cerr << std::setw(12) << model.names[k] << ":  ";
    std::cerr << " ID = " << k;
    std::cerr << '\n';
  }

  std::cout << "\nFrames:\n";
  for (int k = 0; k < model.nframes; ++k) {
    std::cerr << std::setw(12) << model.frames[k].name << ":  ";
    std::cerr << " ID = " << k;
    std::cerr << ", parent = " << model.frames[k].parent;
    std::cerr << ", type = ";

    std::string frameType;
    if ((model.frames[k].type & pinocchio::FrameType::OP_FRAME) != 0) {
      frameType += "OP_FRAME ";
    }
    if ((model.frames[k].type & pinocchio::FrameType::JOINT) != 0) {
      frameType += "JOINT ";
    }
    if ((model.frames[k].type & pinocchio::FrameType::FIXED_JOINT) != 0) {
      frameType += "FIXED_JOINT ";
    }
    if ((model.frames[k].type & pinocchio::FrameType::BODY) != 0) {
      frameType += "BODY ";
    }
    if ((model.frames[k].type & pinocchio::FrameType::SENSOR) != 0) {
      frameType += "SENSOR ";
    }
    std::cerr << "\"" << frameType << "\"\n";
  }
}

}  // namespace ocs2
