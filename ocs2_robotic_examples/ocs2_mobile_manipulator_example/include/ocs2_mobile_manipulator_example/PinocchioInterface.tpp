#include <iomanip>

#include <ocs2_mobile_manipulator_example/PinocchioInterface.h>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "CppAdHelpers.h"

namespace mobile_manipulator {

template <typename SCALAR>
PinocchioInterface<SCALAR>::PinocchioInterface(const std::string& urdfPath) {
  pinocchio::ModelTpl<double> tempModel;

  // add 3 DOF for wheelbase
  pinocchio::JointModelComposite jointComposite(3);
  jointComposite.addJoint(pinocchio::JointModelPX());
  jointComposite.addJoint(pinocchio::JointModelPY());
  jointComposite.addJoint(pinocchio::JointModelRZ());

  // build robot model and robot data
  pinocchio::urdf::buildModel(urdfPath, jointComposite, tempModel);

  robotModel_ = std::make_shared<const PinocchioModel>(tempModel.cast<SCALAR>());
  robotData_ = PinocchioData(*robotModel_);
}

template <typename SCALAR>
PinocchioInterface<SCALAR>::PinocchioInterface(const PinocchioInterface& other) {
  robotModel_ = other.robotModel_;
  robotData_ = other.robotData_;
}

template <typename SCALAR>
PinocchioInterface<SCALAR>& PinocchioInterface<SCALAR>::operator=(const PinocchioInterface& rhs) {
  robotModel_ = rhs.robotModel_;
  robotData_ = rhs.robotData_;
}

template <typename SCALAR>
Pose<SCALAR> PinocchioInterface<SCALAR>::getBodyPoseInWorldFrame(const std::string bodyName,
                                                                 const Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>& q) {
  const pinocchio::JointIndex bodyId = robotModel_->getBodyId(bodyName);

  pinocchio::forwardKinematics(*robotModel_, robotData_, q);
  pinocchio::updateFramePlacements(*robotModel_, robotData_);

  Pose<SCALAR> pose;
  pose.position = robotData_.oMf[bodyId].translation();
  pose.orientation = matrixToQuaternion(robotData_.oMf[bodyId].rotation());

  return pose;
}

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

}  // namespace mobile_manipulator
