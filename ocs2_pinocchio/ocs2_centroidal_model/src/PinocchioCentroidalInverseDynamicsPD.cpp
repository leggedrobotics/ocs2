#include "ocs2_centroidal_model/PinocchioCentroidalInverseDynamicsPD.h"

#include <pinocchio/algorithm/rnea.hpp>

#include "ocs2_centroidal_model/AccessHelperFunctions.h"
#include "ocs2_centroidal_model/ModelHelperFunctions.h"

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioCentroidalInverseDynamicsPD::PinocchioCentroidalInverseDynamicsPD(PinocchioInterface& pinocchioInterface,
                                                                           CentroidalModelInfo centroidalModelInfo,
                                                                           std::vector<std::string> contactNames3DoF)
    : pinocchioInterfacePtr_(&pinocchioInterface),
      centroidalModelPinocchioMapping_(centroidalModelInfo),
      centroidalModelRbdConversions_(pinocchioInterface, centroidalModelInfo),
      contactNames3DoF_(std::move(contactNames3DoF)) {
  centroidalModelPinocchioMapping_.setPinocchioInterface(pinocchioInterface);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t PinocchioCentroidalInverseDynamicsPD::getTorque(const vector_t& desiredState, const vector_t& desiredInput,
                                                         const vector_t& desiredJointAccelerations, const vector_t& measuredState,
                                                         const vector_t& measuredInput) {
  // handles
  auto& interface = *pinocchioInterfacePtr_;
  const auto& info = centroidalModelPinocchioMapping_.getCentroidalModelInfo();
  const auto& model = pinocchioInterfacePtr_->getModel();
  auto& data = pinocchioInterfacePtr_->getData();

  // desired
  CentroidalModelRbdConversions::Vector6 desiredBasePose, desiredBaseVelocity, desiredBaseAcceleration;
  centroidalModelRbdConversions_.computeBaseKinematicsFromCentroidalModel(desiredState, desiredInput, desiredJointAccelerations,
                                                                          desiredBasePose, desiredBaseVelocity, desiredBaseAcceleration);
  vector_t qDesired(info.generalizedCoordinatesNum), vDesired(info.generalizedCoordinatesNum), aDesired(info.generalizedCoordinatesNum);
  qDesired << desiredBasePose, centroidal_model::getJointAngles(desiredState, info);
  vDesired << desiredBaseVelocity, centroidal_model::getJointVelocities(desiredInput, info);
  aDesired << desiredBaseAcceleration, desiredJointAccelerations;
  pinocchio::container::aligned_vector<pinocchio::Force> fextDesired(model.njoints, pinocchio::Force::Zero());
  for (size_t i = 0; i < info.numThreeDofContacts; i++) {
    const auto name = contactNames3DoF_[i];
    const auto frameIndex = model.getBodyId(name);
    const auto jointIndex = model.frames[frameIndex].parent;
    const Eigen::Vector3d translationJointFrameToContactFrame = model.frames[frameIndex].placement.translation();
    const Eigen::Matrix3d rotationWorldFrameToJointFrame = data.oMi[jointIndex].rotation().transpose();
    const Eigen::Vector3d contactForce = rotationWorldFrameToJointFrame * centroidal_model::getContactForces(desiredInput, i, info);
    fextDesired[jointIndex].linear() = contactForce;
    fextDesired[jointIndex].angular() = translationJointFrameToContactFrame.cross(contactForce);
  }

  // measured
  const vector_t qMeasured = centroidalModelPinocchioMapping_.getPinocchioJointPosition(measuredState);
  updateCentroidalDynamics(interface, info, qMeasured);
  const vector_t vMeasured = centroidalModelPinocchioMapping_.getPinocchioJointVelocity(measuredState, measuredInput);

  // feedforward
  vector_t torque = pinocchio::rnea(model, data, qDesired, vDesired, aDesired, fextDesired);

  // feedback
  torque.noalias() += pGains_ * (qDesired - qMeasured);
  torque.noalias() += dGains_ * (vDesired - vMeasured);

  return torque;
}

}  // namespace ocs2
