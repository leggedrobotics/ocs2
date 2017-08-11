/*
 * AnymalComDynamics.cpp
 *
 *  Created on: Aug 11, 2017
 *      Author: Jan Carius
 */

#include <c_anymal_switched_model/dynamics/AnymalComDynamics.h>

namespace anymal
{

AnymalComDynamics::AnymalComDynamics() {
  // initialize the model
  const std::string urdfFilePath("/path/to/description.urdf");
  if(!model_.initModelFromUrdfFile(urdfFilePath)){
    std::runtime_error("Error initializing the model.");
  }

  // set intial zero state
  state_.setZero();
  model_.setState(state_,true,true,true);
}

Eigen::Matrix<double,6,6> AnymalComDynamics::comInertia(
    const Eigen::Matrix<double,12,1>& q) {
  setState(q);

  auto rbdlModel = model_.getRbdlModel();
  Eigen::VectorXd dummy_qdot = Eigen::VectorXd::Zero(12);
  Eigen::VectorXd dummy_qddot = Eigen::VectorXd::Zero(12);
  Eigen::VectorXd dummy_tau = Eigen::VectorXd::Zero(12);
  // execute fwd dynamics s.t. articulatd body algorithm updates all variables
  RigidBodyDynamics::ForwardDynamics(rbdlModel,q,dummy_qdot,dummy_tau,dummy_qddot);

  return rbdlModel.IA[0];
}

Eigen::Matrix<double,4,4> AnymalComDynamics::comHomogeneous(
    const Eigen::Matrix<double,12,1>& q) {
  setState(q);

  Eigen::Matrix<double,4,4> comHom = Eigen::Matrix<double,4,4>::Identity();

  const Eigen::Vector3d position =
      model_.getPositionWorldToCom(quadruped_model::CoordinateFrame::BASE);

  comHom.block<3,1>(0,3) = position;
  return comHom;
}

Eigen::Matrix<double,6,6> AnymalComDynamics::comInertiaDerivative(
    const Eigen::Matrix<double,12,1>& q,
    const Eigen::Matrix<double,12,1>& dq) {
  return Eigen::Matrix<double,6,6>::Zero(); //TODO
}

Eigen::Matrix<double,6,12> AnymalComDynamics::comMomentumJacobian(
    const Eigen::Matrix<double,12,1>& q) {
  return Eigen::Matrix<double,6,12>::Zero(); //TODO
}

Eigen::Matrix<double,6,12> AnymalComDynamics::comMomentumJacobianDerivative(
    const Eigen::Matrix<double,12,1>& q,
    const Eigen::Matrix<double,12,1>& dq) {
  return Eigen::Matrix<double,6,12>::Zero(); //TODO
}

Eigen::Matrix<double,3,1> AnymalComDynamics::comVelocityInBaseFrame(
    const Eigen::Matrix<double,12,1>& q, const Eigen::Matrix<double,12,1>& dq) {
  return model_.getLinearVelocityCom(quadruped_model::CoordinateFrame::BASE);
}

void AnymalComDynamics::setState(const Eigen::Matrix<double,12,1>& q){
  state_.getJointPositions().toImplementation() = q;
  model_.setState(state_,true,false,false);
}

void AnymalComDynamics::setState(const Eigen::Matrix<double,12,1>& q,
              const Eigen::Matrix<double,12,1>& dq){
  state_.getJointPositions().toImplementation() = q;
  state_.getJointVelocities().toImplementation() = dq;
  model_.setState(state_,true,true,false);
}


}  // end of namespace anymal
