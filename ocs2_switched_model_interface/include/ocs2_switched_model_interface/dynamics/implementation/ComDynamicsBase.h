/*
 * ComDynamicsBase.h
 *
 *  Created on: Nov 7, 2017
 *      Author: farbod
 */

namespace switched_model {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
ComDynamicsBase<JOINT_COORD_SIZE>* ComDynamicsBase<JOINT_COORD_SIZE>::clone() const {
  return new ComDynamicsBase<JOINT_COORD_SIZE>(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void ComDynamicsBase<JOINT_COORD_SIZE>::setData(const joint_coordinate_t& qJoints, const joint_coordinate_t& dqJoints) {
  qJoints_ = qJoints;    // Joints' coordinate
  dqJoints_ = dqJoints;  // Joints' velocity
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void ComDynamicsBase<JOINT_COORD_SIZE>::computeFlowMap(const scalar_t& t, const state_vector_t& x, const input_vector_t& u,
                                                       state_vector_t& dxdt) {
  // Rotation matrix from Base frame (or the coincided frame world frame) to Origin frame (global world).
  Eigen::Matrix3d o_R_b = RotationMatrixBasetoOrigin(x.head<3>());

  // base to CoM displacement in the CoM frame
  com_base2CoM_ = comModelPtr_->comPositionBaseFrame(qJoints_);

  // base coordinate
  q_base_.template head<3>() = x.segment<3>(0);
  q_base_.template tail<3>() = x.segment<3>(3) - o_R_b * com_base2CoM_;

  // update kinematic model
  kinematicModelPtr_->update(q_base_, qJoints_);

  // local angular velocity (com_W_com) and local linear velocity (com_V_com) of CoM
  Eigen::VectorBlock<const state_vector_t, 3> com_W_com = x.segment<3>(6);
  Eigen::VectorBlock<const state_vector_t, 3> com_V_com = x.segment<3>(9);

  // base to stance feet displacement in the CoM frame
  for (size_t i = 0; i < 4; i++) {
    kinematicModelPtr_->footPositionBaseFrame(i, com_base2StanceFeet_[i]);
  }

  // Inertia matrix in the CoM frame and its derivatives
  M_ = comModelPtr_->comInertia(qJoints_);
  dMdt_ = comModelPtr_->comInertiaDerivative(qJoints_, dqJoints_);
  Eigen::Matrix3d rotationMInverse = M_.topLeftCorner<3, 3>().inverse();
  MInverse_ << rotationMInverse, Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Identity() / M_(5, 5);

  // Coriolis and centrifugal forces
  C_.head<3>() = com_W_com.cross(M_.topLeftCorner<3, 3>() * com_W_com) + dMdt_.topLeftCorner<3, 3>() * com_W_com;
  C_.tail<3>().setZero();

  // gravity effect on CoM in CoM coordinate
  MInverseG_ << Eigen::Vector3d::Zero(), -o_R_b.transpose() * o_gravityVector_;

  // CoM Jacobian in the Base frame
  b_comJacobainOmega_ = (MInverse_ * comModelPtr_->comMomentumJacobian(qJoints_)).template topRows<3>();

  // contact JacobianTransposeLambda
  Eigen::Vector3d com_comToFoot;
  Eigen::Matrix<double, 6, 1> JcTransposeLambda(Eigen::VectorXd::Zero(6));
  for (size_t i = 0; i < 4; i++) {
    com_comToFoot = com_base2StanceFeet_[i] - com_base2CoM_;

    JcTransposeLambda.head<3>() += com_comToFoot.cross(u.segment<3>(3 * i));
    JcTransposeLambda.tail<3>() += u.segment<3>(3 * i);
  }

  // angular velocities to Euler angle derivatives transformation
  Eigen::Vector3d eulerAngles = x.head<3>();
  Eigen::Matrix3d transformAngVel2EulerAngDev = ocs2::AngularVelocitiesToEulerAngleDerivativesMatrix(eulerAngles);

  // CoM dynamics
  dxdt.segment<3>(0) = transformAngVel2EulerAngDev * (com_W_com - b_comJacobainOmega_ * dqJoints_);
  dxdt.segment<3>(3) = o_R_b * com_V_com;
  dxdt.tail<6>() = MInverse_ * (-C_ + JcTransposeLambda) - MInverseG_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void ComDynamicsBase<JOINT_COORD_SIZE>::getFeetPositions(std::array<Eigen::Vector3d, 4>& b_base2StanceFeet) const {
  b_base2StanceFeet = com_base2StanceFeet_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void ComDynamicsBase<JOINT_COORD_SIZE>::getBasePose(base_coordinate_t& o_basePose) const {
  o_basePose = q_base_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
Eigen::Matrix<double, 6, 6> ComDynamicsBase<JOINT_COORD_SIZE>::getM() const {
  return M_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
Eigen::Matrix<double, 6, 1> ComDynamicsBase<JOINT_COORD_SIZE>::getC() const {
  return C_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
Eigen::Matrix<double, 6, 1> ComDynamicsBase<JOINT_COORD_SIZE>::getG() const {
  return M_ * MInverseG_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
Eigen::Matrix<double, 6, 6> ComDynamicsBase<JOINT_COORD_SIZE>::getMInverse() const {
  return MInverse_;
}

}  // end of namespace switched_model
