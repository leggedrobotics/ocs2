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
std::shared_ptr<typename ComDynamicsBase<JOINT_COORD_SIZE>::Base> ComDynamicsBase<JOINT_COORD_SIZE>::clone() const {

	return std::allocate_shared< ComDynamicsBase<JOINT_COORD_SIZE>, Eigen::aligned_allocator<ComDynamicsBase<JOINT_COORD_SIZE>> > (
			Eigen::aligned_allocator<ComDynamicsBase<JOINT_COORD_SIZE>>(), *this);
};


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void ComDynamicsBase<JOINT_COORD_SIZE>::initializeModel(const std::vector<size_t>& systemStockIndexes, const std::vector<scalar_t>& switchingTimes,
		const state_vector_t& initState, const size_t& activeSubsystemIndex/*=0*/, const char* algorithmName/*=NULL*/) {

	Base::initializeModel(systemStockIndexes, switchingTimes, initState, activeSubsystemIndex, algorithmName);
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void ComDynamicsBase<JOINT_COORD_SIZE>::setData(const std::array<bool,4>& stanceLegs,
		const joint_coordinate_t& qJoints,
		const joint_coordinate_t& dqJoints)  {

	stanceLegs_ = stanceLegs;
	qJoints_  = qJoints;	  // Joints' coordinate
	dqJoints_ = dqJoints;     // Joints' velocity
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void ComDynamicsBase<JOINT_COORD_SIZE>::computeDerivative(const scalar_t& t,
		const state_vector_t& x,
		const control_vector_t& u,
		state_vector_t& dxdt)   {

	// Rotation matrix from Base frame (or the coincided frame world frame) to Origin frame (global world).
	Eigen::Matrix3d o_R_b = SwitchedModel<JOINT_COORD_SIZE>::RotationMatrixBasetoOrigin(x.head<3>());

	// base to CoM displacement in the CoM frame
	com_base2CoM_ = comModelPtr_->comPositionBaseFrame(qJoints_);

	// base coordinate
	q_base_.template head<3>() = x.segment<3>(0);
	q_base_.template tail<3>() = x.segment<3>(3) - o_R_b * com_base2CoM_;

	// local angular velocity (com_W_com) and local linear velocity (com_V_com) of CoM
	Eigen::VectorBlock<const state_vector_t,3> com_W_com = x.segment<3>(6);
	Eigen::VectorBlock<const state_vector_t,3> com_V_com = x.segment<3>(9);

	// update kinematic model
	kinematicModelPtr_->update(q_base_, qJoints_);

	// base to stance feet displacement in the CoM frame
	for (size_t i=0; i<4; i++)
		if (stanceLegs_[i]==true || constrainedIntegration_==false)
			kinematicModelPtr_->footPositionBaseFrame(i, com_base2StanceFeet_[i]);
		else
			com_base2StanceFeet_[i].setZero();

	// Inertia matrix in the CoM frame and its derivatives
	M_    = comModelPtr_->comInertia(qJoints_);
	dMdt_ = comModelPtr_->comInertiaDerivative(qJoints_, dqJoints_);
	Eigen::Matrix3d rotationMInverse = M_.topLeftCorner<3,3>().inverse();
	MInverse_ << rotationMInverse, Eigen::Matrix3d::Zero(),
			Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Identity()/M_(5,5);

	// Coriolis and centrifugal forces
	C_.head<3>() = com_W_com.cross(M_.topLeftCorner<3,3>()*com_W_com) + dMdt_.topLeftCorner<3,3>()*com_W_com;
	C_.tail<3>().setZero();

	// gravity effect on CoM in CoM coordinate
	MInverseG_ << Eigen::Vector3d::Zero(), -o_R_b.transpose() * o_gravityVector_;

	// CoM Jacobin in the Base frame
	b_comJacobainOmega_ = ( MInverse_*comModelPtr_->comMomentumJacobian(qJoints_) ).template topRows<3>();

	// contact JacobianTransposeLambda
	Eigen::Vector3d com_comToFoot;
	Eigen::Matrix<double, 6, 1> JcTransposeLambda(Eigen::VectorXd::Zero(6));
	for (size_t i=0; i<4; i++)  {

		// for a swing leg skip the followings
		if (stanceLegs_[i]==false && constrainedIntegration_==true) continue;

		com_comToFoot  = com_base2StanceFeet_[i]-com_base2CoM_;

		JcTransposeLambda.head<3>() += com_comToFoot.cross(u.segment<3>(3*i));
		JcTransposeLambda.tail<3>() += u.segment<3>(3*i);
	}

	// angular velocities to Euler angle derivatives transformation
	Eigen::Matrix3d transformAngVel2EulerAngDev = AngularVelocitiesToEulerAngleDerivativesMatrix(x.head<3>());

	// CoM dynamics
	dxdt.segment<3>(0) = o_R_b * transformAngVel2EulerAngDev * (com_W_com - b_comJacobainOmega_*dqJoints_);
	dxdt.segment<3>(3) = o_R_b * com_V_com;
	dxdt.tail<6>() = MInverse_ * (-C_ + JcTransposeLambda) - MInverseG_;

}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void ComDynamicsBase<JOINT_COORD_SIZE>::calculateBasePose(const joint_coordinate_t& qJoints,
		const base_coordinate_t& comPose,
		base_coordinate_t& basePose) {

	// Rotation matrix from Base frame (or the coincided frame world frame) to Origin frame (global world).
	Eigen::Matrix3d o_R_b = SwitchedModel<JOINT_COORD_SIZE>::RotationMatrixBasetoOrigin(comPose.template head<3>());

	// base to CoM displacement in the CoM frame
	Eigen::Vector3d com_base2CoM = comModelPtr_->comPositionBaseFrame(qJoints);

	// base coordinate
	basePose.template head<3>() = comPose.template segment<3>(0);
	basePose.template tail<3>() = comPose.template segment<3>(3) - o_R_b * com_base2CoM;
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void ComDynamicsBase<JOINT_COORD_SIZE>::calculateBaseLocalVelocities(const joint_coordinate_t& qJoints,
		const joint_coordinate_t& dqJoints,
		const base_coordinate_t& comLocalVelocities,
		base_coordinate_t& baseLocalVelocities) {

	// base to CoM displacement in the CoM frame
	Eigen::Vector3d com_base2CoM = comModelPtr_->comPositionBaseFrame(qJoints);

	// CoM Jacobin in the Base frame
	Eigen::Matrix<double,6,12> b_comJacobain = comModelPtr_->comJacobainBaseFrame(qJoints);

	// local velocities of Base (com_W_b)
	baseLocalVelocities.template head<3>() = comLocalVelocities.template head<3>() - b_comJacobain.template topRows<3>()*dqJoints;
	baseLocalVelocities.template tail<3>() = comLocalVelocities.template tail<3>() - b_comJacobain.template bottomRows<3>()*dqJoints
			+ com_base2CoM.cross(baseLocalVelocities.template head<3>());

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/*
 * @brief Computes the matrix which transforms derivatives of angular velocities in the body frame to euler angles derivatives
 * WARNING: matrix is singular when rotation around y axis is +/- 90 degrees
 * @param[in] eulerAngles: euler angles in xyz convention
 * @return M: matrix that does the transformation
 */
template <size_t JOINT_COORD_SIZE>
Eigen::Matrix3d ComDynamicsBase<JOINT_COORD_SIZE>::AngularVelocitiesToEulerAngleDerivativesMatrix (Eigen::Vector3d eulerAngles){

	Eigen::Matrix<double,3,3> M;
	double sinPsi = sin(eulerAngles(2));
	double cosPsi = cos(eulerAngles(2));
	double sinTheta = sin(eulerAngles(1));
	double cosTheta = cos(eulerAngles(1));

	M << 	cosPsi/cosTheta,          -sinPsi/cosTheta,          0,
			sinPsi, 				   cosPsi,                   0,
		   -cosPsi*sinTheta/cosTheta,  sinTheta*sinPsi/cosTheta, 1;

	return M;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void ComDynamicsBase<JOINT_COORD_SIZE>::getStanceLegs(std::array<bool,4>& stanceLegs) const {
	stanceLegs = stanceLegs_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void ComDynamicsBase<JOINT_COORD_SIZE>::getFeetPositions(std::array<Eigen::Vector3d,4>& b_base2StanceFeet) const {
	b_base2StanceFeet = com_base2StanceFeet_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
Eigen::Matrix<double, 6, 6>  ComDynamicsBase<JOINT_COORD_SIZE>::getM() const {
	return M_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
Eigen::Matrix<double, 6, 1>  ComDynamicsBase<JOINT_COORD_SIZE>::getC() const {
	return C_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
Eigen::Matrix<double, 6, 1>  ComDynamicsBase<JOINT_COORD_SIZE>::getG() const {
	return M_*MInverseG_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
Eigen::Matrix<double, 6, 6>  ComDynamicsBase<JOINT_COORD_SIZE>::getMInverse() const {
	return MInverse_;
}

} //end of namespace switched_model
