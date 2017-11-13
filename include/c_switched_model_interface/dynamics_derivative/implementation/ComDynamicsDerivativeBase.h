/*
 * ComDynamicsDerivativeBase.h
 *
 *  Created on: Nov 10, 2017
 *      Author: farbod
 */

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
std::shared_ptr<typename ComDynamicsDerivativeBase<JOINT_COORD_SIZE>::Base>
	ComDynamicsDerivativeBase<JOINT_COORD_SIZE>::ComDynamicsDerivativeBase::clone()  const {

	return std::allocate_shared< ComDynamicsDerivativeBase<JOINT_COORD_SIZE>, Eigen::aligned_allocator<ComDynamicsDerivativeBase<JOINT_COORD_SIZE>> >(
			Eigen::aligned_allocator<ComDynamicsDerivativeBase<JOINT_COORD_SIZE>>(), *this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void ComDynamicsDerivativeBase<JOINT_COORD_SIZE>::initializeModel(const std::vector<size_t>& systemStockIndexes, const std::vector<scalar_t>& switchingTimes,
		const state_vector_t& initState, const size_t& activeSubsystemIndex/*=0*/, const char* algorithmName/*=NULL*/)  {

	Base::initializeModel(systemStockIndexes, switchingTimes, initState, activeSubsystemIndex, algorithmName);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void ComDynamicsDerivativeBase<JOINT_COORD_SIZE>::setData(const std::array<bool,4>& stanceLegs,
		const joint_coordinate_t& qJoints,
		const joint_coordinate_t& dqJoints)  {

	stanceLegs_ = stanceLegs;
	qJoints_  = qJoints;
	dqJoints_ = dqJoints;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void ComDynamicsDerivativeBase<JOINT_COORD_SIZE>::setCurrentStateAndControl(const scalar_t& t,
		const state_vector_t& x,
		const control_vector_t& u)  {

	Base::setCurrentStateAndControl(t, x, u);

	// Rotation matrix from Base frame (or the coincided frame world frame) to Origin frame (global world).
	o_R_b_ = SwitchedModel<JOINT_COORD_SIZE>::RotationMatrixBasetoOrigin(x.head<3>());

	// base to CoM displacement in the CoM frame
	com_base2CoM_ = comModelPtr_->comPositionBaseFrame(qJoints_);

	// base coordinate
	q_base_.template head<3>() = x.segment<3>(0);
	q_base_.template tail<3>() = x.segment<3>(3) - o_R_b_ * com_base2CoM_;

	// update kinematic model
	kinematicModelPtr_->update(q_base_, qJoints_);

	for (size_t i=0; i<4; i++)  {

		// base to stance feet displacement in the CoM frame
		kinematicModelPtr_->footPositionBaseFrame(i, com_base2StanceFeet_[i]);
		com_com2StanceFeet_[i]  = com_base2StanceFeet_[i]-com_base2CoM_;

		// feet Jacobain's in the Base frame
		kinematicModelPtr_->footJacobainBaseFrame(i, b_feetJacobains_[i]);
	}  // end of i loop

	// Inertia matrix in the CoM frame and its derivatives
	M_ = comModelPtr_->comInertia(qJoints_);
	dMdt_ = (useInertiaMatrixDerivate_==true) ? comModelPtr_->comInertiaDerivative(qJoints_, dqJoints_)
			: Eigen::Matrix<double,6,6>::Zero();
	Eigen::Matrix3d rotationMInverse = M_.topLeftCorner<3,3>().inverse();
	MInverse_ << rotationMInverse, Eigen::Matrix3d::Zero(),
			Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Identity()/M_(5,5);

	// CoM Jacobin in the Base frame
	b_comJacobain_ = MInverse_*comModelPtr_->comMomentumJacobian(qJoints_);

	// local angular velocity of Base (w_W_base_)
	w_W_base_ = x.segment<3>(6) - b_comJacobain_.template topRows<3>()*dqJoints_;

	// jacobina of angular velocities to Euler angle derivatives transformation
	jacobianOfAngularVelocityMapping_ = JacobianOfAngularVelocityMapping(x.head<3>(), w_W_base_).transpose();

	// CoM Jacobin time derivative in the Base frame
//	const double h = sqrt(Eigen::NumTraits<double>::epsilon());
//	joint_coordinate_t qJointsPlus = qJoints_ + dqJoints_*h;
//	b_comJacobainTimeDerivative_ = (ComInertia(qJointsPlus).inverse() * ComMomentumJacobian(qJointsPlus) - b_comJacobain_)/h;
	if (useInertiaMatrixDerivate_==true)
		b_comJacobainTimeDerivative_ = MInverse_*comModelPtr_->comMomentumJacobianDerivative(qJoints_, dqJoints_) - MInverse_*dMdt_*b_comJacobain_;
	else
		b_comJacobainTimeDerivative_.setZero();

	// derivative of the Inertia matrix w.r.t. qJoints
	for (size_t j=0; j<12; j++)
		if (useInertiaMatrixDerivate_==true) {
			joint_coordinate_t dqdt = joint_coordinate_t::Zero();
			dqdt(j) = 1;
			partialM_[j] = comModelPtr_->comInertiaDerivative(qJoints_, dqdt) ;
		}  else
			partialM_[j].setZero();
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void ComDynamicsDerivativeBase<JOINT_COORD_SIZE>::getDerivativeState(state_matrix_t& A)  {

	// local angular velocity (w_W_com) and local linear velocity (w_V_com) of CoM
	Eigen::VectorBlock<state_vector_t,3> com_W_com = x_.segment<3>(6);
	Eigen::VectorBlock<state_vector_t,3> com_V_com = x_.segment<3>(9);

	// A matrix
	/*			o_theta_b	o_x_com		com_w_com	com_v_com
	 * 		|	A00			A01			A02			A03		|
	 * A = 	|	A10			A11			A12			A13		|
	 *		|	A20			A21			A22			A23		|
	 * 		|	A30			A31			A32			A33		|
	 */

	// first three rows
	A.block<3,3>(0,0) = jacobianOfAngularVelocityMapping_.block<3,3>(0,0);
	A.block<3,3>(0,3) = Eigen::Matrix3d::Zero();
	A.block<3,3>(0,6) = jacobianOfAngularVelocityMapping_.block<3,3>(0,3);
	A.block<3,3>(0,9) = Eigen::Matrix3d::Zero();

	// second three rows
	A.block<3,3>(3,0) = -SwitchedModel<JOINT_COORD_SIZE>::CrossProductMatrix(o_R_b_*com_V_com);
	A.block<3,3>(3,3) = Eigen::Matrix3d::Zero();
	A.block<3,3>(3,6) = Eigen::Matrix3d::Zero();
	A.block<3,3>(3,9) = o_R_b_;

	// third three rows
	A.block<3,3>(6,0) = Eigen::Matrix3d::Zero();
	A.block<3,3>(6,3) = Eigen::Matrix3d::Zero();
	A.block<3,3>(6,6) = MInverse_.topLeftCorner<3,3>() * (
			SwitchedModel<JOINT_COORD_SIZE>::CrossProductMatrix(M_.topLeftCorner<3,3>()*com_W_com) -
			SwitchedModel<JOINT_COORD_SIZE>::CrossProductMatrix(com_W_com)*M_.topLeftCorner<3,3>() -
			dMdt_.topLeftCorner<3,3>() );
	A.block<3,3>(6,9) = Eigen::Matrix3d::Zero();

	// fourth three rows
	A.block<3,3>(9,0) = o_R_b_.transpose() * SwitchedModel<JOINT_COORD_SIZE>::CrossProductMatrix(o_gravityVector_);
	A.block<3,3>(9,3) = Eigen::Matrix3d::Zero();
	A.block<3,3>(9,6) = Eigen::Matrix3d::Zero();
	A.block<3,3>(9,9) = Eigen::Matrix3d::Zero();
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void ComDynamicsDerivativeBase<JOINT_COORD_SIZE>::getDerivativesControl(control_gain_matrix_t& B)  {


	// B matrix
	/*			lambda_LF	lambda_RF	lambda_LH	lambda_RH
	 * 		|	Zero		Zero		Zero		Zero	|
	 * B = 	|	Zero		Zero		Zero		Zero	|
	 *		|	B20			B21			B22			B23		|
	 * 		|	B30			B31			B32			B33		|
	 */

	// first 6 rows
	B.topRows<6>().setZero();;

	for (size_t i=0; i<4; i++)  {

		// for swing legs set the corresponding block to zero
		if (stanceLegs_[i]==false && constrainedIntegration_==true) {
			// third and fourth three rows and ith three columns
			B.block<6,3>(6,i*3).setZero();
			continue;
		}

		// third three rows and ith three columns
		B.block<3,3>(6,3*i) = SwitchedModel<JOINT_COORD_SIZE>::CrossProductMatrix(com_com2StanceFeet_[i]);

		// fourth three rows and ith three columns
		B.block<3,3>(9,3*i).setIdentity();
	}  // end of i loop

	B.bottomRows<6>() = (MInverse_*B.bottomRows<6>()).eval();
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void ComDynamicsDerivativeBase<JOINT_COORD_SIZE>::getApproximateDerivativesJoint(state_joint_matrix_t& partrialF_q)  {

	// first three rows
	partrialF_q.template block<3,12>(0,0) = -o_R_b_ * b_comJacobainTimeDerivative_.template topRows<3>();

	// second three rows
	partrialF_q.template block<3,12>(3,0).setZero();

	// third and fourth three rows
	partrialF_q.template block<3,12>(6,0).setZero();
	partrialF_q.template block<3,12>(9,0).setZero();

	for (size_t i=0; i<4; i++)  {
		// skip the followings if it is a swing leg
		if (stanceLegs_[i]==false && constrainedIntegration_==true)  continue;

		// Com to foot displacement Jacobian
		Eigen::Matrix<double,3,12> b_com2FootJacobain = b_feetJacobains_[i].template bottomRows<3>()-b_comJacobain_.template bottomRows<3>();

		// partila_q ([r]*lamda)
		partrialF_q.template block<3,12>(6,0) -= (SwitchedModel<JOINT_COORD_SIZE>::CrossProductMatrix(u_.segment<3>(3*i)) * b_com2FootJacobain);
	}

	// partila_q ([W]*I*W)
	if (useInertiaMatrixDerivate_==true)
		for (size_t j=0; j<12; j++)
			partrialF_q.template block<3,1>(6,j) -= SwitchedModel<JOINT_COORD_SIZE>::CrossProductMatrix(x_.segment<3>(6)) * partialM_[j].topLeftCorner<3,3>() * x_.segment<3>(6);

	partrialF_q.template block<6,12>(6,0) = ( MInverse_ * partrialF_q.block<6,12>(6,0) );

	/* partial derivative of the MInverse w.r.t. qJoints */
	if (useInertiaMatrixDerivate_==true) {
		static ComDynamicsBase<JOINT_COORD_SIZE> comDyamics(kinematicModelPtr_, comModelPtr_,
				-o_gravityVector_(2), constrainedIntegration_);
		state_vector_t dxdt;
		comDyamics.setData(stanceLegs_, qJoints_, dqJoints_);
		comDyamics.computeDerivative(t_, x_, u_, dxdt);

		Eigen::Matrix<double,6,12> partial;
		for (size_t j=0; j<12; j++)
			partial.col(j) = -MInverse_ * partialM_[j] * dxdt.tail<6>();

		partrialF_q.template bottomRows<6>() += partial;
	}
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void ComDynamicsDerivativeBase<JOINT_COORD_SIZE>::getNumericalDerivativesJoint(state_joint_matrix_t& partrialF_q)  {

	static ComDynamicsBase<JOINT_COORD_SIZE> comDyamics(kinematicModelPtr_, comModelPtr_,
			-o_gravityVector_(2), constrainedIntegration_);
	state_vector_t dxdt;
	comDyamics.setData(stanceLegs_, qJoints_, dqJoints_);
	comDyamics.computeDerivative(t_, x_, u_, dxdt);

	Eigen::Matrix<double,6,12> partial;
	for (size_t j=0; j<12; j++) {
		double h = sqrt(Eigen::NumTraits<double>::epsilon()) * std::max(fabs(qJoints_(j)), 1.0);

		joint_coordinate_t qJointsPlus = qJoints_;
		qJointsPlus(j) += h;

		comDyamics.setData(stanceLegs_, qJointsPlus, dqJoints_);
		state_vector_t dxdtPlus;
		comDyamics.computeDerivative(t_, x_, u_, dxdtPlus);

		partrialF_q.col(j) = (dxdtPlus-dxdt)/h;
	}  // end of j loop
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void ComDynamicsDerivativeBase<JOINT_COORD_SIZE>::getApproximateDerivativesJointVelocity(state_joint_matrix_t& partrialF_dq)  {

	// first three rows
//	partrialF_dq.block<3,12>(0,0) = -o_R_b_ * b_comJacobain_.topRows<3>();
	partrialF_dq.template block<3,12>(0,0) = -jacobianOfAngularVelocityMapping_.block<3,3>(0,3) * b_comJacobain_.template topRows<3>();

	// second three rows
	partrialF_dq.template block<3,12>(3,0).setZero();

	// third three rows
	partrialF_dq.template block<3,12>(6,0).setZero();
	if (useInertiaMatrixDerivate_==true)
		for (size_t j=0; j<12; j++)
			partrialF_dq.template block<3,1>(6,j) = -MInverse_.template topLeftCorner<3,3>() * partialM_[j].template topLeftCorner<3,3>() * x_.template segment<3>(6);

	// fourth three rows
	partrialF_dq.template block<3,12>(9,0).setZero();
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/*
 * to map local angular velocity \omega_W expressed in body coordinates, to changes in Euler Angles expressed in an inertial frame q_I
 * we have to map them via \dot{q}_I = H \omega_W, where H is the matrix defined in kindr getMappingFromLocalAngularVelocityToDiff.
 * You can see the kindr cheat sheet to figure out how to build this matrix. The following code computes the Jacobian of \dot{q}_I
 * with respect to \q_I and \omega_W. Thus the lower part of the Jacobian is H and the upper part is dH/dq_I \omega_W. We include
 * both parts for more efficient computation. The following code is computed using auto-diff.
 * @param eulerAnglesXyz
 * @param angularVelocity
 * @return
 */
template <size_t JOINT_COORD_SIZE>
Eigen::Matrix<double,6,3> ComDynamicsDerivativeBase<JOINT_COORD_SIZE>::JacobianOfAngularVelocityMapping(
		const Eigen::Vector3d& eulerAnglesXyz, const Eigen::Vector3d& angularVelocity) {

	using namespace std;

	Eigen::Matrix<double,6,1> xAD;
	xAD << eulerAnglesXyz, angularVelocity;
	const double* x = xAD.data();

	std::array<double,10> v;

	Eigen::Matrix<double,6,3> jac;
	double* y = jac.data();

	y[9] = sin(x[2]);
	y[10] = cos(x[2]);
	v[0] = cos(x[1]);
	v[1] = 1 / v[0];
	y[3] = v[1] * y[10];
	v[2] = sin(x[1]);
	y[1] = 0 - (0 - (0 - x[4] * y[9] + x[3] * y[10]) * 1 / v[0] * v[1]) * v[2];
	v[3] = sin(x[2]);
	v[4] = 0 - v[1];
	y[4] = v[4] * y[9];
	v[5] = y[10];
	y[2] = 0 - x[3] * v[1] * v[3] + x[4] * v[4] * v[5];
	y[8] = 0 - x[4] * v[3] + x[3] * v[5];
	v[6] = v[1] * y[9];
	v[7] = v[4] * y[10];
	v[8] = v[2];
	y[15] = v[7] * v[8];
	y[16] = v[6] * v[8];
	v[9] = x[4] * v[8];
	v[8] = x[3] * v[8];
	y[13] = (x[4] * v[6] + x[3] * v[7]) * v[0] - (0 - (v[9] * y[9] - v[8] * y[10]) * 1 / v[0] * v[1]) * v[2];
	y[14] = 0 - v[8] * v[4] * v[3] + v[9] * v[1] * v[5];
	// dependent variables without operations
	y[0] = 0;
	y[5] = 0;
	y[6] = 0;
	y[7] = 0;
	y[11] = 0;
	y[12] = 0;
	y[17] = 1;


	return jac;
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void ComDynamicsDerivativeBase<JOINT_COORD_SIZE>::getBase2CoMInComFrame(Eigen::Vector3d& com_base2CoM) const {
	com_base2CoM = com_base2CoM_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void ComDynamicsDerivativeBase<JOINT_COORD_SIZE>::getBasePose(Eigen::Vector3d& basePose) const {
	basePose = q_base_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void ComDynamicsDerivativeBase<JOINT_COORD_SIZE>::getCom2StanceFeetInComFrame(
		std::array<Eigen::Vector3d,4>& com_com2StanceFeet) const {
	com_com2StanceFeet = com_com2StanceFeet_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void ComDynamicsDerivativeBase<JOINT_COORD_SIZE>::getBase2StanceFeetInComFrame(
		std::array<Eigen::Vector3d,4>& com_base2StanceFeet) const {
	com_base2StanceFeet = com_base2StanceFeet_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void ComDynamicsDerivativeBase<JOINT_COORD_SIZE>::getComJacobianInBaseFrame(
		base_jacobian_matrix_t& b_comJacobain) const {
	b_comJacobain = b_comJacobain_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void ComDynamicsDerivativeBase<JOINT_COORD_SIZE>::getFeetJacobiansInBaseFrame(
		std::array<base_jacobian_matrix_t,4>& b_feetJacobains) const {
	b_feetJacobains = b_feetJacobains_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void ComDynamicsDerivativeBase<JOINT_COORD_SIZE>::getComJacobianTimeDerivativeInBaseFrame(
		base_jacobian_matrix_t& b_comJacobainTimeDerivative) const {
	b_comJacobainTimeDerivative = b_comJacobainTimeDerivative_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void ComDynamicsDerivativeBase<JOINT_COORD_SIZE>::getRotationMatrixBasetoOrigin(Eigen::Matrix3d& o_R_b) const {
	o_R_b = o_R_b_;
}

