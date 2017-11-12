/*
 * ComKinoDynamicsDerivativeBase.h
 *
 *  Created on: Nov 12, 2017
 *      Author: farbod
 */

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
virtual std::shared_ptr<ComKinoDynamicsDerivativeBase<JOINT_COORD_SIZE>::Base> ComKinoDynamicsDerivativeBase<JOINT_COORD_SIZE>::clone() {

	return std::allocate_shared< ComKinoDynamicsDerivativeBase<JOINT_COORD_SIZE>, Eigen::aligned_allocator<ComKinoDynamicsDerivativeBase<JOINT_COORD_SIZE>> > (
			Eigen::aligned_allocator<ComKinoDynamicsDerivativeBase<JOINT_COORD_SIZE>>(), *this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void ComKinoDynamicsDerivativeBase<JOINT_COORD_SIZE>::initializeModel(const std::vector<size_t>& systemStockIndexes, const std::vector<scalar_t>& switchingTimes,
		const state_vector_t& initState, const size_t& activeSubsystemIndex/*=0*/, const char* algorithmName/*=NULL*/)  {

	Base::initializeModel(systemStockIndexes, switchingTimes, initState, activeSubsystemIndex, algorithmName);
	comDynamicsDerivative_.initializeModel(systemStockIndexes, switchingTimes, initState.head<12>(), activeSubsystemIndex, algorithmName);

	// use the provided planner to get the swing legs CPG
	if (feetZDirectionPlanner_) {
		feetZDirectionPlanner_->planSingleMode(systemStockIndexes, switchingTimes, activeSubsystemIndex, feetZDirectionCPGs_);

		// find the next phase stanceLegs
		std::vector<std::array<size_t,4> > finalTimesIndices;
		feetZDirectionPlanner_->getFinalTimesIndices(finalTimesIndices);
		for (size_t j=0; j<4; j++)  nextPhaseStanceLegs_[j] = (finalTimesIndices[activeSubsystemIndex][j] == activeSubsystemIndex+1);
	}

	algorithmName_.assign(algorithmName);
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void ComKinoDynamicsDerivativeBase<JOINT_COORD_SIZE>::setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x,
		const control_vector_t& u)  {

	Base::setCurrentStateAndControl(t, x, u);

	// HyQ's joints
	qJoints_  = x.template tail<12>();
	// HyQ's joints' velocities
	dqJoints_ = u.template tail<12>();
	// HyQ's Base local velocities
	baseLocalVelocities_ = x.template segment<6>(6);

	// set CoM data
	comDynamicsDerivative_.setData(stanceLegs_, qJoints_, dqJoints_);
	comDynamicsDerivative_.setCurrentStateAndControl(t, x.template head<12>(), u.template head<12>());

	// Rotation matrix from Base frame to Origin frame (global world)
	comDynamicsDerivative_.getRotationMatrixBasetoOrigin(o_R_b_);



	// Base to CoM displacement in the CoM frame
	comDynamicsDerivative_.getBase2CoMInComFrame(com_base2CoM_);
	// CoM Jacobian in the Base frame
	comDynamicsDerivative_.getComJacobianInBaseFrame(b_comJacobain_);
	// CoM Jacobin time derivative in the Base frame
	comDynamicsDerivative_.getComJacobianTimeDerivativeInBaseFrame(b_comJacobainTimeDerivative_);
	// CoM to feet in the CoM frame
	comDynamicsDerivative_.getCom2StanceFeetInComFrame(com_com2StanceFeet_);
	// Feet Jacobian in the Base frame
	comDynamicsDerivative_.getFeetJacobiansInBaseFrame(b_feetJacobains_);

	// feet Jacobins time derivative in the Base frame
	const double h = sqrt(Eigen::NumTraits<double>::epsilon());
	joints_coordinate_t qJointsPlus = qJoints_ + dqJoints_*h;
	for (size_t i=0; i<4; i++) {
		base_jacobian_matrix_t b_footJacobainPlus;
		SwitchedModelKinematics::FootJacobainBaseFrame(qJointsPlus, i, b_footJacobainPlus);
		b_feetJacobainsTimeDerivative_[i] = (b_footJacobainPlus-b_feetJacobains_[i])/h;
	}  // end of i loop

	// jacobina of angular velocities to Euler angle derivatives transformation
	jacobianOfAngularVelocityMapping_ = JacobianOfAngularVelocityMapping(x.segment<3>(0), x.segment<3>(6)).transpose();



	for (size_t i=0; i<4; i++) {
		// CoM to feet in the CoM frame
		SwitchedModelKinematics::FootPositionBaseFrame(qJoints_, i, b_base2StanceFeet_[i]);
		// Feet Jacobian in the Base frame
		SwitchedModelKinematics::FootJacobainBaseFrame(qJoints_, i, b_feetJacobains_[i]);

		// feet Jacobins time derivative in the Base frame
		base_jacobian_matrix_t b_footJacobainPlus;
		SwitchedModelKinematics::FootJacobainBaseFrame(qJointsPlus, i, b_footJacobainPlus);
		b_feetJacobainsTimeDerivative_[i].setZero();
		b_feetJacobainsTimeDerivative_[i].block<3,3>(0,3*i) = (b_footJacobainPlus.block<3,3>(3,3*i)-b_feetJacobains_[i].block<3,3>(3,3*i))/h;
	}  // end of i loop


	// if GapIndicator is provided
	if (endEffectorStateConstraints_.empty()==false)
		for (size_t i=0; i<4; i++)  {
			// check for the gaps
			feetConstraintIsActive_[i] = false;

			if (stanceLegs_[i]==false && nextPhaseStanceLegs_[i]==true)  {
				// calculate the the foot position in the Origin frame.
				Eigen::Vector3d o_origin2StanceFoot = x.segment<3>(3) + o_R_b_ * b_base2StanceFeet_[i];

				feetConstraintJacobains_[i].setZero();
				for (size_t j=0; j<endEffectorStateConstraints_.size(); j++) {

					if (endEffectorStateConstraints_[j]->isActive(o_origin2StanceFoot)==false)  continue;

					double constraintValue = endEffectorStateConstraints_[j]->constraintValue(o_origin2StanceFoot);
					if (constraintValue>std::numeric_limits<double>::epsilon()) {
						feetConstraintIsActive_[i]  = true;
						feetConstraintJacobains_[i] = endEffectorStateConstraints_[j]->constraintDerivative(o_origin2StanceFoot);
						break;
					}  // end of if
				}  // end of j loop

			}  // end of if
		}  // end of i loop

}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/*
 * calculate and retrieve the A matrix
 */
template <size_t JOINT_COORD_SIZE>
void ComKinoDynamicsDerivativeBase<JOINT_COORD_SIZE>::getDerivativeState(state_matrix_t& A)  {

	// A matrix
	/*			com		qJoints
	 * A = 	|	A00			A01	|
	 *		|	0			0	|
	 */

	// CoM derivative with respect to CoM
	Eigen::Matrix<double,12,12> comAcom;

	// first three rows
	comAcom.block<3,3>(0,0) = jacobianOfAngularVelocityMapping_.leftCols<3>();
	comAcom.block<3,3>(0,3) = Eigen::Matrix3d::Zero();
	comAcom.block<3,3>(0,6) = jacobianOfAngularVelocityMapping_.rightCols<3>();
	comAcom.block<3,3>(0,9) = Eigen::Matrix3d::Zero();

	// second three rows
	comAcom.block<3,3>(3,0) = -SwitchedModelKinematics::CrossProductMatrix(o_R_b_*baseLocalVelocities_.tail<3>());
	comAcom.block<3,3>(3,3) = Eigen::Matrix3d::Zero();
	comAcom.block<3,3>(3,6) = Eigen::Matrix3d::Zero();
	comAcom.block<3,3>(3,9) = o_R_b_;

	// third three rows
	comAcom.block<3,3>(6,0) = Eigen::Matrix3d::Zero();
	comAcom.block<3,3>(6,3) = Eigen::Matrix3d::Zero();
	comAcom.block<3,3>(6,6) = MInverse_.topLeftCorner<3,3>() * (
			SwitchedModelKinematics::CrossProductMatrix(M_.topLeftCorner<3,3>()*baseLocalVelocities_.head<3>()) -
			SwitchedModelKinematics::CrossProductMatrix(baseLocalVelocities_.head<3>())*M_.topLeftCorner<3,3>() );
	comAcom.block<3,3>(6,9) = Eigen::Matrix3d::Zero();

	// fourth three rows
	comAcom.block<3,3>(9,0) = o_R_b_.transpose() * SwitchedModelKinematics::CrossProductMatrix(o_gravityVector_);
	comAcom.block<3,3>(9,3) = Eigen::Matrix3d::Zero();
	comAcom.block<3,3>(9,6) = Eigen::Matrix3d::Zero();
	comAcom.block<3,3>(9,9) = Eigen::Matrix3d::Zero();

	// CoM derivative with respect to qJoints
	Eigen::Matrix<double,12,12> comAjoints;
	comAjoints.setZero();
	for (size_t i=0; i<4; i++)
		comAjoints.block<3,3>(6,3*i) = -MInverse_.topLeftCorner<3,3>() * SwitchedModelKinematics::CrossProductMatrix(u_.segment<3>(3*i))
						* b_feetJacobains_[i].block<3,3>(3,3*i);

	A << comAcom,    							comAjoints,
		 Eigen::Matrix<double,12,12>::Zero(), 	Eigen::Matrix<double,12,12>::Zero();
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/*
 * calculate and retrieve the B matrix
 */
template <size_t JOINT_COORD_SIZE>
void ComKinoDynamicsDerivativeBase<JOINT_COORD_SIZE>::getDerivativesControl(control_gain_matrix_t& B)  {

	// B matrix
	/*			lambda		jointsVel=omega
	 * B = 	|	B00			B01	|
	 *		|	B10			B11	|
	 */
	// CoM derivative with respect to lambdas
	Eigen::Matrix<double,12,12> comBlambda;
	// first 6 rows
	comBlambda.topRows<6>().setZero();
	// second 6 rows
	for (size_t i=0; i<4; i++)  {

		// for swing legs set the corresponding block to zero
		if (stanceLegs_[i]==false && options_.constrainedIntegration_==true) {
			// third and fourth three rows and ith three columns
			comBlambda.block<6,3>(6,i*3).setZero();
			continue;
		}

		// third three rows and ith three columns
		comBlambda.block<3,3>(6,3*i) = SwitchedModelKinematics::CrossProductMatrix(b_base2StanceFeet_[i]);

		// fourth three rows and ith three columns
		comBlambda.block<3,3>(9,3*i).setIdentity();
	}  // end of i loop

	comBlambda.bottomRows<6>() = (MInverse_*comBlambda.bottomRows<6>()).eval();

	B << comBlambda,    						Eigen::Matrix<double,12,12>::Zero(),
		 Eigen::Matrix<double,12,12>::Zero(), 	Eigen::Matrix<double,12,12>::Identity();

}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/*
 * calculate and retrieve the C matrix
 */
template <size_t JOINT_COORD_SIZE>
void ComKinoDynamicsDerivativeBase<JOINT_COORD_SIZE>::getConstraint1DerivativesState(constraint1_state_matrix_t& C) {


	// C matrix
	/*			o_theta_b	o_x_com		com_w_com	com_v_com	qJoints
	 * 		|	C00			C01			C02			C03			C04		|
	 * 		|	C10			C11			C12			C13			C14		|
	 * 	C =	|	C20			C21			C22			C23			C24		|
	 * 		|	C30			C31			C32			C33			C34		|
	 * 		|	0			0			0			0			dJz/dt	|
	 */

	size_t nextFreeIndex = 0;
	for (size_t i=0; i<4; i++) {

		// for a swing leg
		if (stanceLegs_[i]==false) {
			C.block<3,24>(nextFreeIndex,0).setZero();
			nextFreeIndex += 3;
			continue;
		}

		// Ci0
		C.block<3,3>(nextFreeIndex,0).setZero();

		// Ci1
		C.block<3,3>(nextFreeIndex,3).setZero();

		// Ci2
		C.block<3,3>(nextFreeIndex,6) = -SwitchedModelKinematics::CrossProductMatrix(b_base2StanceFeet_[i]);

		// Ci3
		C.block<3,3>(nextFreeIndex,9).setIdentity();

		// Ci4
		C.block<3,12>(nextFreeIndex,12) = SwitchedModelKinematics::CrossProductMatrix(baseLocalVelocities_.head<3>()) * (
						b_feetJacobains_[i].bottomRows<3>())
						+ b_feetJacobainsTimeDerivative_[i];

		nextFreeIndex += 3;
	}

	// for the swing legs z direction constraints, if its CPG is provided
	for (size_t i=0; i<4; i++)
		if (stanceLegs_[i]==false && feetZDirectionCPGs_[i]!=NULL) {
			Eigen::Matrix<double,3,24> partial_x;

			// Ci0
			Eigen::Vector3d o_footVelocity = o_R_b_ * ( b_feetJacobains_[i].bottomRows<3>()*dqJoints_ + baseLocalVelocities_.tail<3>()
					+ baseLocalVelocities_.head<3>().cross(b_base2StanceFeet_[i]) );
			partial_x.block<3,3>(0,0) = -SwitchedModelKinematics::CrossProductMatrix(o_footVelocity);

			// Ci1
			partial_x.block<3,3>(0,3).setZero();

			// Ci2
			partial_x.block<3,3>(0,6) = -o_R_b_ * SwitchedModelKinematics::CrossProductMatrix(b_base2StanceFeet_[i]);

			// Ci3
			partial_x.block<3,3>(0,9) = o_R_b_;

			// Ci4
			partial_x.block<3,12>(0,12) = o_R_b_ * ( SwitchedModelKinematics::CrossProductMatrix(baseLocalVelocities_.head<3>()) * (
					b_feetJacobains_[i].bottomRows<3>())
					+ b_feetJacobainsTimeDerivative_[i] );

			C.block<1,24>(nextFreeIndex,0) = options_.zDirectionVelocityWeight_*partial_x.bottomRows<1>();
			nextFreeIndex++;
		}

}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/*
 * calculate and retrieve the D matrix
 */
template <size_t JOINT_COORD_SIZE>
void ComKinoDynamicsDerivativeBase<JOINT_COORD_SIZE>::getConstraint1DerivativesControl(constraint1_control_matrix_t& D) {

	// D matrix
	/*			lambda		dqJoints
	 * 		|	D00			D01		|
	 * 		|	D10			D11		|
	 * D = 	|	D20			D21		|
	 * 		|	D30			D31		|
	 * 		|	0			Jz		|
	 */

	size_t nextFreeIndex = 0;
	for (size_t i=0; i<4; i++) {

		// for swing leg
		if (stanceLegs_[i]==false) {
			D.block<3,24>(nextFreeIndex,0).setZero();
			D.block<3,3>(nextFreeIndex,nextFreeIndex) = options_.contactForceWeight_*Eigen::Matrix3d::Identity();
			nextFreeIndex += 3;
			continue;
		}

		// Di0
		D.block<3,12>(nextFreeIndex,0).setZero();

		// Di1
		D.block<3,12>(nextFreeIndex,12) = b_feetJacobains_[i].bottomRows<3>();

		nextFreeIndex += 3;
	}  // end of i loop


	// for the swing legs z direction constraints, if its CPG is provided
	for (size_t i=0; i<4; i++)
		if (stanceLegs_[i]==false && feetZDirectionCPGs_[i]!=NULL) {
			Eigen::Matrix<double,3,12> partial_dq = o_R_b_ * ( b_feetJacobains_[i].bottomRows<3>() );
			D.block<1,12>(nextFreeIndex,0).setZero();
			D.block<1,12>(nextFreeIndex,12) = options_.zDirectionVelocityWeight_*partial_dq.bottomRows<1>();
			nextFreeIndex++;
		}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/*
 * calculate and retrieve the F matrix
 */
template <size_t JOINT_COORD_SIZE>
void ComKinoDynamicsDerivativeBase<JOINT_COORD_SIZE>::getConstraint2DerivativesState(constraint2_state_matrix_t& F)  {

	size_t numConstraint2 = 0;
	if (options_.zDirectionPositionWeight_<std::numeric_limits<double>::epsilon())  return;

	for (size_t i=0; i<4; i++)
		if (stanceLegs_[i]==false && feetZDirectionCPGs_[i]!=NULL) {
			// foot jacobian in the Origin frame
			Eigen::Matrix<double, 3, 18> o_footJacobian;
			o_footJacobian.block<3,3>(0,0)  = -SwitchedModelKinematics::CrossProductMatrix(o_R_b_*b_base2StanceFeet_[i]);
			o_footJacobian.block<3,3>(0,3)  = Eigen::Matrix3d::Identity();
			o_footJacobian.block<3,12>(0,6) = o_R_b_ * (b_feetJacobains_[i].bottomRows<3>());

			F.block<1,6>(numConstraint2,0)   = options_.zDirectionPositionWeight_ * o_footJacobian.block<1,6>(2,0);
			F.block<1,6>(numConstraint2,6)   = Eigen::Matrix<double,1,6>::Zero();
			F.block<1,12>(numConstraint2,12) = options_.zDirectionPositionWeight_ * o_footJacobian.block<1,12>(2,6);
			numConstraint2++;
		}

}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/*
 * calculate and retrieve the F matrix
 */
template <size_t JOINT_COORD_SIZE>
void ComKinoDynamicsDerivativeBase<JOINT_COORD_SIZE>::getFinalConstraint2DerivativesState(constraint2_state_matrix_t& F)  {

	size_t numConstraint2 = 0;

	for (size_t i=0; i<4; i++)
		if (feetConstraintIsActive_[i]==true)  {
			// foot jacobian in the Origin frame
			Eigen::Matrix<double, 3, 18> o_footJacobian;
			o_footJacobian.block<3,3>(0,0)  = -SwitchedModelKinematics::CrossProductMatrix(o_R_b_*b_base2StanceFeet_[i]);
			o_footJacobian.block<3,3>(0,3)  = Eigen::Matrix3d::Identity();
			o_footJacobian.block<3,12>(0,6) = o_R_b_ * (b_feetJacobains_[i].bottomRows<3>());

			// gap jacobian
			Eigen::Matrix<double, 1, 18> o_gapJacobian = feetConstraintJacobains_[i].transpose() * o_footJacobian;

			F.block<1,6>(numConstraint2,0)   = o_gapJacobian.block<1,6>(0,0);
			F.block<1,6>(numConstraint2,6)   = Eigen::Matrix<double,1,6>::Zero();
			F.block<1,12>(numConstraint2,12) = o_gapJacobian.block<1,12>(0,6);

			numConstraint2++;
		}
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
Eigen::Matrix<double,6,3> ComKinoDynamicsDerivativeBase<JOINT_COORD_SIZE>::JacobianOfAngularVelocityMapping(
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


