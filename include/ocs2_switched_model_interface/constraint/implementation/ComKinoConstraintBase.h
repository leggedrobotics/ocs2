/*
 * ComKinoConstraintBase.h
 *
 *  Created on: Nov 12, 2017
 *      Author: farbod
 */


namespace switched_model {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>*
	ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::clone() const {

	return new ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::initializeModel(
		logic_rules_machine_t& logicRulesMachine,
		const size_t& partitionIndex,
		const char* algorithmName/*=NULL*/) {

	Base::initializeModel(logicRulesMachine, partitionIndex, algorithmName);

	findActiveSubsystemFnc_ = std::move( logicRulesMachine.getHandleToFindActiveEventCounter(partitionIndex) );

	logicRulesPtr_ = logicRulesMachine.getLogicRulesPtr();

	numEventTimes_ = logicRulesMachine.getLogicRulesPtr()->getNumEventTimes();

	endEffectorStateConstraintsPtr_ = logicRulesPtr_->getEndEffectorStateConstraintsPtr();

	if (algorithmName!=NULL)
		algorithmName_.assign(algorithmName);
	else
		algorithmName_.clear();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::setCurrentStateAndControl(
		const scalar_t& t,
		const state_vector_t& x,
		const input_vector_t& u) {

	Base::setCurrentStateAndControl(t, x, u);

	activeSubsystem_ = findActiveSubsystemFnc_(t);
	logicRulesPtr_->getMotionPhaseLogics(activeSubsystem_, stanceLegs_, zDirectionRefsPtr_);

	// if it is not the last subsystem (numSubsystems = numEventTimes_+1)
	if (activeSubsystem_ < numEventTimes_)
		logicRulesPtr_->getContactFlags(activeSubsystem_+1, nextPhaseStanceLegs_);
	else
		nextPhaseStanceLegs_.fill(true);

	// the indices of the event times on which a leg started and finished swinging.
	logicRulesPtr_->getFeetPlanner().getStartTimesIndices(startTimesIndices_);
	logicRulesPtr_->getFeetPlanner().getFinalTimesIndices(finalTimesIndices_);

	// joints
	qJoints_  = x.template tail<12>();
	// joints' velocities
	dqJoints_ = u.template tail<12>();

	// Rotation matrix from Base frame (or the coincided frame world frame) to Origin frame (global world).
	o_R_b_ = RotationMatrixBasetoOrigin(x.template head<3>());

	// base to CoM displacement in the CoM frame
	com_base2CoM_ = comModelPtr_->comPositionBaseFrame(qJoints_);

	// Inertia matrix in the CoM frame and its derivatives
	M_ = comModelPtr_->comInertia(qJoints_);
	Eigen::Matrix3d rotationMInverse = M_.topLeftCorner<3,3>().inverse();
	MInverse_ << rotationMInverse, Eigen::Matrix3d::Zero(),
			Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Identity()/M_(5,5);

	if (useInertiaMatrixDerivate())
		dMdt_ = comModelPtr_->comInertiaDerivative(qJoints_, dqJoints_);
	else
		dMdt_.setZero();

	// CoM Jacobian in the Base frame
	b_comJacobain_ = MInverse_ * comModelPtr_->comMomentumJacobian(qJoints_);
	// Time derivative of CoM Jacobian in the Base frame
	if (useInertiaMatrixDerivate())
		b_comJacobainTimeDerivative_ = MInverse_*comModelPtr_->comMomentumJacobianDerivative(qJoints_, dqJoints_) - MInverse_*dMdt_*b_comJacobain_;
	else
		b_comJacobainTimeDerivative_.setZero();

	// base coordinate
	basePose_.template head<3>() = x.template segment<3>(0);
	basePose_.template tail<3>() = x.template segment<3>(3) - o_R_b_ * com_base2CoM_;

	// local angular and linear velocity of Base
	baseLocalVelocities_ = x.template segment<6>(6) - b_comJacobain_ * dqJoints_;
	baseLocalVelocities_.template head<3>() = x.template segment<3>(6) - b_comJacobain_.template topRows<3>()*dqJoints_;
	baseLocalVelocities_.template tail<3>() = x.template segment<3>(9) - b_comJacobain_.template bottomRows<3>()*dqJoints_
			+ com_base2CoM_.cross(baseLocalVelocities_.template head<3>());

	// update kinematic model
	kinematicModelPtr_->update(basePose_, qJoints_);

	for (size_t i=0; i<NUM_CONTACT_POINTS_; i++)  {

		// base to stance feet displacement in the CoM frame
		kinematicModelPtr_->footPositionBaseFrame(i, com_base2StanceFeet_[i]);
		com_com2StanceFeet_[i] = com_base2StanceFeet_[i]-com_base2CoM_;

		// foot position in the Origin frame
		kinematicModelPtr_->footPositionOriginFrame(i, o_origin2StanceFeet_[i]);

		// feet Jacobain's in the Base frame
		kinematicModelPtr_->footJacobainBaseFrame(i, b_feetJacobains_[i]);

	}  // end of i loop


	// feet Jacobian time derivative in the Base frame
	const double h = sqrt(Eigen::NumTraits<double>::epsilon());
	joint_coordinate_t qJointsPlus = qJoints_ + dqJoints_*h;
	kinematicModelPtr_->update(basePose_, qJointsPlus);
	for (size_t i=0; i<NUM_CONTACT_POINTS_; i++) {
		base_jacobian_matrix_t b_footJacobainPlus;
		kinematicModelPtr_->footJacobainBaseFrame(i, b_footJacobainPlus);
		b_feetJacobainsTimeDerivative_[i].setZero();
		b_feetJacobainsTimeDerivative_[i].template block<6,3>(0,3*i) = (b_footJacobainPlus.template block<6,3>(0,3*i)-b_feetJacobains_[i].template block<6,3>(0,3*i))/h;
	}  // end of i loop

	// if GapIndicator is provided
	if (endEffectorStateConstraintsPtr_ && endEffectorStateConstraintsPtr_->empty()==false)
		for (size_t i=0; i<NUM_CONTACT_POINTS_; i++)  {
			// check for the gaps
			feetConstraintIsActive_[i] = false;

			if (stanceLegs_[i]==false && nextPhaseStanceLegs_[i]==true)  {

				feetConstraintValues_[i] = 0;
				feetConstraintJacobains_[i].setZero();
				for (const auto& eeConstraintPtr_ : *endEffectorStateConstraintsPtr_) {

					if (eeConstraintPtr_->isActive(o_origin2StanceFeet_[i])==false)
						continue;

					scalar_t constraintValue = eeConstraintPtr_->constraintValue(o_origin2StanceFeet_[i]);
					if (constraintValue>std::numeric_limits<scalar_t>::epsilon()) {
						feetConstraintIsActive_[i]  = true;
						feetConstraintValues_[i] 	= constraintValue;
						feetConstraintJacobains_[i] = eeConstraintPtr_->constraintDerivative(o_origin2StanceFeet_[i]);
						break;
					}  // end of if
				}  // end of j loop

			}  // end of if
		}  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getConstraint1(
		constraint1_vector_t& g1) {

	size_t nextFreeIndex = 0;
	for (size_t i=0; i<NUM_CONTACT_POINTS_; i++) {

		// the contact force at swing leg is zero
		if (stanceLegs_[i]==false) {
			g1.template segment<3>(nextFreeIndex) = options_.contactForceWeight_*Base::u_.template segment<3>(3*i);
			nextFreeIndex += 3;

		} else {

			// stance foot velocity in the World frame
			g1.template segment<3>(nextFreeIndex) = b_feetJacobains_[i].template bottomRows<3>()*dqJoints_ + baseLocalVelocities_.template tail<3>()
								+ baseLocalVelocities_.template head<3>().cross(com_base2StanceFeet_[i]);

			nextFreeIndex += 3;
		}

	}  // end of i loop

	// add the swing legs z direction constraints, if its CPG is provided
	for (size_t i=0; i<NUM_CONTACT_POINTS_; i++) {
		if (stanceLegs_[i]==false && zDirectionRefsPtr_[i]!=nullptr) {

			// stance foot velocity in the Origin frame
			Eigen::Vector3d o_footVelocity = o_R_b_ * ( b_feetJacobains_[i].template bottomRows<3>()*dqJoints_ + baseLocalVelocities_.template tail<3>()
					+ baseLocalVelocities_.template head<3>().cross(com_base2StanceFeet_[i]) );

			g1(nextFreeIndex) = options_.zDirectionVelocityWeight_ *
					(o_footVelocity(2) - zDirectionRefsPtr_[i]->calculateVelocity(Base::t_));
			nextFreeIndex++;
		}
	}  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
size_t ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::numStateInputConstraint(
		const scalar_t& time) {

	size_t numConstraint1 = 12;

	// add the swing legs z direction constraints, if its CPG is provided
	for (size_t i=0; i<NUM_CONTACT_POINTS_; i++) {
		if (stanceLegs_[i]==false && zDirectionRefsPtr_[i]!=nullptr) {
			numConstraint1++;
		}
	}  // end of i loop

	return numConstraint1;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getConstraint2(
		constraint2_vector_t& g2) {

	size_t numConstraint2 = 0;
	if (options_.zDirectionPositionWeight_<std::numeric_limits<double>::epsilon())  return;

	for (size_t i=0; i<NUM_CONTACT_POINTS_; i++)
		if (stanceLegs_[i]==false && zDirectionRefsPtr_[i]!=nullptr) {

			g2(numConstraint2) = options_.zDirectionPositionWeight_ *
					( o_origin2StanceFeet_[i](2)-zDirectionRefsPtr_[i]->calculatePosition(Base::t_) );
			numConstraint2++;
		}  // end of if loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
size_t ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::numStateOnlyConstraint(
		const scalar_t& time) {

	size_t numConstraint2 = 0;
	if (options_.zDirectionPositionWeight_<std::numeric_limits<double>::epsilon())
		return numConstraint2;

	for (size_t i=0; i<NUM_CONTACT_POINTS_; i++)
		if (stanceLegs_[i]==false && zDirectionRefsPtr_[i]!=nullptr) {
			numConstraint2++;
		}  // end of if loop

	return numConstraint2;
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getInequalityConstraint(scalar_array_t& h) {
	h.clear();
	const scalar_t &mu = options_.frictionCoefficient_;
	for (size_t i=0; i<NUM_CONTACT_POINTS_; i++){
		if (stanceLegs_[i]){
			const scalar_t Fx = Base::u_(3 * i + 0);
			const scalar_t Fy = Base::u_(3 * i + 1);
			const scalar_t Fz = Base::u_(3 * i + 2);
			h.push_back( Fz*sqrt(2.0*mu*mu)-sqrt(Fx*Fx+Fy*Fy+mu*mu*Fz*Fz) );
		}
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
size_t ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::numInequalityConstraint(
		const scalar_t &time) {

	size_t numInequalities = 0;
	for (size_t i=0; i<NUM_CONTACT_POINTS_; i++){
		if (stanceLegs_[i]){
			numInequalities++;
		}
	}
	return numInequalities;
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getFinalConstraint2(
		constraint2_vector_t& g2Final) {

	size_t numFinalConstraint2 = 0;

	for (size_t i=0; i<NUM_CONTACT_POINTS_; i++)
		if (feetConstraintIsActive_[i]==true)  {

			g2Final(numFinalConstraint2) = feetConstraintValues_[i];
			numFinalConstraint2++;
		}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
size_t ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::numStateOnlyFinalConstraint(
		const scalar_t& time) {

	size_t numFinalConstraint2 = 0;

	for (size_t i=0; i<NUM_CONTACT_POINTS_; i++)
		if (feetConstraintIsActive_[i]==true)  {
			numFinalConstraint2++;
		}

	return numFinalConstraint2;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getConstraint1DerivativesState(
		constraint1_state_matrix_t& C) {


	// C matrix
	/*			o_theta_b	o_x_com		com_w_com	com_v_com	qJoints
	 * 		|	C00			C01			C02			C03			C04		|
	 * 		|	C10			C11			C12			C13			C14		|
	 * 	C =	|	C20			C21			C22			C23			C24		|
	 * 		|	C30			C31			C32			C33			C34		|
	 * 		|	0			0			0			0			dJz/dt	|
	 */

	size_t nextFreeIndex = 0;
	for (size_t i=0; i<NUM_CONTACT_POINTS_; i++) {

		// for a swing leg
		if (stanceLegs_[i]==false) {
			C.template block<3,24>(nextFreeIndex,0).setZero();
			nextFreeIndex += 3;
			continue;
		}

		// Ci0
		C.template block<3,3>(nextFreeIndex,0).setZero();

		// Ci1
		C.template block<3,3>(nextFreeIndex,3).setZero();

		// Ci2
		C.template block<3,3>(nextFreeIndex,6) = -CrossProductMatrix(com_com2StanceFeet_[i]);

		// Ci3
		C.template block<3,3>(nextFreeIndex,9).setIdentity();

		// Ci4
		C.template block<3,12>(nextFreeIndex,12) = CrossProductMatrix(baseLocalVelocities_.template head<3>()) * (
						b_feetJacobains_[i].template bottomRows<3>()-b_comJacobain_.template bottomRows<3>())
						+ b_feetJacobainsTimeDerivative_[i].template bottomRows<3>()-b_comJacobainTimeDerivative_.template bottomRows<3>()
						+ CrossProductMatrix(com_com2StanceFeet_[i]) * b_comJacobainTimeDerivative_.template topRows<3>();

		nextFreeIndex += 3;
	}

	// for the swing legs z direction constraints, if its CPG is provided
	for (size_t i=0; i<NUM_CONTACT_POINTS_; i++)
		if (stanceLegs_[i]==false && zDirectionRefsPtr_[i]!=nullptr) {
			Eigen::Matrix<double,3,24> partial_x;

			// Ci0
			Eigen::Vector3d o_footVelocity = o_R_b_ * ( b_feetJacobains_[i].template bottomRows<3>()*dqJoints_ + baseLocalVelocities_.template tail<3>()
					+ baseLocalVelocities_.template head<3>().cross(com_base2StanceFeet_[i]) );
			partial_x.template block<3,3>(0,0) = -CrossProductMatrix(o_footVelocity);

			// Ci1
			partial_x.template block<3,3>(0,3).setZero();

			// Ci2
			partial_x.template block<3,3>(0,6) = -o_R_b_ * CrossProductMatrix(com_com2StanceFeet_[i]);

			// Ci3
			partial_x.template block<3,3>(0,9) = o_R_b_;

			// Ci4
			partial_x.template block<3,12>(0,12) = o_R_b_ * ( CrossProductMatrix(baseLocalVelocities_.template head<3>()) * (
					b_feetJacobains_[i].template bottomRows<3>()-b_comJacobain_.template bottomRows<3>())
					+ b_feetJacobainsTimeDerivative_[i].template bottomRows<3>()-b_comJacobainTimeDerivative_.template bottomRows<3>()
					+ CrossProductMatrix(com_com2StanceFeet_[i]) * b_comJacobainTimeDerivative_.template topRows<3>() );

			C.template block<1,24>(nextFreeIndex,0) = options_.zDirectionVelocityWeight_*partial_x.template bottomRows<1>();
			nextFreeIndex++;
		}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getConstraint1DerivativesControl(
		constraint1_input_matrix_t& D) {

	// D matrix
	/*			lambda		dqJoints
	 * 		|	D00			D01		|
	 * 		|	D10			D11		|
	 * D = 	|	D20			D21		|
	 * 		|	D30			D31		|
	 * 		|	0			Jz		|
	 */

	size_t nextFreeIndex = 0;
	for (size_t i=0; i<NUM_CONTACT_POINTS_; i++) {

		// for swing leg
		if (stanceLegs_[i]==false) {
			D.template block<3,24>(nextFreeIndex,0).setZero();
			D.template block<3,3>(nextFreeIndex,nextFreeIndex) = options_.contactForceWeight_*Eigen::Matrix3d::Identity();
			nextFreeIndex += 3;
			continue;
		}

		// Di0
		D.template block<3,12>(nextFreeIndex,0).setZero();

		// Di1
		D.template block<3,12>(nextFreeIndex,12) = b_feetJacobains_[i].template bottomRows<3>() - b_comJacobain_.template bottomRows<3>() +
				CrossProductMatrix(com_com2StanceFeet_[i])*b_comJacobain_.template topRows<3>();

		nextFreeIndex += 3;
	}  // end of i loop


	// for the swing legs z direction constraints, if its CPG is provided
	for (size_t i=0; i<NUM_CONTACT_POINTS_; i++)
		if (stanceLegs_[i]==false && zDirectionRefsPtr_[i]!=nullptr) {
			Eigen::Matrix<double,3,12> partial_dq = o_R_b_ * ( b_feetJacobains_[i].template bottomRows<3>() - b_comJacobain_.template bottomRows<3>() +
					CrossProductMatrix(com_com2StanceFeet_[i])*b_comJacobain_.template topRows<3>() );
			D.template block<1,12>(nextFreeIndex,0).setZero();
			D.template block<1,12>(nextFreeIndex,12) = options_.zDirectionVelocityWeight_*partial_dq.template bottomRows<1>();
			nextFreeIndex++;
		}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getConstraint1DerivativesEventTimes(
		constraint1_vector_array_t& g1DevArray) {

	// set all to zero
	g1DevArray.resize(numEventTimes_);
	for (constraint1_vector_t& g1Dev : g1DevArray)
		g1Dev.setZero();

	size_t nextFreeIndex = 0;
	for (size_t j=0; j<NUM_CONTACT_POINTS_; j++) {
		// the contact force at swing leg is zero
		if (stanceLegs_[j]==false) {
			nextFreeIndex += 3;
		} else {
			// stance foot velocity in the World frame
			nextFreeIndex += 3;
		}
	} // end of j loop

	// add the swing legs z direction constraints derivative, if its CPG is provided
	for (size_t j=0; j<NUM_CONTACT_POINTS_; j++) {
		if (stanceLegs_[j]==false && zDirectionRefsPtr_[j]!=nullptr) {
			const int& startTimesIndex = startTimesIndices_[j][activeSubsystem_];
			g1DevArray[startTimesIndex](nextFreeIndex) = -options_.zDirectionVelocityWeight_ *
					zDirectionRefsPtr_[j]->calculateStartTimeDerivative(Base::t_);
			const int& finalTimesIndex = finalTimesIndices_[j][activeSubsystem_];
			g1DevArray[finalTimesIndex](nextFreeIndex) = -options_.zDirectionVelocityWeight_ *
					zDirectionRefsPtr_[j]->calculateFinalTimeDerivative(Base::t_);
			nextFreeIndex++;
		}
	} // end of j loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getConstraint2DerivativesState(
		constraint2_state_matrix_t& F)  {

	size_t numConstraint2 = 0;
	if (options_.zDirectionPositionWeight_<std::numeric_limits<double>::epsilon())  return;

	for (size_t i=0; i<NUM_CONTACT_POINTS_; i++)
		if (stanceLegs_[i]==false && zDirectionRefsPtr_[i]!=nullptr) {
			// foot jacobian in the Origin frame
			Eigen::Matrix<double, 3, 18> o_footJacobian;
			o_footJacobian.block<3,3>(0,0)  = -CrossProductMatrix(o_R_b_*com_com2StanceFeet_[i]);
			o_footJacobian.block<3,3>(0,3)  = Eigen::Matrix3d::Identity();
			o_footJacobian.block<3,12>(0,6) = o_R_b_ * (b_feetJacobains_[i].template bottomRows<3>()-b_comJacobain_.template bottomRows<3>());

			F.template block<1,6>(numConstraint2,0)   = options_.zDirectionPositionWeight_ * o_footJacobian.block<1,6>(2,0);
			F.template block<1,6>(numConstraint2,6)   = Eigen::Matrix<double,1,6>::Zero();
			F.template block<1,12>(numConstraint2,12) = options_.zDirectionPositionWeight_ * o_footJacobian.block<1,12>(2,6);
			numConstraint2++;
		}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getInequalityConstraintDerivativesState(state_vector_array_t &dhdx) {

	dhdx.clear();
	for (size_t i=0; i<NUM_CONTACT_POINTS_; i++){
		if (stanceLegs_[i]){
			// Friction cone constraint is independent of state
			dhdx.emplace_back(state_vector_t::Zero());
		}
	}
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getInequalityConstraintDerivativesControl(
		switched_model::ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::input_vector_array_t &dhdu) {

	dhdu.clear();
	input_vector_t frictionConeDerivative;
	const scalar_t &mu = options_.frictionCoefficient_;
	for (size_t i=0; i<NUM_CONTACT_POINTS_; i++){
		if (stanceLegs_[i]){
			frictionConeDerivative.setZero();
			const scalar_t Fx = Base::u_(3 * i + 0);
			const scalar_t Fy = Base::u_(3 * i + 1);
			const scalar_t Fz = Base::u_(3 * i + 2);
			const scalar_t F_norm = sqrt(Fx*Fx+Fy*Fy+Fz*Fz*mu*mu);
			if (F_norm > 0){
				frictionConeDerivative(3 * i + 0) = -Fx/F_norm;
				frictionConeDerivative(3 * i + 1) = -Fy/F_norm;
				frictionConeDerivative(3 * i + 2) = -mu*mu*Fz/F_norm + sqrt(2.0*mu*mu);
			} else {  // Fi / F_norm = 0 / 0 ==> 0
				frictionConeDerivative(3 * i + 2) = sqrt(2.0*mu*mu);
			}
			dhdu.push_back( frictionConeDerivative );
		}
	}
}



/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getInequalityConstraintSecondDerivativesState(
		switched_model::ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::state_matrix_array_t &ddhdxdx) {

	ddhdxdx.clear();
	for (size_t i=0; i<NUM_CONTACT_POINTS_; i++){
		if (stanceLegs_[i]){
			//  Friction cone constraint independent of state
			ddhdxdx.emplace_back(state_matrix_t::Zero());
		}
	}
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getInequalityConstraintSecondDerivativesInput(
		switched_model::ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::input_matrix_array_t &ddhdudu) {

	ddhdudu.clear();
	input_matrix_t frictionConeHessian;
	const scalar_t &mu = options_.frictionCoefficient_;
	for (size_t i=0; i<NUM_CONTACT_POINTS_; i++){
		if (stanceLegs_[i]){
			//  d2/dudu ( sqrt(mu*mu + 1)*Fz >= sqrt(Fx*Fx + Fy*Fy + Fz*Fz) )
			frictionConeHessian.setZero();
			const scalar_t Fx = Base::u_(3 * i + 0);
			const scalar_t Fy = Base::u_(3 * i + 1);
			const scalar_t Fz = Base::u_(3 * i + 2);
			const scalar_t F_norm2 = Fx*Fx+Fy*Fy+mu*mu*Fz*Fz;
			if (F_norm2 > 0){
				const scalar_t F_norm32 = pow(F_norm2, 1.5);
				frictionConeHessian(3 * i + 0, 3 * i + 0) = -(Fy*Fy+mu*mu*Fz*Fz)/F_norm32;
				frictionConeHessian(3 * i + 0, 3 * i + 1) = Fx*Fy/F_norm32;
				frictionConeHessian(3 * i + 0, 3 * i + 2) = Fx*mu*mu*Fz/F_norm32;
				frictionConeHessian(3 * i + 1, 3 * i + 0) = Fx*Fy/F_norm32;
				frictionConeHessian(3 * i + 1, 3 * i + 1) = -(Fx*Fx+mu*mu*Fz*Fz)/F_norm32;
				frictionConeHessian(3 * i + 1, 3 * i + 2) = Fy*mu*mu*Fz/F_norm32;
				frictionConeHessian(3 * i + 2, 3 * i + 0) = Fx*mu*mu*Fz/F_norm32;
				frictionConeHessian(3 * i + 2, 3 * i + 1) = Fy*mu*mu*Fz/F_norm32;
				frictionConeHessian(3 * i + 2, 3 * i + 2) = -mu*mu*(Fx*Fx+Fy*Fy)/F_norm32;
			} // else all terms are 0 / 0 ==> 0

			ddhdudu.push_back( frictionConeHessian );
		}
	}
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getInequalityConstraintDerivativesInputState(
		switched_model::ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::input_state_matrix_array_t &ddhdudx) {

	ddhdudx.clear();
	for (size_t i=0; i<NUM_CONTACT_POINTS_; i++){
		if (stanceLegs_[i]){
			//  Friction cone constraint independent of state
			ddhdudx.emplace_back(input_state_matrix_t::Zero());
		}
	}
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getFinalConstraint2DerivativesState(
		constraint2_state_matrix_t& F)  {

	size_t numConstraint2 = 0;

	for (size_t i=0; i<NUM_CONTACT_POINTS_; i++)
		if (feetConstraintIsActive_[i]==true)  {
			// foot jacobian in the Origin frame
			Eigen::Matrix<double, 3, 18> o_footJacobian;
			o_footJacobian.block<3,3>(0,0)  = -CrossProductMatrix(o_R_b_*com_com2StanceFeet_[i]);
			o_footJacobian.block<3,3>(0,3)  = Eigen::Matrix3d::Identity();
			o_footJacobian.block<3,12>(0,6) = o_R_b_ * (b_feetJacobains_[i].template bottomRows<3>()-b_comJacobain_.template bottomRows<3>());

			// gap jacobian
			Eigen::Matrix<double, 1, 18> o_gapJacobian = feetConstraintJacobains_[i].transpose() * o_footJacobian;

			F.template block<1,6>(numConstraint2,0)   = o_gapJacobian.block<1,6>(0,0);
			F.template block<1,6>(numConstraint2,6)   = Eigen::Matrix<double,1,6>::Zero();
			F.template block<1,12>(numConstraint2,12) = o_gapJacobian.block<1,12>(0,6);

			numConstraint2++;
		}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::setStanceLegs(
		const contact_flag_t& stanceLegs)  {

	stanceLegs_ = stanceLegs;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getStanceLegs(
		contact_flag_t& stanceLegs)  {

	stanceLegs = stanceLegs_;
}


} //end of namespace switched_model
