/*
 * ComKinoDynamicsBase.h
 *
 *  Created on: Nov 12, 2017
 *      Author: farbod
 */


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
std::shared_ptr<ComKinoDynamicsBase<JOINT_COORD_SIZE>::Base> ComKinoDynamicsBase<JOINT_COORD_SIZE>::clone() {

	return std::allocate_shared< ComKinoDynamicsBase<JOINT_COORD_SIZE>, Eigen::aligned_allocator<ComKinoDynamicsBase<JOINT_COORD_SIZE>> > (
			Eigen::aligned_allocator<ComKinoDynamicsBase<JOINT_COORD_SIZE>>(), *this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void ComKinoDynamicsBase<JOINT_COORD_SIZE>::initializeModel(const std::vector<size_t>& systemStockIndexes, const std::vector<scalar_t>& switchingTimes,
		const state_vector_t& initState, const size_t& activeSubsystemIndex/*=0*/, const char* algorithmName/*=NULL*/)  {

	Base::initializeModel(systemStockIndexes, switchingTimes, initState, activeSubsystemIndex, algorithmName);
	comDynamics_.initializeModel(systemStockIndexes, switchingTimes, initState.head<12>(), activeSubsystemIndex, algorithmName);

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
void ComKinoDynamicsBase<JOINT_COORD_SIZE>::SwitchedHyQDynamics::computeDerivative(const scalar_t& t,
		const state_vector_t& x,
		const control_vector_t& u,
		state_vector_t& dxdt)   {

	// set data for CoM class
	comDynamics_.setData(stanceLegs_, x.template tail<12>(), u.template tail<12>());

	// CoM state time derivatives
	typename ComDynamicsBase<JOINT_COORD_SIZE>::state_vector_t stateDerivativeCoM;
	comDynamics_.computeDerivative(t, x.head<12>(), u.head<12>(), stateDerivativeCoM);

	// extended state time derivatives
	dxdt << stateDerivativeCoM, u.template tail<12>();
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void ComKinoDynamicsBase<JOINT_COORD_SIZE>::computeConstriant1(const scalar_t& t,
			const state_vector_t& x,
			const control_vector_t& u,
			size_t& numConstraint1,
			constraint1_vector_t& g1) {

	// Joints
	Eigen::VectorBlock<const state_vector_t,12>   qJoints  = x.tail<12>();
	// Joints' velocities
	Eigen::VectorBlock<const control_vector_t,12> dqJoints = u.tail<12>();
	// CoM local velocities
	Eigen::VectorBlock<const state_vector_t,6> comLocalVelocities = x.segment<6>(6);
	// Base pse
	base_coordinate_t basePose;
	// Base local velocities
	base_coordinate_t baseLocalVelocities;

	// Rotation matrix from Base frame (or the coincided frame world frame) to Origin frame (global world).
	Eigen::Matrix3d o_R_b = SwitchedModel::RotationMatrixBasetoOrigin(comPose.head<3>());

	// base to CoM displacement in the CoM frame
	Eigen::Vector3d com_base2CoM = comModel_.comPositionBaseFrame(qJoints);

	// base coordinate
	basePose.head<3>() = comPose.segment<3>(0);
	basePose.tail<3>() = comPose.segment<3>(3) - o_R_b * com_base2CoM;

	// CoM Jacobin in the Base frame
	Eigen::Matrix<double,6,12> b_comJacobain = comModel_.comJacobainBaseFrame(qJoints);

	// local velocities of Base (b_W_b)
	baseLocalVelocities.head<3>() = comLocalVelocities.head<3>() - b_comJacobain.topRows<3>()*dqJoints;
	baseLocalVelocities.tail<3>() = comLocalVelocities.tail<3>() - b_comJacobain.bottomRows<3>()*dqJoints
			+ com_base2CoM.cross(baseLocalVelocities.head<3>());


	kinematicModel_.update(basePose, qJoints);

	size_t nextFreeIndex = 0;
	for (size_t i=0; i<4; i++) {

		// the contact force at swing leg is zero
		if (stanceLegs_[i]==false) {
			g1.segment<3>(nextFreeIndex) = options_.contactForceWeight_*u.segment<3>(3*i);
			nextFreeIndex += 3;

		} else {
			// stance foot position in the Base frame
			Eigen::Vector3d b_base2Foot;
			kinematicModel_.footPositionBaseFrame(i, b_base2Foot);

			// stance foot Jacobian in the base frame
			Eigen::Matrix<double,6,JOINT_COORD_SIZE> b_footJacobain;
			kinematicModel_.footJacobainBaseFrame(i, b_footJacobain);

			// stance foot velocity in the World frame
			g1.segment<3>(nextFreeIndex) = b_footJacobain.template bottomRows<3>()*dqJoints + baseLocalVelocities.template tail<3>()
						+ baseLocalVelocities.head<3>().cross(b_base2Foot);

			nextFreeIndex += 3;
		}

	}  // end of i loop

	// add the swing legs z direction constraints, if its CPG is provided
	for (size_t i=0; i<4; i++) {
		if (stanceLegs_[i]==false && feetZDirectionCPGs_[i]!=NULL) {

			// stance foot position in the Base frame
			Eigen::Vector3d b_base2Foot;
			kinematicModel_.footPositionBaseFrame(i, b_base2Foot);

			// swing foot Jacobian in the base frame
			Eigen::Matrix<double,6,JOINT_COORD_SIZE> b_footJacobain;
			kinematicModel_.footJacobainBaseFrame(i, b_footJacobain);

			// stance foot velocity in the Origin frame
			Eigen::Vector3d o_footVelocity = o_R_b * ( b_footJacobain.template bottomRows<3>()*dqJoints + baseLocalVelocities.template tail<3>()
					+ baseLocalVelocities.head<3>().cross(b_base2Foot) );

			g1(nextFreeIndex) = options_.zDirectionVelocityWeight_*(o_footVelocity(2) - feetZDirectionCPGs_[i]->calculateVelocity(t));

			nextFreeIndex++;
		}
	}  // end of i loop

	numConstraint1 = nextFreeIndex;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void ComKinoDynamicsBase<JOINT_COORD_SIZE>::computeConstriant2(const scalar_t& t,
		const state_vector_t& x,
		size_t& numConstraint2,
		constraint2_vector_t& g2) {

	numConstraint2 = 0;
	if (options_.zDirectionPositionWeight_<std::numeric_limits<double>::epsilon())  return;

	// Joints
	Eigen::VectorBlock<const state_vector_t,12> qJoints = x.tail<12>();

	// Base pose
	base_coordinate_t basePose;
	ComDynamicsBase<JOINT_COORD_SIZE>::CalculateBasePose(qJoints, x.head<6>(), basePose);

	kinematicModel_.update(basePose, qJoints);

	for (size_t i=0; i<4; i++)
		if (stanceLegs_[i]==false && feetZDirectionCPGs_[i]!=NULL) {

			// foot position in the Origin frame
			Eigen::Vector3d o_origin2Foot;
			kinematicModel_.footPositionOriginFrame(i, o_origin2Foot);

			g2(numConstraint2) = options_.zDirectionPositionWeight_ * ( o_origin2Foot(2)-feetZDirectionCPGs_[i]->calculatePosition(t) );
			numConstraint2++;
		}  // end of if loop
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void ComKinoDynamicsBase<JOINT_COORD_SIZE>::computeFinalConstriant2(const scalar_t& t,
		const state_vector_t& x,
		size_t& numFinalConstraint2,
		constraint2_vector_t& g2Final) {

	numFinalConstraint2 = 0;
	// if a gap is not determined
	if (endEffectorStateConstraints_.empty()==true)   return;

	// Joints
	Eigen::VectorBlock<const state_vector_t,12> qJoints = x.tail<12>();

	// Base pose
	base_coordinate_t basePose;
	ComDynamicsBase<JOINT_COORD_SIZE>::CalculateBasePose(qJoints, x.head<6>(), basePose);

	kinematicModel_.update(basePose, qJoints);


	for (size_t i=0; i<4; i++)
		if (stanceLegs_[i]==false && nextPhaseStanceLegs_[i]==true)  {

			// foot position in the Origin frame
			Eigen::Vector3d o_origin2Foot;
			kinematicModel_.footPositionOriginFrame(i, o_origin2Foot);

			for (size_t j=0; j<endEffectorStateConstraints_.size(); j++)  {

				if (endEffectorStateConstraints_[j]->isActive(o_origin2Foot)==false)  continue;

				double constraintValue = endEffectorStateConstraints_[j]->constraintValue(o_origin2Foot);
				if (constraintValue>std::numeric_limits<double>::epsilon()) {
					g2Final(numFinalConstraint2) = constraintValue;
					numFinalConstraint2++;
					break;
				}  // end of if loop
			}  // end of j loop

		}  // end of if loop
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void ComKinoDynamicsBase<JOINT_COORD_SIZE>::setStanceLegs (const std::array<bool,4>& stanceLegs)  {
	stanceLegs_ = stanceLegs;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void ComKinoDynamicsBase<JOINT_COORD_SIZE>::getStanceLegs (std::array<bool,4>& stanceLegs)  const {
	stanceLegs = stanceLegs_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void ComKinoDynamicsBase<JOINT_COORD_SIZE>::setSwingLegsCpgsPlanner (
		const FeetZDirectionPlannerBase::Ptr& feetZDirectionPlanner) {
	feetZDirectionPlanner_ = feetZDirectionPlanner;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void ComKinoDynamicsBase<JOINT_COORD_SIZE>::setSwingLegsCpgs (const std::array<CpgBase::Ptr,4>& feetZDirectionCPGs) {
	feetZDirectionCPGs_ = feetZDirectionCPGs;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void ComKinoDynamicsBase<JOINT_COORD_SIZE>::setEndEffectorStateConstraints (
		const std::vector<EndEffectorConstraintBase::Ptr>& endEffectorStateConstraints) {
	endEffectorStateConstraints_ = endEffectorStateConstraints;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
std::vector<ComKinoDynamicsBase<JOINT_COORD_SIZE>::EndEffectorConstraintBase::Ptr>& ComKinoDynamicsBase<JOINT_COORD_SIZE>::getEndEffectorStateConstraints () {
	return endEffectorStateConstraints_;
}

