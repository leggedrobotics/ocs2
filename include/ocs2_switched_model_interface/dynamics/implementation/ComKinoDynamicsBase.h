/*
 * ComKinoDynamicsBase.h
 *
 *  Created on: Nov 12, 2017
 *      Author: farbod
 */
namespace switched_model {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
ComKinoDynamicsBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>*
	ComKinoDynamicsBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::clone() const {

	return new ComKinoDynamicsBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ComKinoDynamicsBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::initializeModel(
		logic_rules_machine_t& logicRulesMachine,
		const size_t& partitionIndex,
		const char* algorithmName/*=NULL*/) {

	Base::initializeModel(logicRulesMachine, partitionIndex, algorithmName);

	findActiveSubsystemFnc_ = std::move( logicRulesMachine.getHandleToFindActiveEventCounter(partitionIndex) );

	logicRulesPtr_ = logicRulesMachine.getLogicRulesPtr();

	if (algorithmName!=NULL)
		algorithmName_.assign(algorithmName);
	else
		algorithmName_.clear();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ComKinoDynamicsBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::computeFlowMap(
		const scalar_t& t,
		const state_vector_t& x,
		const input_vector_t& u,
		state_vector_t& dxdt)   {

//	size_t index = findActiveSubsystemFnc_(t);
//	logicRulesPtr_->getContactFlags(index, stanceLegs_);

	// set data for CoM class
	comDynamics_.setData(stanceLegs_, x.template tail<12>(), u.template tail<12>());

	// CoM state time derivatives
	typename ComDynamicsBase<JOINT_COORD_SIZE>::state_vector_t stateDerivativeCoM;
	comDynamics_.computeFlowMap(t, x.template head<12>(), u.template head<12>(), stateDerivativeCoM);

	// extended state time derivatives
	dxdt << stateDerivativeCoM, u.template tail<12>();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
//template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
//void ComKinoDynamicsBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::computeGuardSurfaces(
//		const scalar_t& t,
//		const state_vector_t& x,
//		std::vector<scalar_t>& guardSurfacesValue) {
//
//	// Rotation matrix from Base frame (or the coincided frame world frame) to Origin frame (global world).
//	Eigen::Matrix3d o_R_b = RotationMatrixBasetoOrigin(x.template head<3>());
//
//	base_coordinate_t  comPose = x.template head<6>();
//	joint_coordinate_t qJoints = x.template tail<12>();
//
//	base_coordinate_t  basePose;
//	comModelPtr_->calculateBasePose(qJoints, comPose, basePose);
//
//	// update kinematic model
//	kinematicModelPtr_->update(basePose, qJoints);
//
//	// base to stance feet displacement in the CoM frame
//	std::array<Eigen::Matrix<scalar_t,3,1>,NUM_CONTACT_POINTS_> o_stanceFeetPosition;
//	kinematicModelPtr_->feetPositionsOriginFrame(o_stanceFeetPosition);
//
//	Eigen::Vector4d distanceToGround;
//	for (size_t i=0; i<NUM_CONTACT_POINTS_; i++) {
//		scalar_t groundHight = groundProfilePtr_->getGroundHight(o_stanceFeetPosition[i]);
//		distanceToGround(i) = o_stanceFeetPosition[i](2) - groundHight;
//	}
//
//	const scalar_t eps = options_.eps_;
//	const scalar_t eta = options_.eta_;
//	Eigen::Vector4d expDistanceToGround = distanceToGround.unaryExpr(
//			[&](const scalar_t& x) { return std::exp(eta/eps*(x-eps)); });
//	Eigen::Vector4d expMinusDistanceToGround = expDistanceToGround.cwiseInverse();
//
//	const size_t numModes = std::pow(2,NUM_CONTACT_POINTS_);
//	guardSurfacesValue.clear();
//	guardSurfacesValue.reserve(numModes);
//
//	for (size_t i=0; i<numModes; i++) {
//
//		std::array<bool,NUM_CONTACT_POINTS_> contactFlag = modeNumber2StanceLeg(i);
//
//		scalar_t guardSurfaceValue = 1.0;
//		for (size_t j=0; j<NUM_CONTACT_POINTS_; j++) {
//			if (contactFlag[j]==true)
//				guardSurfaceValue -= 1.0/(scalar_t)NUM_CONTACT_POINTS_ * expMinusDistanceToGround(j);
//			else
//				guardSurfaceValue -= 1.0/(scalar_t)NUM_CONTACT_POINTS_ * expDistanceToGround(j);
//
//			scalar_t temp;
//			if (contactFlag[j]==true)
//				temp = 1.0 - expMinusDistanceToGround(j);
//			else
//				temp = 1.0 - expDistanceToGround(j);
//
//			std::cout << "Leg[" << j << "]:\t Contact: " <<  contactFlag[j] << "\t Value: " << temp << std::endl;
//		}
//
//		guardSurfacesValue.push_back(guardSurfaceValue);
//
//	}  // end of i loop
//
//	std::cout << "time: " << t << std::endl;
//	std::cout << "Sate: " << x.transpose() << std::endl;
//	std::cout << "guardSurfacesValue: {";
//	for (auto d : guardSurfacesValue)
//		std::cout << d << ", ";
//	std::cout << "\b\b}\n";
//}

template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ComKinoDynamicsBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::computeGuardSurfaces(
		const scalar_t& t,
		const state_vector_t& x,
		dynamic_vector_t& guardSurfacesValue) {

	// Rotation matrix from Base frame (or the coincided frame world frame) to Origin frame (global world).
	Eigen::Matrix3d o_R_b = RotationMatrixBasetoOrigin(x.template head<3>());

	base_coordinate_t  comPose = x.template head<6>();
	joint_coordinate_t qJoints = x.template tail<12>();

	base_coordinate_t  basePose;
	comModelPtr_->calculateBasePose(qJoints, comPose, basePose);

	// update kinematic model
	kinematicModelPtr_->update(basePose, qJoints);

	// base to stance feet displacement in the CoM frame
	std::array<Eigen::Matrix<scalar_t,3,1>,NUM_CONTACT_POINTS_> o_stanceFeetPosition;
	kinematicModelPtr_->feetPositionsOriginFrame(o_stanceFeetPosition);

	Eigen::Vector4d distanceToGround;
	for (size_t i=0; i<NUM_CONTACT_POINTS_; i++) {
		scalar_t groundHight = groundProfilePtr_->getGroundHight(o_stanceFeetPosition[i]);
		distanceToGround(i) = o_stanceFeetPosition[i](2) - groundHight;
	}

	auto clampingFunc		= [&](const scalar_t& x) { return std::max( std::min(x, 5.0), -5.0); };
	auto contactEstabishing = [&](const scalar_t& x) { return clampingFunc(x/options_.eps_ - 1.0); };
	auto contactBreaking    = [&](const scalar_t& x) { return clampingFunc(1.0 - x/options_.eps_); };

	const size_t numModes = std::pow(2, (int)NUM_CONTACT_POINTS_);
	guardSurfacesValue.resize(numModes);

	for (size_t i=0; i<numModes; i++) {

		std::array<bool,NUM_CONTACT_POINTS_> contactFlag = modeNumber2StanceLeg(i);

		Eigen::Vector4d eachLegGuardSurfaces;
		for (size_t j=0; j<NUM_CONTACT_POINTS_; j++) {
			if (contactFlag[j]==true)
				eachLegGuardSurfaces(j) = contactEstabishing(distanceToGround(j));
			else
				eachLegGuardSurfaces(j) = contactBreaking(distanceToGround(j));
		}

		// Soft-Max
		Eigen::Vector4d expEachLegGuardSurfaces = eachLegGuardSurfaces.unaryExpr(
					[&](const scalar_t& x) { return std::exp(options_.eta_*x); });
		Eigen::Vector4d softMax = eachLegGuardSurfaces.cwiseProduct(expEachLegGuardSurfaces);
		guardSurfacesValue(i) = softMax.sum() / expEachLegGuardSurfaces.sum();
		// Max
//		guardSurfacesValue.push_back(eachLegGuardSurfaces.maxCoeff());

	}  // end of i loop

//	std::cout << "time: " << t << std::endl;
//	std::cout << "distanceToGround: " << distanceToGround.transpose() << std::endl;
//	for (size_t j=0; j<NUM_CONTACT_POINTS_; j++) {
//		std::cout << "Leg[" << j << "]: \t contactEstabishing: " << contactEstabishing(distanceToGround(j))
//				<< "\t contactBreaking: " << contactBreaking(distanceToGround(j)) << std::endl;
//	}
//	std::cout << "guardSurfacesValue: ";
//	for (size_t i=0; i<16; i++)
//		if (guardSurfacesValue[i]<0.6)
//			std::cout << ",   [" << i << "]: " << guardSurfacesValue[i];
//	std::cout << "\n";
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ComKinoDynamicsBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::setStanceLegs (const contact_flag_t& stanceLegs)  {
	stanceLegs_ = stanceLegs;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ComKinoDynamicsBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getStanceLegs (contact_flag_t& stanceLegs)  const {
	stanceLegs = stanceLegs_;
}


} // end of namespace switched_model


