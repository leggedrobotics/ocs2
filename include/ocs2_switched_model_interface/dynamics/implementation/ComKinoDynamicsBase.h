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
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
ComKinoDynamicsBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>*
	ComKinoDynamicsBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::clone() const {

	return new ComKinoDynamicsBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void ComKinoDynamicsBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::computeFlowMap(
		const scalar_t& t,
		const state_vector_t& x,
		const input_vector_t& u,
		state_vector_t& dxdt)   {

	// set data for CoM class
	comDynamics_.setData(x.template tail<12>(), u.template tail<12>());

	// CoM state time derivatives
	typename ComDynamicsBase<JOINT_COORD_SIZE>::state_vector_t stateDerivativeCoM;
	comDynamics_.computeFlowMap(t, x.template head<12>(), u.template head<12>(), stateDerivativeCoM);

	// extended state time derivatives
	dxdt << stateDerivativeCoM, u.template tail<12>();
}

template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void ComKinoDynamicsBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::computeGuardSurfaces(
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

	}  // end of i loop

}


} // end of namespace switched_model


