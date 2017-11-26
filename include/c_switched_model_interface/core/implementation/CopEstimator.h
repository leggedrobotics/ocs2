/*
 * CopEstimator.h
 *
 *  Created on: Nov 24, 2017
 *      Author: farbod
 */


namespace switched_model {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void CopEstimator<JOINT_COORD_SIZE>::copErrorEstimator(const std::array<bool,4>& stanceLegs,
		const joint_coordinate_t& qJoints, const joint_coordinate_t& lambda,
		Eigen::Vector2d& copError, // copError = xyMomentum_total - cop_des * lambda_total
		Eigen::Matrix<double,2,JOINT_COORD_SIZE>& devJoints_copError,
		Eigen::Matrix<double,2,JOINT_COORD_SIZE>& devLambda_copError) {

	double numStanceLegs = 0.0;
	b_totalPressureMoment_.setZero();
	devJoints_b_totalPressureMoment_.setZero();
	totalPressure_ = 0.0;
	b_base2CopDesired_.setZero();
	devJoints_b_base2CopDesired_.setZero();

	kinematicModelPtr_->update(base_coordinate_t::Zero(), qJoints); // for foot position + jacobian only care about qJoints

	for (size_t j=0; j<4; j++)
		if (stanceLegs[j]==true) {

			Eigen::Vector3d b_base2StanceFoot;
			kinematicModelPtr_->footPositionBaseFrame(j, b_base2StanceFoot);
			base_jacobian_matrix_t b_jacobianFoot;
			kinematicModelPtr_->footJacobainBaseFrame(j, b_jacobianFoot);

			// stance feet [x,y]'
			b_feetPosition_[j] = b_base2StanceFoot.template head<2>();

			// += stanceLeg
			numStanceLegs += 1;
			// =+ [x,y]' lambda_z
			b_totalPressureMoment_ += b_feetPosition_[j]*lambda(3*j+2);
			// =+ d/dq [x,y]' lambda_z
			devJoints_b_totalPressureMoment_ += b_jacobianFoot.template block<2,JOINT_COORD_SIZE>(3,0)*lambda(3*j+2);

			// += lambda_z
			totalPressure_ += lambda(3*j+2);
			// =+ [x,y]'
			b_base2CopDesired_ += b_feetPosition_[j];
			// =+ d/dq [x,y]'
			devJoints_b_base2CopDesired_ += b_jacobianFoot.template block<2,JOINT_COORD_SIZE>(3,0);
		}

	if (numStanceLegs > 0) {
		b_base2CopDesired_ /= numStanceLegs;
		devJoints_b_base2CopDesired_ /= numStanceLegs;
	}

	// copError = xyMomentum_total - cop_des * lambda_total
	copError = b_totalPressureMoment_ - b_base2CopDesired_*totalPressure_;

	// cop derivative w.r.t. joint
	devJoints_copError = devJoints_b_totalPressureMoment_ - devJoints_b_base2CopDesired_ * totalPressure_;

	// cop derivative w.r.t. contact forces
	devLambda_copError.setZero();
	for (size_t j=0; j<4; j++)
		if (stanceLegs[j]==true)
			devLambda_copError.col(3*j+2) = b_feetPosition_[j] - b_base2CopDesired_;

	// normalization to total weight
	copError /= totalWeight_;
	devJoints_copError /= totalWeight_;
	devLambda_copError /= totalWeight_;

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
Eigen::Vector2d CopEstimator<JOINT_COORD_SIZE>::copErrorEstimator(const std::array<bool,4>& stanceLegs,
		const joint_coordinate_t& qJoints, const joint_coordinate_t& lambda) {

	Eigen::Vector2d copError;
	Eigen::Matrix<double,2,12> devJoints_copError;
	Eigen::Matrix<double,2,12> devLambda_copError;

	copErrorEstimator(stanceLegs, qJoints, lambda, copError, devJoints_copError, devLambda_copError);

	return copError;
}

} // end of namespace switched_model

