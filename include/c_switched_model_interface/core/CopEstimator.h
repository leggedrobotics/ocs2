/*
 * CopEstimator.h
 *
 *  Created on: Nov 24, 2017
 *      Author: farbod
 */

#ifndef COPESTIMATOR_H_
#define COPESTIMATOR_H_

#include <array>
#include <memory>
#include <iostream>
#include <Eigen/Dense>

#include "c_switched_model_interface/core/SwitchedModel.h"
#include "c_switched_model_interface/core/KinematicsModelBase.h"
#include "c_switched_model_interface/core/ComModelBase.h"


namespace switched_model {

template< size_t JOINT_COORD_SIZE >
class CopEstimator
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::shared_ptr<CopEstimator<JOINT_COORD_SIZE>> Ptr;

	typedef ComModelBase<JOINT_COORD_SIZE> com_model_t;
	typedef KinematicsModelBase<JOINT_COORD_SIZE> kinematic_model_t;

	typedef typename SwitchedModel<JOINT_COORD_SIZE>::base_coordinate_t  base_coordinate_t;
	typedef typename SwitchedModel<JOINT_COORD_SIZE>::joint_coordinate_t joint_coordinate_t;
	typedef Eigen::Matrix<double,6,JOINT_COORD_SIZE> base_jacobian_matrix_t;

	CopEstimator(const typename kinematic_model_t::Ptr& kinematicModelPtr, const typename com_model_t::Ptr& comModelPtr)
	: kinematicModelPtr_(kinematicModelPtr->clone()),
	  comModelPtr_(comModelPtr->clone()),
	  totalWeight_(comModelPtr->totalMass()*9.81)
	{}

	/**
	 * copy constructor
	 */
	CopEstimator(const CopEstimator& rhs)
	: kinematicModelPtr_(rhs.kinematicModelPtr_->clone()),
	  comModelPtr_(rhs.comModelPtr_->clone()),
	  totalWeight_(rhs.totalWeight_)
	{}

	/**
	 * Estimates the CoP displacement (copError) from the center of the support polygon. It also calcultes copError deribatives
	 * with respect to joint angles and contact forces. All the output values are normalized by the total weight of robot.
	 *
	 * @param stanceLegs [in] Stance leg configuration
	 * @param qJoints [in] Joint angles
	 * @param lambda [in] contact forces
	 * @param copError [out] CoP displacement from the center of the support polygon
	 * @param devJoints_copError [out] derivative of CoP w.r.t. joint angles
	 * @param devLambda_copError [out] derivative of CoP w.r.t. contact forces
	 */
	void copErrorEstimator(const std::array<bool,4>& stanceLegs, const joint_coordinate_t& qJoints, const joint_coordinate_t& lambda,
			Eigen::Vector2d& copError, Eigen::Matrix<double,2,12>& devJoints_copError, Eigen::Matrix<double,2,12>& devLambda_copError);

	/**
	 * Estimates the CoP displacement (copError) from the center of the support polygon. the output value is
	 * normalized by the total weight of robot.
	 *
	 * @param stanceLegs [in] Stance leg configuration
	 * @param qJoints [in] Joint angles
	 * @param lambda [in] contact forces
	 * @return CoP displacement from the center of the support polygon
	 */
	Eigen::Vector2d copErrorEstimator(const std::array<bool,4>& stanceLegs, const joint_coordinate_t& qJoints, const joint_coordinate_t& lambda);


private:
	typename kinematic_model_t::Ptr kinematicModelPtr_;
	typename com_model_t::Ptr comModelPtr_;

	double totalWeight_;

	std::array<Eigen::Vector2d,4> b_feetPosition_;

	double totalPressure_;
	Eigen::Vector2d b_totalPressureMoment_;
	Eigen::Vector2d b_base2CopDesired_;
	Eigen::Matrix<double,2,JOINT_COORD_SIZE> devJoints_b_totalPressureMoment_;
	Eigen::Matrix<double,2,JOINT_COORD_SIZE> devJoints_b_base2CopDesired_;
};

}  // end of switched_model namespace

#include "implementation/CopEstimator.h"

#endif /* COPESTIMATOR_H_ */

