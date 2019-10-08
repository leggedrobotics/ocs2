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

#include "ocs2_switched_model_interface/core/ComModelBase.h"
#include "ocs2_switched_model_interface/core/KinematicsModelBase.h"
#include "ocs2_switched_model_interface/core/SwitchedModel.h"


namespace switched_model {
namespace tpl {

template <size_t JOINT_COORD_SIZE, typename SCALAR_T>
class CopEstimator
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::shared_ptr<CopEstimator<JOINT_COORD_SIZE, SCALAR_T>> Ptr;

	typedef ComModelBase<JOINT_COORD_SIZE, SCALAR_T> com_model_t;
	typedef KinematicsModelBase<JOINT_COORD_SIZE, SCALAR_T> kinematic_model_t;

	typedef Eigen::Matrix<SCALAR_T, 6, JOINT_COORD_SIZE> base_jacobian_matrix_t;

	typedef SwitchedModel<JOINT_COORD_SIZE, SCALAR_T> switched_model_t;
	typedef typename switched_model_t::contact_flag_t     contact_flag_t;
	typedef typename switched_model_t::base_coordinate_t  base_coordinate_t;
	typedef typename switched_model_t::joint_coordinate_t joint_coordinate_t;

	CopEstimator(
			const kinematic_model_t& kinematicModel,
			const com_model_t& comModel,
			const SCALAR_T& gravitationalAcceleration=9.81)
	: kinematicModelPtr_(kinematicModel.clone())
	, comModelPtr_(comModel.clone())
	, totalWeight_(comModel.totalMass()*gravitationalAcceleration)
	{
		if (gravitationalAcceleration<0)
			throw std::runtime_error("Gravitational acceleration should be a positive value.");
	}

	/**
	 * copy constructor
	 */
	CopEstimator(const CopEstimator& rhs)
	: kinematicModelPtr_(rhs.kinematicModelPtr_->clone())
	, comModelPtr_(rhs.comModelPtr_->clone())
	, totalWeight_(rhs.totalWeight_)
	{}

	/**
	 * Estimates the CoP displacement (copError) from the center of the support polygon. It also calculates copError derivatives
	 * with respect to joint angles and contact forces. All the output values are normalized by the total weight of robot.
	 *
	 * @param stanceLegs [in] Stance leg configuration
	 * @param qJoints [in] Joint angles
	 * @param lambda [in] contact forces
	 * @param copError [out] CoP displacement from the center of the support polygon
	 * @param devJoints_copError [out] derivative of CoP w.r.t. joint angles
	 * @param devLambda_copError [out] derivative of CoP w.r.t. contact forces
	 */
	void copErrorEstimator(
			const contact_flag_t& stanceLegs,
			const joint_coordinate_t& qJoints,
			const joint_coordinate_t& lambda,
			Eigen::Matrix<SCALAR_T,2,1>& copError,
			Eigen::Matrix<SCALAR_T,2,JOINT_COORD_SIZE>& devJoints_copError,
			Eigen::Matrix<SCALAR_T,2,JOINT_COORD_SIZE>& devLambda_copError);

	/**
	 * Estimates the CoP displacement (copError) from the center of the support polygon. the output value is
	 * normalized by the total weight of robot.
	 *
	 * @param stanceLegs [in] Stance leg configuration
	 * @param qJoints [in] Joint angles
	 * @param lambda [in] contact forces
	 * @return CoP displacement from the center of the support polygon
	 */
	Eigen::Matrix<SCALAR_T,2,1> copErrorEstimator(
			const contact_flag_t& stanceLegs,
			const joint_coordinate_t& qJoints,
			const joint_coordinate_t& lambda);


private:
	typename kinematic_model_t::Ptr kinematicModelPtr_;
	typename com_model_t::Ptr comModelPtr_;

	SCALAR_T totalWeight_;

	std::array<Eigen::Matrix<SCALAR_T,2,1>,switched_model_t::NUM_CONTACT_POINTS> b_feetPosition_;

	SCALAR_T totalPressure_;
	Eigen::Matrix<SCALAR_T,2,1> b_totalPressureMoment_;
	Eigen::Matrix<SCALAR_T,2,1> b_base2CopDesired_;
	Eigen::Matrix<SCALAR_T,2,JOINT_COORD_SIZE> devJoints_b_totalPressureMoment_;
	Eigen::Matrix<SCALAR_T,2,JOINT_COORD_SIZE> devJoints_b_base2CopDesired_;
};

}  // end of tpl namespace

template <size_t JOINT_COORD_SIZE>
using CopEstimator = tpl::CopEstimator<JOINT_COORD_SIZE, double>;

}  // end of switched_model namespace

#include "implementation/CopEstimator.h"

#endif /* COPESTIMATOR_H_ */

