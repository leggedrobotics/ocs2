/*
 * WeightCompensationForces.h
 *
 *  Created on: Jul 7, 2016
 *      Author: farbod
 */

#ifndef WEIGHTCOMPENSATIONFORCES_H_
#define WEIGHTCOMPENSATIONFORCES_H_

#include <array>
#include <Eigen/Dense>

#include <ocs2_core/misc/EigenQuadSolve.h>

#include "ocs2_switched_model_interface/core/ComModelBase.h"
#include "ocs2_switched_model_interface/core/KinematicsModelBase.h"
#include "ocs2_switched_model_interface/core/SwitchedModel.h"
#include "ocs2_switched_model_interface/misc/SphericalCoordinate.h"

namespace switched_model {

template <size_t JOINT_COORD_SIZE, typename scalar_t = double>
class WeightCompensationForces
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::shared_ptr<WeightCompensationForces<JOINT_COORD_SIZE>> Ptr;

	typedef ComModelBase<JOINT_COORD_SIZE> com_model_t;
	typedef KinematicsModelBase<JOINT_COORD_SIZE> kinematic_model_t;

	typedef Eigen::Matrix<scalar_t,3,1>	vector_3d_t;
	typedef Eigen::Matrix<scalar_t,4,1>	vector_4d_t;
	typedef Eigen::Matrix<scalar_t,3,3>	matrix_3d_t;
	typedef std::array<vector_3d_t,4>  	vector_3d_array_t;
	typedef Eigen::Matrix<scalar_t, Eigen::Dynamic, 1> 				vector_xd_t;
	typedef Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic> matrix_xd_t;

	typedef typename SwitchedModel<JOINT_COORD_SIZE>::contact_flag_t 	 contact_flag_t;
	typedef typename SwitchedModel<JOINT_COORD_SIZE>::base_coordinate_t  base_coordinate_t;
	typedef typename SwitchedModel<JOINT_COORD_SIZE>::joint_coordinate_t joint_coordinate_t;

	WeightCompensationForces(const kinematic_model_t& kinematicModel, const com_model_t& comModel)
	: kinematicModelPtr_(kinematicModel.clone()),
	  comModelPtr_(comModel.clone())
	{}

	WeightCompensationForces(const WeightCompensationForces& rhs)
	: kinematicModelPtr_(rhs.kinematicModelPtr_->clone()),
	  comModelPtr_(rhs.comModelPtr_->clone())
	{}

	~WeightCompensationForces() = default;


	/******************************************************************************************************/
	/******************************************************************************************************/
	/******************************************************************************************************/
	void computeSphericalForces(
			const vector_3d_t& w_gravityVectorconst,
			const contact_flag_t& stanceLegs,
			const joint_coordinate_t& qJoints,
			vector_3d_array_t& sphericalForces)  {

		const scalar_t totalMass = comModelPtr_->totalMass();

		// CoM Position in Base frame
		vector_3d_t b_comPosition = comModelPtr_->comPositionBaseFrame(qJoints);

		Eigen::Matrix<scalar_t,3,4> transform;
		kinematicModelPtr_->update(base_coordinate_t::Zero(), qJoints);
		for (size_t i=0; i<4; i++)
			if (stanceLegs[i] == true) {
				// foot position in the Base frame
				vector_3d_t b_feetPosition;
				kinematicModelPtr_->footPositionBaseFrame(i, b_feetPosition);
				transform.col(i) = SphericalCoordinate::FromSphericalToCartesian(b_feetPosition-b_comPosition).col(0);
			} else
				transform.col(i).setZero();

		// number of stance legs
		size_t numStanceLeg = stanceLegs[0] + stanceLegs[1] + stanceLegs[2] + stanceLegs[3];
		size_t independentContacts = (numStanceLeg<4)? numStanceLeg : 3;

		Eigen::JacobiSVD<Eigen::Matrix<scalar_t,3,4>> transformSVD(transform, Eigen::ComputeFullU | Eigen::ComputeFullV);

		Eigen::Matrix<scalar_t,4,3> inveseSigma = Eigen::Matrix<scalar_t,4,3>::Zero();
		for (size_t i=0; i<independentContacts; i++)
			inveseSigma(i,i) = 1.0/transformSVD.singularValues()(i);

		Eigen::Matrix<scalar_t,4,3> inverseTransform = transformSVD.matrixV().transpose().inverse() * inveseSigma * transformSVD.matrixU().inverse();
		vector_4d_t radialForces = -inverseTransform*(w_gravityVectorconst*totalMass);

		// 4 legs' contact forces in the Spherical coordinate for weight compensation
		for (size_t i=0; i<4; i++)
			sphericalForces[i] << radialForces(i), 0.0, 0.0;
	}


	/******************************************************************************************************/
	/******************************************************************************************************/
	/******************************************************************************************************/
	void computeCartesianForces(
			const vector_3d_t& w_gravityVectorconst,
			const contact_flag_t& stanceLegs,
			const joint_coordinate_t& qJoints,
			vector_3d_array_t& cartesianForces)  {

//		vector_3d_array_t sphericalForces;
//		computeSphericalForces(w_gravityVectorconst, stanceLegs, qJoints, sphericalForces);
//
//		// CoM Position in Base frame
//		vector_3d_t b_comPosition;
//		ComModel::ComPositionBaseFrame(qJoints, b_comPosition);
//
//		matrix_3d_t transform;
//		vector_3d_t b_feetPosition;
//		for (size_t i=0; i<4; i++)
//			if (stanceLegs[i] == true) {
//				// foot position in the Base frame
//				hyq::SwitchedModelKinematics::FootPositionBaseFrame(qJoints, i, b_feetPosition);
//				transform = hyq::SphericalCoordinate::FromSphericalToCartesian(b_feetPosition-b_comPosition);
//				cartesianForces[i] = transform * sphericalForces[i];
//			} else
//				cartesianForces[i].setZero();

		const scalar_t totalMass = comModelPtr_->totalMass();

		// number of stance legs
		size_t numStanceLeg = stanceLegs[0] + stanceLegs[1] + stanceLegs[2] + stanceLegs[3];

		// CoM Position in Base frame
		vector_3d_t b_comPosition = comModelPtr_->comPositionBaseFrame(qJoints);

		// F = A * lambda
		matrix_xd_t A(6, 3*numStanceLeg);
		size_t itr = 0;
		kinematicModelPtr_->update(base_coordinate_t::Zero(), qJoints);
		for (size_t i=0; i<4; i++)
			if (stanceLegs[i] == true) {
				// foot position in the Base frame
				vector_3d_t b_footPosition;
				kinematicModelPtr_->footPositionBaseFrame(i, b_footPosition);

				vector_3d_t com_footPosition = b_footPosition - b_comPosition;

				A.template block<6,3>(0, 3*itr) << switched_model::CrossProductMatrix(com_footPosition),
						matrix_3d_t::Identity();

				itr++;
			}

		 // F
		base_coordinate_t F;
		F << vector_3d_t::Zero(), -w_gravityVectorconst*totalMass;

		// lambda
		// F = A * lambda
		vector_xd_t lambda;
		bool feasibility;
		matrix_xd_t JcStacked = A.transpose();
		SolveQPforForces(F, JcStacked, 3*numStanceLeg, numStanceLeg,
				lambda, feasibility);

		itr = 0;
		for (size_t i=0; i<4; i++)
			if (stanceLegs[i] == true) {
				cartesianForces[i] = lambda.template segment<3>(3*itr);
				itr++;
			} else
				cartesianForces[i].setZero();
	}


	/******************************************************************************************************/
	/******************************************************************************************************/
	/******************************************************************************************************/
	void computeZMPForces(
			const vector_3d_t& w_gravityVectorconst,
			const contact_flag_t& stanceLegs,
			const joint_coordinate_t& qJoints,
			vector_3d_array_t& sphericalForces)  {

		const scalar_t totalMass = comModelPtr_->totalMass();

		// CoM Position in Base frame
		vector_3d_t b_comPosition = comModelPtr_->comPositionBaseFrame(qJoints);
		// feet position
		vector_3d_array_t com_feetPositions;
		vector_3d_t com_feetCenterPosition = vector_3d_t::Zero();
		kinematicModelPtr_->update(qJoints);
		for (size_t i=0; i<4; i++)
			if (stanceLegs[i] == true) {
				// foot position in the Base frame
				vector_3d_t b_footPosition;
				kinematicModelPtr_->footPositionBaseFrame(i, b_footPosition);
				com_feetPositions[i] = b_footPosition - b_comPosition;
				com_feetCenterPosition += com_feetPositions[i];
			} else
				com_feetPositions[i].setZero();

		com_feetCenterPosition /= 3.0;

		matrix_3d_t A;
		A.row(0) << Eigen::Matrix<scalar_t,1,3>(1.0, 1.0, 1.0);
		size_t itr = 0;
		for (size_t i=0; i<4; i++)
			if (stanceLegs[i] == true) {
				A.template block<2,1>(1,itr) = com_feetPositions[i].template head<2>();
				itr++;
			}

		vector_3d_t B;
		B << 1.0, com_feetCenterPosition.template head<2>();
		B *= (-w_gravityVectorconst(2)*totalMass);

		vector_3d_t Fz = A.inverse()*B;

		itr = 0;
		for (size_t i=0; i<4; i++)
			if (stanceLegs[i] == true) {
				sphericalForces[i] = SphericalCoordinate::FromCartesianToSpherical(com_feetPositions[i]) * vector_3d_t(0.0, 0.0, Fz(itr));
				itr++;
			} else
				sphericalForces[i].setZero();

	}


	/******************************************************************************************************/
	/******************************************************************************************************/
	/******************************************************************************************************/
	/**
	 * @brief Solves QP in order to find contact forces. It gets net force on the trunk as an input and
	 * computes contact forces for the fet. Contact forces satisfy friction circle.
	 * @param[in] F: net force on the trunk
	 * @param[in] JcStacked: matrix with stacked Jacobians of the contact feet
	 * @param[in] numForces: total number of forces that have to be optimized (these are decision
	 * 						  varaibles), numForces = 3 * numLegs.
	 * @param[in] numLegs: number of feet that are in contact with the ground
	 * @param[out] contactForces: optimized contact Forces
	 * @param[out] feasibility: feasibility status of the QP
	 */
	static void SolveQPforForces(
			const base_coordinate_t& F,
			const matrix_xd_t& JcStacked,
			const size_t numForces,
			const size_t numLegs,
			vector_xd_t& contactForces,
			bool& feasibility)  {

		// vector for contact Forces and roation matrix
		vector_xd_t lambda(numForces);

		//initialize matrices for quadprog
		matrix_xd_t G(numForces,numForces), CI(numForces, numLegs),CE, Gcopy(numForces,numForces);
		vector_xd_t g0(numForces), ci0(1), ce0;

		// for cost function
		G = JcStacked * JcStacked.transpose() * 2;
		Gcopy = G + matrix_xd_t::Identity(numForces, numForces) * 5e-4;	//convexify since quadprog assumes strong convexity
		g0 = -2 * JcStacked *F;

		// matrices and vectors for inequality constraints
		CI = matrix_xd_t::Zero(numForces, numLegs);
		ci0 = vector_xd_t::Zero(numLegs);

		//set the constraints up
		for(size_t i=0; i<numLegs; i++)
			CI(3*i+2,i) = 1;	// for Fz >= 0


		// invoke quadprog
		scalar_t cost = solve_quadprog(Gcopy, g0,  CE, ce0,  CI, ci0, lambda);

		contactForces = lambda;

		// if the quadprog is infeasible do the batch optimization
		if (cost == std::numeric_limits<scalar_t>::infinity())
			feasibility = false;
		else
			feasibility = true;
	}


private:
	typename kinematic_model_t::Ptr kinematicModelPtr_;
	typename com_model_t::Ptr comModelPtr_;

};

} // end of namespace switched model


#endif /* WEIGHTCOMPENSATIONFORCES_H_ */
