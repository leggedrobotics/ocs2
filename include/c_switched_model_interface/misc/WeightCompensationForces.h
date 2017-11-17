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
#include <c_switched_model_interface/core/SwitchedModel.h>
#include <c_switched_model_interface/misc/SphericalCoordinate.h>

namespace switched_model {

template<typename ComModel, typename KinematicsModel>
class WeightCompensationForces
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef Eigen::Matrix<double,12,1>     joint_coordinate_t;
	typedef std::array<Eigen::Vector3d,4>  vector_3d_array;

	WeightCompensationForces() {}
	~WeightCompensationForces() {}


	void ComputeSphericalForces(const Eigen::Vector3d& w_gravityVectorconst,
			const std::array<bool,4>& stanceLegs, const joint_coordinate_t& qJoints,
			vector_3d_array& sphericalForces)  {

		const double totalMass = comModel_.totalMass();

		// CoM Position in Base frame
		Eigen::Vector3d b_comPosition = comModel_.comPositionBaseFrame(qJoints);

		Eigen::Matrix<double,3,4> transform;
		kinModel_.update(Eigen::Matrix<double,6,1>::Zero(), qJoints);
		for (size_t i=0; i<4; i++)
			if (stanceLegs[i] == true) {
				// foot position in the Base frame
				Eigen::Vector3d b_feetPosition;
				kinModel_.footPositionBaseFrame(i, b_feetPosition);
				transform.col(i) = SphericalCoordinate::FromSphericalToCartesian(b_feetPosition-b_comPosition).col(0);
			} else
				transform.col(i).setZero();

		// number of stance legs
		size_t numStanceLeg = stanceLegs[0] + stanceLegs[1] + stanceLegs[2] + stanceLegs[3];
		size_t independentContacts = (numStanceLeg<4)? numStanceLeg : 3;

		Eigen::JacobiSVD<Eigen::Matrix<double,3,4>> transformSVD(transform, Eigen::ComputeFullU | Eigen::ComputeFullV);

		Eigen::Matrix<double,4,3> inveseSigma = Eigen::MatrixXd::Zero(4,3);
		for (size_t i=0; i<independentContacts; i++)
			inveseSigma(i,i) = 1.0/transformSVD.singularValues()(i);

		Eigen::Matrix<double,4,3> inverseTransform = transformSVD.matrixV().transpose().inverse() * inveseSigma * transformSVD.matrixU().inverse();
		Eigen::Vector4d radialForces = -inverseTransform*(w_gravityVectorconst*totalMass);

		// 4 legs' contact forces in the Spherical coordinate for weight compensation
		for (size_t i=0; i<4; i++)
			sphericalForces[i] << radialForces(i), 0.0, 0.0;
	}


	void ComputeCartesianForces(const Eigen::Vector3d& w_gravityVectorconst,
			const std::array<bool,4>& stanceLegs, const joint_coordinate_t& qJoints,
			vector_3d_array& cartesianForces)  {

//		vector_3d_array sphericalForces;
//		ComputeSphericalForces(w_gravityVectorconst, stanceLegs, qJoints, sphericalForces);
//
//		// CoM Position in Base frame
//		Eigen::Vector3d b_comPosition;
//		ComModel::ComPositionBaseFrame(qJoints, b_comPosition);
//
//		Eigen::Matrix3d transform;
//		Eigen::Vector3d b_feetPosition;
//		for (size_t i=0; i<4; i++)
//			if (stanceLegs[i] == true) {
//				// foot position in the Base frame
//				hyq::SwitchedModelKinematics::FootPositionBaseFrame(qJoints, i, b_feetPosition);
//				transform = hyq::SphericalCoordinate::FromSphericalToCartesian(b_feetPosition-b_comPosition);
//				cartesianForces[i] = transform * sphericalForces[i];
//			} else
//				cartesianForces[i].setZero();

		const double totalMass = comModel_.totalMass();

		// number of stance legs
		size_t numStanceLeg = stanceLegs[0] + stanceLegs[1] + stanceLegs[2] + stanceLegs[3];

		// CoM Position in Base frame
		Eigen::Vector3d b_comPosition = comModel_.comPositionBaseFrame(qJoints);

		// F = A * lambda
		Eigen::MatrixXd A(6, 3*numStanceLeg);
		size_t itr = 0;
		kinModel_.update(Eigen::Matrix<double,6,1>::Zero(), qJoints);
		for (size_t i=0; i<4; i++)
			if (stanceLegs[i] == true) {
				// foot position in the Base frame
				Eigen::Vector3d b_footPosition;
				kinModel_.footPositionBaseFrame(i, b_footPosition);

				Eigen::Vector3d com_footPosition = b_footPosition - b_comPosition;

				A.block<6,3>(0, 3*itr) << switched_model::CrossProductMatrix(com_footPosition),
						Eigen::Matrix3d::Identity();

				itr++;
			}

		 // F
		Eigen::Matrix<double,6,1> F;
		F << Eigen::Vector3d::Zero(), -w_gravityVectorconst*totalMass;

		// lambda
		// F = A * lambda
		Eigen::VectorXd lambda;
		bool feasibility;
		Eigen::MatrixXd JcStacked = A.transpose();
		SolveQPforForces(F, JcStacked, 3*numStanceLeg, numStanceLeg,
				lambda, feasibility);

		itr = 0;
		for (size_t i=0; i<4; i++)
			if (stanceLegs[i] == true) {
				cartesianForces[i] = lambda.segment<3>(3*itr);
				itr++;
			} else
				cartesianForces[i].setZero();
	}


	void ComputeZMPForces(const Eigen::Vector3d& w_gravityVectorconst,
			const std::array<bool,4>& stanceLegs, const joint_coordinate_t& qJoints,
			vector_3d_array& sphericalForces)  {

		const double totalMass = comModel_.totalMass();

		// CoM Position in Base frame
		Eigen::Vector3d b_comPosition = comModel_.comPositionBaseFrame(qJoints);
		// feet position
		std::array<Eigen::Vector3d,4> com_feetPositions;
		Eigen::Vector3d com_feetCenterPosition = Eigen::Vector3d::Zero();
		kinModel_.update(qJoints);
		for (size_t i=0; i<4; i++)
			if (stanceLegs[i] == true) {
				// foot position in the Base frame
				Eigen::Vector3d b_footPosition;
				kinModel_.footPositionBaseFrame(i, b_footPosition);
				com_feetPositions[i] = b_footPosition - b_comPosition;
				com_feetCenterPosition += com_feetPositions[i];
			} else
				com_feetPositions[i].setZero();

		com_feetCenterPosition /= 3.0;

		Eigen::Matrix3d A;
		A.row(0) << Eigen::RowVector3d(1.0, 1.0, 1.0);
		size_t itr = 0;
		for (size_t i=0; i<4; i++)
			if (stanceLegs[i] == true) {
				A.block<2,1>(1,itr) = com_feetPositions[i].head<2>();
				itr++;
			}

		Eigen::Vector3d B;
		B << 1.0, com_feetCenterPosition.head<2>();
		B *= (-w_gravityVectorconst(2)*totalMass);

		Eigen::Vector3d Fz = A.inverse()*B;

		itr = 0;
		for (size_t i=0; i<4; i++)
			if (stanceLegs[i] == true) {
				sphericalForces[i] = SphericalCoordinate::FromCartesianToSpherical(com_feetPositions[i]) * Eigen::Vector3d(0.0, 0.0, Fz(itr));
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
	static void SolveQPforForces(const Eigen::Matrix<double,6,1>& F,
			const Eigen::MatrixXd& JcStacked,
			const size_t numForces,
			const size_t numLegs,
			Eigen::VectorXd& contactForces,
			bool& feasibility){

		// vector for contact Forces and roation matrix
		Eigen::VectorXd lambda(numForces);

		//initialize matrices for quadprog
		Eigen::MatrixXd G(numForces,numForces), CI(numForces, numLegs),CE, Gcopy(numForces,numForces);
		Eigen::VectorXd g0(numForces), ci0(1), ce0;

		// for cost function
		G = JcStacked * JcStacked.transpose() * 2;
		Gcopy = G + Eigen::MatrixXd::Identity(numForces, numForces) * 5e-4;	//convexify since quadprog assumes strong convexity
		g0 = -2 * JcStacked *F;

		// matrices and vectors for inequality constraints
		CI = Eigen::MatrixXd::Zero(numForces, numLegs);
		ci0 = Eigen::VectorXd::Zero(numLegs);

		//set the constraints up
		for(size_t i=0; i<numLegs; i++)
			CI(3*i+2,i) = 1;	// for Fz >= 0


		// invoke quadprog
		double cost = solve_quadprog(Gcopy, g0,  CE, ce0,  CI, ci0, lambda);

		contactForces = lambda;

		// if the quadprog is infeasible do the batch optimization
		if (cost == std::numeric_limits<double>::infinity())
			feasibility = false;
		else
			feasibility = true;
	}


private:
	ComModel comModel_;
	KinematicsModel kinModel_;

};

} // end of namespace switched model


#endif /* WEIGHTCOMPENSATIONFORCES_H_ */
