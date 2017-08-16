/*
 * CoMDynamics.h
 *
 *  Created on: Feb 9, 2016
 *      Author: farbod
 */

#ifndef HYQ_COMDYNAMICS_H_
#define HYQ_COMDYNAMICS_H_

#include <array>
#include <memory>
#include <iostream>
#include <Eigen/Dense>

#include <dynamics/ControlledSystemBase.h>

#include <CoM/CoMGenerated.h>

#include "misc/SphericalCoordinate.h"
#include "kinematics/SwitchedModelKinematics.h"

namespace hyq {

class CoMDynamics : public ocs2::ControlledSystemBase<12, 12>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef ocs2::ControlledSystemBase<12,12> Base;
	typedef Eigen::Matrix<double,6,1>  com_coordinate_t;
	typedef Eigen::Matrix<double,12,1> joints_coordinate_t;


	CoMDynamics(const double& gravitationalAcceleration=9.81, const bool& constrainedIntegration=true)

	: o_gravityVector_(0.0, 0.0, -gravitationalAcceleration),
	  constrainedIntegration_(constrainedIntegration)
	{
		if (gravitationalAcceleration<0)  throw std::runtime_error("Gravitational acceleration should be a positive value.");
		qJointsDefault_ << -0.2, 0.723, -1.458,	 -0.2, 0.723, -1.458,  -0.2, -0.723, 1.458,  -0.2, -0.723, 1.458;
		dqJointsDefault_.setZero();
	}

	virtual ~CoMDynamics() {}

	/*
	 * clone this class.
	 */
	virtual std::shared_ptr<Base> clone() const  override {
		return std::allocate_shared<CoMDynamics, Eigen::aligned_allocator<CoMDynamics>>( Eigen::aligned_allocator<CoMDynamics>(), *this);
	};

	/*
	 * Initializes the model: This method should always be called at the very first call of the model.
	 */
	virtual void initializeModel(const std::vector<size_t>& systemStockIndexes, const std::vector<scalar_t>& switchingTimes,
			const state_vector_t& initState, const size_t& activeSubsystemIndex=0, const char* algorithmName=NULL) override;

	/*
	 * Calculates the CoM state time evolution based on the current CoM state (x), contact forces input
	 * at all feet (u), the stance leg configuration (stanceLegs_), the current joints' angel (qJoints_),
	 * and the current joints' velocities (dqJoints_).
	 *
	 * The CoM state (x) consists of:
	 * 		+ Base orientation w.r.t Origin frame (3-states)
	 * 		+ CoM position w.r.t Origin frame (3-states)
	 * 		+ CoM local angular and linear velocities in CoM frame (inertia frame coincide at CoM and is
	 * 		  parallel to Base frame) (6-states)
	 *
	 * The control input (u) consists of:
	 * 		+ feet's contact forces in the CoM frame with spherical coordinate (12-inputs = 3-XYZ-forces * 4-feet)
	 *
	 * The CoM state time derivatives (dxdt) consists of:
	 * 		+ Base angular velocity w.r.t Origin frame (3-states)
	 * 		+ CoM linear velocity w.r.t Origin frame (3-states)
	 * 		+ CoM angular and linear accelerations w.r.t CoM frame (6-states)
	 *
	 */
	virtual void computeDerivative(const scalar_t& t,
			const state_vector_t& x,
			const control_vector_t& u,
			state_vector_t& dxdt) override;

	/*
	 * set joints' angel and angular velocity, and stance leg configuration. This data is
	 * required in computeDerivative() method.
	 */
	void setData(const std::array<bool,4>& stanceLegs,
			const joints_coordinate_t& qJoints,
			const joints_coordinate_t& dqJoints);

	/*
	 * Calculates the Base orientation and position based on the current joints' angel (qJoints)
	 * and the current CoM orientation and position.
	 */
	static void CalculateBasePose(const joints_coordinate_t& qJoints,
			const com_coordinate_t& comPose,
			com_coordinate_t& basePose);

	/*
	 * Calculates the Base local velocities based on the current joints' angel (qJoints),
	 * the current joints' velocities (dqJoints), CoM local velocities (comLocalVelocities).
	 */
	static void CalculateBaseLocalVelocities(const joints_coordinate_t& qJoints,
			const joints_coordinate_t& dqJoints,
			const com_coordinate_t& comLocalVelocities,
			com_coordinate_t& baseLocalVelocities);

	/*
	 * user interface for StanceLegs and FeetPositions
	 */
	void getStanceLegs(std::array<bool,4>& stanceLegs) const  { stanceLegs = stanceLegs_; }
	void getFeetPositions(std::array<Eigen::Vector3d,4>& b_base2StanceFeet) const { b_base2StanceFeet = com_base2StanceFeet_; }
	/*
	 * user interface for CoM dynamics elements.
	 * Note these values are updated after each call of computeDerivative() method.
	 */
	Eigen::Matrix<double, 6, 6>  getM() const { return M_;  }
	Eigen::Matrix<double, 6, 1>  getC() const { return C_;  }
	Eigen::Matrix<double, 6, 1>  getG() const { return M_*MInverseG_;  }
	Eigen::Matrix<double, 6, 6>  getMInverse() const { return MInverse_;  }

	// Computes the matrix which transforms derivatives of angular velocities in the body frame to Euler angles derivatives
	static Eigen::Matrix3d AngularVelocitiesToEulerAngleDerivativesMatrix (Eigen::Vector3d eulerAngles);

private:

	Eigen::Vector3d o_gravityVector_;

	bool constrainedIntegration_;

	std::array<bool,4> stanceLegs_;
	joints_coordinate_t qJoints_;
	joints_coordinate_t dqJoints_;
	joints_coordinate_t qJointsDefault_;
	joints_coordinate_t dqJointsDefault_;

	Eigen::Vector3d com_base2CoM_;
	Eigen::Matrix<double,6,12> b_comJacobain_;
	std::array<Eigen::Vector3d,4> com_base2StanceFeet_;

	// Inertia matrix and its derivative
	Eigen::Matrix<double, 6, 6> M_;
	Eigen::Matrix<double, 6, 6> MInverse_;
	Eigen::Matrix<double, 6, 6> dMdt_;
	// Coriolis and centrifugal forces
	Eigen::Matrix<double, 6, 1> C_;
	// gravity effect on CoM in CoM coordinate
	Eigen::Matrix<double, 6, 1> MInverseG_;

};

}  // end of hyq namespace

#endif /* HYQ_COMDYNAMICS_H_ */
