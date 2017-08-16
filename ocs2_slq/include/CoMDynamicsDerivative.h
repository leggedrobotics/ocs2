/*
 * CoMDynamicsDerivative.h
 *
 *  Created on: Feb 10, 2016
 *      Author: farbod
 */

#ifndef HYQ_COMDYNAMICSDERIVATIVE_H_
#define HYQ_COMDYNAMICSDERIVATIVE_H_

#include <array>
#include <memory>
#include <iostream>
#include <Eigen/Dense>

#include <dynamics/DerivativesBase.h>

#include <CoM/CoMGenerated.h>

#include "misc/SphericalCoordinate.h"
#include "kinematics/SwitchedModelKinematics.h"
#include "CoMDynamics.h"

namespace hyq {

class CoMDynamicsDerivative : public ocs2::DerivativesBase<12,12>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef ocs2::DerivativesBase<12,12> Base;
	typedef Eigen::Matrix<double,12,1>  joints_coordinate_t;
	typedef Eigen::Matrix<double,12,12> state_joint_matrix_t;
	typedef Eigen::Matrix<double,6,12>  base_jacobian_matrix_t;

	CoMDynamicsDerivative(const double& gravitationalAcceleration=9.81, const bool& constrainedIntegration=true)

	: o_gravityVector_(0.0, 0.0, -gravitationalAcceleration),
	  constrainedIntegration_(constrainedIntegration)
	{
		if (gravitationalAcceleration<0)  throw std::runtime_error("Gravitational acceleration should be a positive value.");
		qJointsDefault_ << -0.2, 0.723, -1.458,	 -0.2, 0.723, -1.458,  -0.2, -0.723, 1.458,  -0.2, -0.723, 1.458;
		dqJointsDefault_.setZero();
	}

	virtual ~CoMDynamicsDerivative() {}

	/*
	 * clone this class.
	 */
	virtual std::shared_ptr<Base> clone() const override  {
		return std::allocate_shared<CoMDynamicsDerivative, Eigen::aligned_allocator<CoMDynamicsDerivative>>(
				Eigen::aligned_allocator<CoMDynamicsDerivative>(), *this);
	}

	/*
	 * Initializes the model: This method should always be called at the very first call of the model.
	 */
	virtual void initializeModel(const std::vector<size_t>& systemStockIndexes, const std::vector<scalar_t>& switchingTimes,
			const state_vector_t& initState, const size_t& activeSubsystemIndex=0, const char* algorithmName=NULL) override;

	/*
	 * set the current state and contact force input
	 */
	virtual void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const control_vector_t& u) override;

	/*
	 * set generalized coordinates, their velocities, stance leg configuration, and optionally stance feet
	 * position w.r.t. the Origin frame. This data is required in computeDerivative() method.
	 */
	void setData(const std::array<bool,4>& stanceLegs,
			const joints_coordinate_t& qJoints,
			const joints_coordinate_t& dqJoints);

	/*
	 * calculate and retrieve the A matrix
	 */
	virtual void getDerivativeState(state_matrix_t& A)  override;

	/*
	 * calculate and retrieve the B matrix
	 */
	virtual void getDerivativesControl(control_gain_matrix_t& B)  override;

	/*
	 * calculate and retrieve the approximate derivatives w.r.t. joints
	 */
	void getApproximateDerivativesJoint(state_joint_matrix_t& partrialF_q);
	void getNumericalDerivativesJoint(state_joint_matrix_t& partrialF_q);

	/*
	 * calculate and retrieve the approximate derivatives w.r.t. joints' velocities
	 */
	void getApproximateDerivativesJointVelocity(state_joint_matrix_t& partrialF_dq);

	/*
	 * user interface for CoM dynamics elements.
	 */
	void getBase2CoMInComFrame(Eigen::Vector3d& com_base2CoM) const
	{ com_base2CoM = com_base2CoM_; }
	void getCom2StanceFeetInComFrame(std::array<Eigen::Vector3d,4>& com_com2StanceFeet) const
	{ com_com2StanceFeet = com_com2StanceFeet_; }
	void getComJacobianInBaseFrame(base_jacobian_matrix_t& b_comJacobain) const
	{ b_comJacobain = b_comJacobain_; }
	void getFeetJacobiansInBaseFrame(std::array<base_jacobian_matrix_t,4>& b_feetJacobains) const
	{ b_feetJacobains = b_feetJacobains_; }
	void getComJacobianTimeDerivativeInBaseFrame(base_jacobian_matrix_t& b_comJacobainTimeDerivative) const
	{ b_comJacobainTimeDerivative = b_comJacobainTimeDerivative_; }
	void getRotationMatrixOrigintoBase(Eigen::Matrix3d& b_R_o) const
	{ b_R_o = b_R_o_; }


	/**
	 * to map local angular velocity \omega_W expressed in body coordinates, to changes in Euler Angles expressed in an inertial frame q_I
	 * we have to map them via \dot{q}_I = H \omega_W, where H is the matrix defined in kindr getMappingFromLocalAngularVelocityToDiff.
	 * You can see the kindr cheat sheet to figure out how to build this matrix. The following code computes the Jacobian of \dot{q}_I
	 * with respect to \q_I and \omega_W. Thus the lower part of the Jacobian is H and the upper part is dH/dq_I \omega_W. We include
	 * both parts for more efficient computation. The following code is computed using auto-diff.
	 * @param eulerAnglesXyz
	 * @param angularVelocity
	 * @return
	 */
	static Eigen::Matrix<double,6,3> JacobianOfAngularVelocityMapping(const Eigen::Vector3d& eulerAnglesXyz, const Eigen::Vector3d& angularVelocity);

private:
	Eigen::Vector3d o_gravityVector_;

	bool constrainedIntegration_;
	const bool useInertiaMatrixDerivate_ = false;

	std::array<bool,4> stanceLegs_;
	joints_coordinate_t qJoints_;
	joints_coordinate_t dqJoints_;
	joints_coordinate_t qJointsDefault_;
	joints_coordinate_t dqJointsDefault_;

	Eigen::Matrix3d b_R_o_;

	// local angular velocity of Base (w_W_base)
	Eigen::Vector3d w_W_base_;

	// jacobian of the angular velocities to Euler angle derivatives transformation D(d\theta / dt) = [ d/d(theta), d/d(w_omega_b) ]
	Eigen::Matrix<double,3,6> jacobianOfAngularVelocityMapping_;

	Eigen::Vector3d com_base2CoM_;
	base_jacobian_matrix_t b_comJacobain_;
	base_jacobian_matrix_t b_comJacobainTimeDerivative_;
	std::array<Eigen::Vector3d,4> com_com2StanceFeet_;

	std::array<base_jacobian_matrix_t,4> b_feetJacobains_;

	// Inertia matrix
	Eigen::Matrix<double, 6, 6> M_;
	Eigen::Matrix<double, 6, 6> dMdt_;
	Eigen::Matrix<double, 6, 6> MInverse_;
	std::array<Eigen::Matrix<double,6,6>, 12> partialM_;

};

}  // end of hyq namespace

#endif /* HYQ_COMDYNAMICSDERIVATIVE_H_ */
