/*
 * AnymalCom.h
 *
 *  Created on: Aug 11, 2017
 *      Author: Jan Carius
 */

#ifndef ANYMAL_ANYMALCOM_H_
#define ANYMAL_ANYMALCOM_H_

#include <c_switched_model_interface/core/ComModelBase.h>

#include <iit/robots/anymal/inertia_properties.h>
#include <iit/robots/anymal/transforms.h>
#include <iit/robots/anymal/jsim.h>
#include <iit/robots/anymal/miscellaneous.h>

namespace anymal
{

class AnymalCom : public switched_model::ComModelBase<12>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum { LF=0,  RF=1,  LH=2,  RH=3 };

	typedef switched_model::ComModelBase<12> Base;
	typedef Base::joint_coordinate_t joint_coordinate_t;

	/**
	 * Constructor needed for initialization
	 */
	AnymalCom();

	/**
	 * Clone AnymalCom class.
	 */
	std::shared_ptr<Base> clone() const override;

	/**
	 * Set default joint configuration. Updates the CoM position and inertia
	 */
	void setJointConfiguration(const joint_coordinate_t& q);

	/**
	 * calculate CoM Position in Base frame for given joint q
	 */
	Eigen::Vector3d comPositionBaseFrame(const joint_coordinate_t& q) override;

	/**
	  calculate CoM Position in Base frame for default q
	 */
	Eigen::Vector3d comPositionBaseFrame() {return comPositionBaseFrame_;}

	/**
	 * calculate homogeneous transformation base -> CoM for given q
	 */
	Eigen::Matrix<double,4,4> comHomogeneous(const joint_coordinate_t& q) override;

	/**
	 * calculate homogeneous transformation base -> CoM for default q
	 */
	Eigen::Matrix<double,4,4> comHomogeneous();

	/**
	 * calculate CoM Velocity in Base Frame
	 */
	Eigen::Matrix<double,3,1> comVelocityInBaseFrame(
			const Eigen::Matrix<double,12,1>& q,
			const Eigen::Matrix<double,12,1>& dq) override;

	/**
	 * calculate CoM Inertia for given q
	 */
	Eigen::Matrix<double, 6, 6> comInertia(const joint_coordinate_t& q) override;

	/**
	 * Total mass of robot
	 * @return mass in kg
	 */
	double totalMass() const override;

	/**
	 * calculate CoM Inertia for default q
	 */
	Eigen::Matrix<double, 6, 6> comInertia() {return comInertia_;}

	/**
	 * calculate CoM Inertia Derivative
	 */
	Eigen::Matrix<double,6,6> comInertiaDerivative(
			const joint_coordinate_t& q,
			const joint_coordinate_t& dq) override;

	/**
	 * calculate CoM Momentum Jacobian
	 * i.e. p_com = J_p_com(q) * dq
	 * Note: excluding the contribution of the trunk inertia
	 */
	Eigen::Matrix<double,6,12> comMomentumJacobian(const joint_coordinate_t& q) override;

	/**
	 * calculate CoM Momentum Jacobian Derivative
	 */
	Eigen::Matrix<double,6,12> comMomentumJacobianDerivative(
			const joint_coordinate_t& q,
			const joint_coordinate_t& dq) override;


private:

	iit::ANYmal::dyn::InertiaProperties inertiaProperties_;
	iit::ANYmal::HomogeneousTransforms homTransforms_;
	iit::ANYmal::ForceTransforms forceTransforms_;
	iit::ANYmal::dyn::JSIM jointSpaceInertiaMatrix_;

	// cached values for current default joint configuration
	Eigen::Vector3d comPositionBaseFrame_;
	Eigen::Matrix<double,6,6> comInertia_;

};

}  // end of anymal namespace

#endif /* ANYMAL_ANYMALCOM_H_ */
