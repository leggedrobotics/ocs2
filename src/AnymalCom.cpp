/*
 * AnymalCom.cpp
 *
 *  Created on: Nov, 2017
 *      Author: Jan Carius
 */

#include <ocs2_anymal_switched_model/dynamics/AnymalCom.h>


namespace anymal {

AnymalCom::AnymalCom() :
		inertiaProperties_(),
		homTransforms_(),
		forceTransforms_(),
		jointSpaceInertiaMatrix_(inertiaProperties_, forceTransforms_)
{
	joint_coordinate_t defaultJointConfig;
	defaultJointConfig << -0.1,  0.7, -1.0,
												 0.1,  0.7, -1.0,
												-0.1, -0.7,  1.0,
												 0.1, -0.7,  1.0;
	setJointConfiguration(defaultJointConfig);
}

void AnymalCom::setJointConfiguration(const joint_coordinate_t& q)
{
	jointSpaceInertiaMatrix_.update(q);
	comPositionBaseFrame_ = iit::ANYmal::getWholeBodyCOM(inertiaProperties_,
			q, homTransforms_);
	comInertia_ = jointSpaceInertiaMatrix_.getWholeBodyInertia();
}

Eigen::Vector3d AnymalCom::comPositionBaseFrame(const joint_coordinate_t& q)
{
	return iit::ANYmal::getWholeBodyCOM(inertiaProperties_, q, homTransforms_);

	// //alternative: only consider the trunk
	// return inertiaProperties_.getCOM_trunk();
}

Eigen::Matrix<double, 6, 6> AnymalCom::comInertia(const joint_coordinate_t& q)
{
	// total inertia of robot in current config in base frame
	jointSpaceInertiaMatrix_.update(q);
	return jointSpaceInertiaMatrix_.getWholeBodyInertia();

	// // alternative: use trunk inertia and scale it up
	// const auto m_trunk = inertiaProperties_.getMass_trunk();
	// const auto m_total = inertiaProperties_.getTotalMass();
	//
	// const auto I_total = (m_total/m_trunk)* inertiaProperties_.getTensor_trunk();
	// return I_total;
}

Eigen::Matrix<double,4,4> AnymalCom::comHomogeneous(const joint_coordinate_t& q)
{
	Eigen::Matrix<double,4,4> res = Eigen::Matrix<double,4,4>::Identity();
	res.topRightCorner<3,1>() = comPositionBaseFrame(q);
	return res;
}

Eigen::Matrix<double,4,4> AnymalCom::comHomogeneous()
{
	Eigen::Matrix<double,4,4> res = Eigen::Matrix<double,4,4>::Identity();
	res.topRightCorner<3,1>() = comPositionBaseFrame();
	return res;
}

Eigen::Matrix<double,6,6> AnymalCom::comInertiaDerivative(
		const joint_coordinate_t& q,
		const joint_coordinate_t& dq)
{
	return Eigen::Matrix<double,6,6>::Zero(); // massless limbs
}

Eigen::Matrix<double,6,12> AnymalCom::comMomentumJacobian(
		const joint_coordinate_t& q)
{
	return Eigen::Matrix<double,6,12>::Zero(); // massless limbs
}

Eigen::Matrix<double,6,12> AnymalCom::comMomentumJacobianDerivative(
		const joint_coordinate_t& q,
		const joint_coordinate_t& dq)
{
	return Eigen::Matrix<double,6,12>::Zero(); // massless limbs
}

Eigen::Matrix<double,3,1> AnymalCom::comVelocityInBaseFrame(
		const joint_coordinate_t& q,
		const joint_coordinate_t& dq)
{
	return Eigen::Matrix<double,3,1>::Zero(); // massless limbs
}

}  // end of namespace anymal
