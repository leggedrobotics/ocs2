/*
 * AnymalComDynamics.cpp
 *
 *  Created on: Nov, 2017
 *      Author: Jan Carius
 */


#include <iit/robots/anymal/miscellaneous.h>

namespace anymal {

AnymalComDynamics::AnymalComDynamics() :
		inertiaProperties_(),
		homTransforms_(),
		forceTransforms_(),
		jointSpaceInertiaMatrix_(inertiaProperties_, forceTransforms_)
{ }

Eigen::Vector3d AnymalComDynamics::comPositionBaseFrame(
		const joint_coordinate_t& q)
{
	return iit::ANYmal::getWholeBodyCOM(inertiaProperties_, q, homTransforms_);

	//alternative (only considering the trunk)
	return inertiaProperties_.getCOM_trunk();
}

Eigen::Matrix<double, 6, 6> AnymalComDynamics::comInertia(
		const Eigen::Matrix<double,12,1>& q)
{
	// total inertia of robot in current config in base frame
	jointSpaceInertiaMatrix_.update(q);
	return jointSpaceInertiaMatrix_.getWholeBodyInertia();

	// alternative: use trunk inertia and scale it up
	const auto m_trunk = inertiaProperties_.getMass_trunk();
	const auto m_total = inertiaProperties_.getTotalMass();

	const auto I_total = (m_total/m_trunk)* inertiaProperties_.getTensor_trunk();

	return I_total;
}

Eigen::Matrix<double,4,4> AnymalComDynamics::comHomogeneous(
		const Eigen::Matrix<double,12,1>& q)
{
	Eigen::Matrix<double,4,4> res = Eigen::Matrix<double,4,4>::Identity();
	res.topRightCorner<3,1>() = comPositionBaseFrame(q);

	return res;
}

Eigen::Matrix<double,6,6> AnymalComDynamics::comInertiaDerivative(
		const Eigen::Matrix<double,12,1>& q,
		const Eigen::Matrix<double,12,1>& dq)
{
	return Eigen::Matrix<double,6,6>::Zero(); // massless limbs
}

Eigen::Matrix<double,6,12> AnymalComDynamics::comMomentumJacobian(
		const Eigen::Matrix<double,12,1>& q)
{
	return Eigen::Matrix<double,6,12>::Zero(); // massless limbs
}

Eigen::Matrix<double,6,12> AnymalComDynamics::comMomentumJacobianDerivative(
		const Eigen::Matrix<double,12,1>& q,
		const Eigen::Matrix<double,12,1>& dq)
{
	return Eigen::Matrix<double,6,12>::Zero(); // massless limbs
}

Eigen::Matrix<double,3,1> AnymalComDynamics::comVelocityInBaseFrame(
		const Eigen::Matrix<double,12,1>& q,
		const Eigen::Matrix<double,12,1>& dq)
{
	return Eigen::Matrix<double,3,1>::Zero(); // massless limbs
}

}  // end of namespace anymal
