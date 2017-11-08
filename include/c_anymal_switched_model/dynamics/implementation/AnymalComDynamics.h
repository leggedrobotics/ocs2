/*
 * AnymalComDynamics.cpp
 *
 *  Created on: Aug 11, 2017
 *      Author: Jan Carius
 */

namespace anymal {

Eigen::Vector3d AnymalComDynamics::comPositionBaseFrame(
		const joint_coordinate_t& q)
{
	dyn::InertiaProperties inertia; //TODO get from model
	HomogeneousTransforms transforms; //TODO get from model

	return getWholeBodyCOM(inertia, transforms);

	//alternative:
	return InertiaProperties::getCOM_trunk();
}

Eigen::Matrix<double, 6, 6> AnymalComDynamics::comInertia(
		const Eigen::Matrix<double,12,1>& q)
{

	const auto m_trunk = InertiaProperties::getMass_trunk();
	const auto m_total = InertiaProperties::getTotalMass();

	const auto I_total = (m_total/m_trunk)* InertiaProperties::getTensor_trunk();

	return I_total;

	// alternative:
	dyn::JSIM::getWholeBodyInertia(); //total inertia of robot in current config in base frame
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
	return ComInertiaDerivative(q, dq); // TODO
}

Eigen::Matrix<double,6,12> AnymalComDynamics::comMomentumJacobian(
		const Eigen::Matrix<double,12,1>& q)
{
	return ComMomentumJacobian(q); // TODO
}

Eigen::Matrix<double,6,12> AnymalComDynamics::comMomentumJacobianDerivative(
		const Eigen::Matrix<double,12,1>& q,
		const Eigen::Matrix<double,12,1>& dq)
{
	return ComMomentumJacobianDerivative(q, dq); // TODO
}

Eigen::Matrix<double,3,1> AnymalComDynamics::comVelocityInBaseFrame(
		const Eigen::Matrix<double,12,1>& q,
		const Eigen::Matrix<double,12,1>& dq)
{
	return ComVelocityInBaseFrame(q, dq); // TODO
}

}  // end of namespace anymal
