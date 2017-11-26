/*
 * AnymalCom.cpp
 *
 *  Created on: Nov, 2017
 *      Author: Jan Carius
 */

#include <ocs2_anymal_switched_model/dynamics/AnymalCom.h>


namespace anymal {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
AnymalCom::AnymalCom()
	: inertiaProperties_(),
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

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
AnymalCom* AnymalCom::clone() const {

	return new AnymalCom;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void AnymalCom::setJointConfiguration(const joint_coordinate_t& q) {

	jointSpaceInertiaMatrix_.update(q);
	comPositionBaseFrame_ = iit::ANYmal::getWholeBodyCOM(inertiaProperties_, q, homTransforms_);

	comInertia_ = jointSpaceInertiaMatrix_.getWholeBodyInertia();
	double& mass = comInertia_(5,5);
	Eigen::Matrix3d crossComPositionBaseFrame = switched_model::CrossProductMatrix(comPositionBaseFrame_);
	comInertia_.template topLeftCorner<3,3>() += mass*crossComPositionBaseFrame*crossComPositionBaseFrame;
	comInertia_.template topRightCorner<3,3>().setZero();
	comInertia_.template bottomLeftCorner<3,3>().setZero();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Eigen::Vector3d AnymalCom::comPositionBaseFrame(const joint_coordinate_t& q)  {

	return comPositionBaseFrame_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Eigen::Matrix<double, 6, 6> AnymalCom::comInertia(const joint_coordinate_t& q)
{
	// total inertia of robot in the default config in base frame
	return comInertia_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
double AnymalCom::totalMass() const {

	return inertiaProperties_.getTotalMass();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Eigen::Matrix<double,4,4> AnymalCom::comHomogeneous(const joint_coordinate_t& q)
{
	Eigen::Matrix<double,4,4> res = Eigen::Matrix<double,4,4>::Identity();
	res.topRightCorner<3,1>() = comPositionBaseFrame(q);
	return res;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Eigen::Matrix<double,4,4> AnymalCom::comHomogeneous() {
	Eigen::Matrix<double,4,4> res = Eigen::Matrix<double,4,4>::Identity();
	res.topRightCorner<3,1>() = comPositionBaseFrame();
	return res;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Eigen::Matrix<double,6,6> AnymalCom::comInertiaDerivative(
		const joint_coordinate_t& q,
		const joint_coordinate_t& dq)
{
	return Eigen::Matrix<double,6,6>::Zero(); // massless limbs
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Eigen::Matrix<double,6,12> AnymalCom::comMomentumJacobian(
		const joint_coordinate_t& q)
{
	return Eigen::Matrix<double,6,12>::Zero(); // massless limbs
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Eigen::Matrix<double,6,12> AnymalCom::comMomentumJacobianDerivative(
		const joint_coordinate_t& q,
		const joint_coordinate_t& dq)
{
	return Eigen::Matrix<double,6,12>::Zero(); // massless limbs
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Eigen::Matrix<double,3,1> AnymalCom::comVelocityInBaseFrame(
		const joint_coordinate_t& q,
		const joint_coordinate_t& dq)
{
	return Eigen::Matrix<double,3,1>::Zero(); // massless limbs
}


}  // end of namespace anymal
