/*
 * KinematicsModelBase.h
 *
 *  Created on: Aug 3, 2017
 *      Author: Farbod
 */

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void KinematicsModelBase<JOINT_COORD_SIZE>::update(const generalized_coordinate_t& generalizedCoordinate)
{
	update(generalizedCoordinate.template head<6>(), generalizedCoordinate.template tail<JOINT_COORD_SIZE>());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
template <typename BASE_COORDINATE, typename JOINT_COORDINATE>
void KinematicsModelBase<JOINT_COORD_SIZE>::update(const Eigen::DenseBase<BASE_COORDINATE>& qBase,
		const Eigen::DenseBase<JOINT_COORDINATE>& qJoint) {

	qBase_  = qBase;
	qJoint_ = qJoint;
	b_R_o_  = RotationMatrixOrigintoBase(qBase_.template head<3>());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void KinematicsModelBase<JOINT_COORD_SIZE>::feetPositionsBaseFrame(std::array<Eigen::Vector3d,4>& feetPositions)
{
	for (size_t i=0; i<4; i++)
		footPositionBaseFrame(i, feetPositions[i]);
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void KinematicsModelBase<JOINT_COORD_SIZE>::footPositionOriginFrame(const size_t& footIndex, Eigen::Vector3d& footPosition)
{
	// calculate foot position in Base frame
	Eigen::Vector3d b_footPosition;
	footPositionBaseFrame(footIndex, b_footPosition);
	// calculate foot position in Origin frame
	footPosition = b_R_o_.transpose()*b_footPosition + qBase_.template tail<3>();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void KinematicsModelBase<JOINT_COORD_SIZE>::footPositionOriginFrame(
		const generalized_coordinate_t& generalizedCoordinate,
		const size_t& footIndex, Eigen::Vector3d& footPosition)
{
	// update the class
	update(generalizedCoordinate);
	// calculate foot position in Origin frame
	footPositionOriginFrame(footIndex, footPosition);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void KinematicsModelBase<JOINT_COORD_SIZE>::feetPositionsOriginFrame(
		std::array<Eigen::Vector3d,4>& feetPositions)
{
	for (size_t i=0; i<4; i++)
		footPositionOriginFrame(i, feetPositions[i]);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void KinematicsModelBase<JOINT_COORD_SIZE>::feetPositionsOriginFrame(
		const generalized_coordinate_t& generalizedCoordinate,
			std::array<Eigen::Vector3d,4>& feetPositions)
{
	// update the class
	update(generalizedCoordinate);
	// calculate feet's positions in Origin frame
	feetPositionsOriginFrame(feetPositions);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void KinematicsModelBase<JOINT_COORD_SIZE>::FromBaseJacobianToInertiaJacobian(
		const Eigen::Matrix3d& i_R_b,
		const Eigen::Vector3d& b_r_point,
		const Eigen::Matrix<double,6,JOINT_COORD_SIZE>& b_J_point,
		Eigen::Matrix<double,6,JOINT_COORD_SIZE + 6>& i_J_point)
{
	// rotation
	i_J_point.template topRows<3>() << i_R_b,  Eigen::Matrix3d::Zero(),  i_R_b*b_J_point.template topRows<3>();
	// translation
	i_J_point.template bottomRows<3>() << -i_R_b*CrossProductMatrix(b_r_point), i_R_b, i_R_b*b_J_point.template bottomRows<3>();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
Eigen::Matrix3d KinematicsModelBase<JOINT_COORD_SIZE>::rotationMatrixOrigintoBase() const  {

	return b_R_o_;
}

