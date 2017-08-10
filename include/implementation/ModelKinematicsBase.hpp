/*
 * ModelKinematicsBase.hpp
 *
 *  Created on: Aug 3, 2017
 *      Author: sasutosh
 */

template< class DerivedClassType, size_t BASE_COORD_SIZE, size_t JOINT_COORD_SIZE >
void ModelKinematicsBase<DerivedClassType, BASE_COORD_SIZE, JOINT_COORD_SIZE>::update(const generalized_coordinate_t& generalizedCoordinate)
{
	qBase_  = generalizedCoordinate.head<BASE_COORDINATE_SIZE>();
	qJoint_ = generalizedCoordinate.tail<JOINT_COORDINATE_SIZE>();
	b_R_o_  = RotationMatrixOrigintoBase(qBase_.head<3>());
}

template< class DerivedClassType, size_t BASE_COORD_SIZE, size_t JOINT_COORD_SIZE >
void ModelKinematicsBase<DerivedClassType, BASE_COORD_SIZE, JOINT_COORD_SIZE>::footPositionBaseFrame(const size_t& footIndex, Eigen::Vector3d& footPosition)
{
	FootPositionBaseFrame(qJoint_, footIndex, footPosition);
}

template< class DerivedClassType, size_t BASE_COORD_SIZE, size_t JOINT_COORD_SIZE >
void ModelKinematicsBase<DerivedClassType, BASE_COORD_SIZE, JOINT_COORD_SIZE>::feetPositionsBaseFrame(std::array<Eigen::Vector3d,4>& feetPositions)
{
	FeetPositionsBaseFrame(qJoint_, feetPositions);
}

template< class DerivedClassType, size_t BASE_COORD_SIZE, size_t JOINT_COORD_SIZE >
static void ModelKinematicsBase<DerivedClassType, BASE_COORD_SIZE, JOINT_COORD_SIZE>::FootPositionBaseFrame(const joint_coordinate_t& jointCoordinate,
		const size_t& footIndex, Eigen::Vector3d& footPosition)
{
	DerivedClassType::FootPositionBaseFrame(jointCoordinate, footIndex, footPosition);
}

template< class DerivedClassType, size_t BASE_COORD_SIZE, size_t JOINT_COORD_SIZE >
static void ModelKinematicsBase<DerivedClassType, BASE_COORD_SIZE, JOINT_COORD_SIZE>::FeetPositionsBaseFrame(const joint_coordinate_t& jointCoordinate,
		std::array<Eigen::Vector3d,4>& feetPositions)
{
	DerivedClassType::FeetPositionsBaseFrame(jointCoordinate, feetPositions);
}

template< class DerivedClassType, size_t BASE_COORD_SIZE, size_t JOINT_COORD_SIZE >
void ModelKinematicsBase<DerivedClassType, BASE_COORD_SIZE, JOINT_COORD_SIZE>::footPositionOriginFrame(const size_t& footIndex, Eigen::Vector3d& footPosition)
{
	// calculate foot position in Base frame
	Eigen::Vector3d b_footPosition;
	footPositionBaseFrame(footIndex, b_footPosition);
	// calculate foot position in Origin frame
	footPosition = b_R_o_.transpose()*b_footPosition + qBase_.tail<3>();
}

template< class DerivedClassType, size_t BASE_COORD_SIZE, size_t JOINT_COORD_SIZE >
void ModelKinematicsBase<DerivedClassType, BASE_COORD_SIZE, JOINT_COORD_SIZE>::footPositionOriginFrame(const generalized_coordinate_t& generalizedCoordinate,
		const size_t& footIndex, Eigen::Vector3d& footPosition)
{
	// update the class
	update(generalizedCoordinate);
	// calculate foot position in Origin frame
	footPositionOriginFrame(footIndex, footPosition);
}

template< class DerivedClassType, size_t BASE_COORD_SIZE, size_t JOINT_COORD_SIZE >
void ModelKinematicsBase<DerivedClassType, BASE_COORD_SIZE, JOINT_COORD_SIZE>::feetPositionsOriginFrame(std::array<Eigen::Vector3d,4>& feetPositions)
{
	for (size_t i=0; i<4; i++)
		footPositionOriginFrame(i, feetPositions[i]);
}

template< class DerivedClassType, size_t BASE_COORD_SIZE, size_t JOINT_COORD_SIZE >
void ModelKinematicsBase<DerivedClassType, BASE_COORD_SIZE, JOINT_COORD_SIZE>::feetPositionsOriginFrame(const generalized_coordinate_t& generalizedCoordinate,
			std::array<Eigen::Vector3d,4>& feetPositions)
{
	// update the class
		update(generalizedCoordinate);
		// calculate feet's positions in Origin frame
		feetPositionsOriginFrame(feetPositions);
}

template< class DerivedClassType, size_t BASE_COORD_SIZE, size_t JOINT_COORD_SIZE >
void ModelKinematicsBase<DerivedClassType, BASE_COORD_SIZE, JOINT_COORD_SIZE>::footJacobainBaseFrame(const size_t& footIndex, Eigen::Matrix<double,6,JOINT_COORDINATE_SIZE>& footJacobain)
{
	FootJacobainBaseFrame(qJoint_, footIndex, footJacobain);
}

template< class DerivedClassType, size_t BASE_COORD_SIZE, size_t JOINT_COORD_SIZE >
static void ModelKinematicsBase<DerivedClassType, BASE_COORD_SIZE, JOINT_COORD_SIZE>::FootJacobainBaseFrame(const joint_coordinate_t& jointCoordinate,
		const size_t& footIndex, Eigen::Matrix<double,6,JOINT_COORDINATE_SIZE>& footJacobain)
{
	DerivedClassType::FootJacobainBaseFrame(jointCoordinate, footIndex, footJacobain);
}

template< class DerivedClassType, size_t BASE_COORD_SIZE, size_t JOINT_COORD_SIZE >
static void ModelKinematicsBase<DerivedClassType, BASE_COORD_SIZE, JOINT_COORD_SIZE>::FromBaseJacobianToInertiaJacobian(
		const Eigen::Matrix3d& i_R_b,
		const Eigen::Vector3d& b_r_point,
		const Eigen::Matrix<double,6,JOINT_COORDINATE_SIZE>& b_J_point,
		Eigen::Matrix<double,6,GENERALIZED_COORDINATE_SIZE>& i_J_point)
{
	// rotation
	i_J_point.topRows<3>() << i_R_b,  Eigen::Matrix3d::Zero(),  i_R_b*b_J_point.topRows<3>();
	// translation
	i_J_point.bottomRows<3>() << -i_R_b*CrossProductMatrix(b_r_point), i_R_b, i_R_b*b_J_point.bottomRows<3>();
}

template< class DerivedClassType, size_t BASE_COORD_SIZE, size_t JOINT_COORD_SIZE >
Eigen::Matrix3d ModelKinematicsBase<DerivedClassType, BASE_COORD_SIZE, JOINT_COORD_SIZE>::rotationMatrixOrigintoBase() const  {

	return b_R_o_;
}

template< class DerivedClassType, size_t BASE_COORD_SIZE, size_t JOINT_COORD_SIZE >
static void ModelKinematicsBase<DerivedClassType, BASE_COORD_SIZE, JOINT_COORD_SIZE>::ComPositionBaseFrame(const joint_coordinate_t& jointCoordinate, Eigen::Vector3d& comPosition)
{
	DerivedClassType::ComPositionBaseFrame(jointCoordinate, comPosition);
}
