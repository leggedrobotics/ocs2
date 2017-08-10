/*
 * ModelKinematicsBase.h
 *
 *  Created on: Aug 3, 2017
 *      Author: sasutosh
 */

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template< class DerivedClassType, size_t BASE_COORD_SIZE, size_t JOINT_COORD_SIZE >
void ModelKinematicsBase<DerivedClassType, BASE_COORD_SIZE, JOINT_COORD_SIZE>::update(const generalized_coordinate_t& generalizedCoordinate)
{
	qBase_  = generalizedCoordinate.head<BASE_COORDINATE_SIZE>();
	qJoint_ = generalizedCoordinate.tail<JOINT_COORDINATE_SIZE>();
	b_R_o_  = RotationMatrixOrigintoBase(qBase_.head<3>());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template< class DerivedClassType, size_t BASE_COORD_SIZE, size_t JOINT_COORD_SIZE >
void ModelKinematicsBase<DerivedClassType, BASE_COORD_SIZE, JOINT_COORD_SIZE>::footPositionBaseFrame(const size_t& footIndex, Eigen::Vector3d& footPosition)
{
	FootPositionBaseFrame(qJoint_, footIndex, footPosition);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template< class DerivedClassType, size_t BASE_COORD_SIZE, size_t JOINT_COORD_SIZE >
void ModelKinematicsBase<DerivedClassType, BASE_COORD_SIZE, JOINT_COORD_SIZE>::feetPositionsBaseFrame(std::array<Eigen::Vector3d,4>& feetPositions)
{
	FeetPositionsBaseFrame(qJoint_, feetPositions);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template< class DerivedClassType, size_t BASE_COORD_SIZE, size_t JOINT_COORD_SIZE >
static void ModelKinematicsBase<DerivedClassType, BASE_COORD_SIZE, JOINT_COORD_SIZE>::FootPositionBaseFrame(const joint_coordinate_t& jointCoordinate,
		const size_t& footIndex, Eigen::Vector3d& footPosition)
{
	DerivedClassType::FootPositionBaseFrame(jointCoordinate, footIndex, footPosition);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template< class DerivedClassType, size_t BASE_COORD_SIZE, size_t JOINT_COORD_SIZE >
static void ModelKinematicsBase<DerivedClassType, BASE_COORD_SIZE, JOINT_COORD_SIZE>::FeetPositionsBaseFrame(const joint_coordinate_t& jointCoordinate,
		std::array<Eigen::Vector3d,4>& feetPositions)
{
	for (size_t i=0; i<4; i++)
		FootPositionBaseFrame(jointCoordinate, i, feetPositions[i]);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template< class DerivedClassType, size_t BASE_COORD_SIZE, size_t JOINT_COORD_SIZE >
void ModelKinematicsBase<DerivedClassType, BASE_COORD_SIZE, JOINT_COORD_SIZE>::footPositionOriginFrame(const size_t& footIndex, Eigen::Vector3d& footPosition)
{
	// calculate foot position in Base frame
	Eigen::Vector3d b_footPosition;
	footPositionBaseFrame(footIndex, b_footPosition);
	// calculate foot position in Origin frame
	footPosition = b_R_o_.transpose()*b_footPosition + qBase_.tail<3>();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template< class DerivedClassType, size_t BASE_COORD_SIZE, size_t JOINT_COORD_SIZE >
void ModelKinematicsBase<DerivedClassType, BASE_COORD_SIZE, JOINT_COORD_SIZE>::footPositionOriginFrame(const generalized_coordinate_t& generalizedCoordinate,
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
template< class DerivedClassType, size_t BASE_COORD_SIZE, size_t JOINT_COORD_SIZE >
void ModelKinematicsBase<DerivedClassType, BASE_COORD_SIZE, JOINT_COORD_SIZE>::feetPositionsOriginFrame(std::array<Eigen::Vector3d,4>& feetPositions)
{
	for (size_t i=0; i<4; i++)
		footPositionOriginFrame(i, feetPositions[i]);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template< class DerivedClassType, size_t BASE_COORD_SIZE, size_t JOINT_COORD_SIZE >
void ModelKinematicsBase<DerivedClassType, BASE_COORD_SIZE, JOINT_COORD_SIZE>::feetPositionsOriginFrame(const generalized_coordinate_t& generalizedCoordinate,
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
template< class DerivedClassType, size_t BASE_COORD_SIZE, size_t JOINT_COORD_SIZE >
void ModelKinematicsBase<DerivedClassType, BASE_COORD_SIZE, JOINT_COORD_SIZE>::footJacobainBaseFrame(const size_t& footIndex, Eigen::Matrix<double,6,JOINT_COORDINATE_SIZE>& footJacobain)
{
	FootJacobainBaseFrame(qJoint_, footIndex, footJacobain);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template< class DerivedClassType, size_t BASE_COORD_SIZE, size_t JOINT_COORD_SIZE >
static void ModelKinematicsBase<DerivedClassType, BASE_COORD_SIZE, JOINT_COORD_SIZE>::FootJacobainBaseFrame(const joint_coordinate_t& jointCoordinate,
		const size_t& footIndex, Eigen::Matrix<double,6,JOINT_COORDINATE_SIZE>& footJacobain)
{
	DerivedClassType::FootJacobainBaseFrame(jointCoordinate, footIndex, footJacobain);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
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

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template< class DerivedClassType, size_t BASE_COORD_SIZE, size_t JOINT_COORD_SIZE >
Eigen::Matrix3d ModelKinematicsBase<DerivedClassType, BASE_COORD_SIZE, JOINT_COORD_SIZE>::rotationMatrixOrigintoBase() const  {

	return b_R_o_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template< class DerivedClassType, size_t BASE_COORD_SIZE, size_t JOINT_COORD_SIZE >
static void ModelKinematicsBase<DerivedClassType, BASE_COORD_SIZE, JOINT_COORD_SIZE>::ComPositionBaseFrame(const joint_coordinate_t& jointCoordinate, Eigen::Vector3d& comPosition)
{
	DerivedClassType::ComPositionBaseFrame(jointCoordinate, comPosition);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template< class DerivedClassType, size_t BASE_COORD_SIZE, size_t JOINT_COORD_SIZE >
template <typename Derived>
Eigen::Matrix3d ModelKinematicsBase<DerivedClassType, BASE_COORD_SIZE, JOINT_COORD_SIZE>::RotationMatrixOrigintoBase(const Eigen::DenseBase<Derived>& eulerAngles)
{

	if (eulerAngles.innerSize()!=3 || eulerAngles.outerSize()!=1)  throw std::runtime_error("Input argument should be a 3-by-1 vector.");

	// inputs are the intrinsic rotation angles in RADIANTS
	double sinAlpha = sin(eulerAngles(0));
	double cosAlpha = cos(eulerAngles(0));
	double sinBeta  = sin(eulerAngles(1));
	double cosBeta  = cos(eulerAngles(1));
	double sinGamma = sin(eulerAngles(2));
	double cosGamma = cos(eulerAngles(2));

	Eigen::Matrix3d Rx, Ry, Rz;
	Rx << 1, 0, 0,					0, cosAlpha, sinAlpha,		0, -sinAlpha, cosAlpha;
	Ry << cosBeta, 0, -sinBeta,		0, 1, 0,		 			sinBeta, 0, cosBeta;
	Rz << cosGamma, sinGamma, 0, 	-sinGamma, cosGamma, 0,		0, 0, 1;

	return Rz*Ry*Rx;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template< class DerivedClassType, size_t BASE_COORD_SIZE, size_t JOINT_COORD_SIZE >
template <typename Derived>
Eigen::Matrix3d ModelKinematicsBase<DerivedClassType, BASE_COORD_SIZE, JOINT_COORD_SIZE>::RotationMatrixBasetoOrigin(const Eigen::DenseBase<Derived>& eulerAngles)
{

	if (eulerAngles.innerSize()!=3 || eulerAngles.outerSize()!=1)  throw std::runtime_error("Input argument should be a 3-by-1 vector.");

	// inputs are the intrinsic rotation angles in RADIANTS
	double sinAlpha = sin(eulerAngles(0));
	double cosAlpha = cos(eulerAngles(0));
	double sinBeta  = sin(eulerAngles(1));
	double cosBeta  = cos(eulerAngles(1));
	double sinGamma = sin(eulerAngles(2));
	double cosGamma = cos(eulerAngles(2));

	Eigen::Matrix3d RxT, RyT, RzT;
	RxT << 1, 0, 0,					0, cosAlpha, -sinAlpha,		0, sinAlpha, cosAlpha;
	RyT << cosBeta, 0, sinBeta,		0, 1, 0,		 			-sinBeta, 0, cosBeta;
	RzT << cosGamma, -sinGamma, 0, 	sinGamma, cosGamma, 0,		0, 0, 1;

	return RxT*RyT*RzT;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template< class DerivedClassType, size_t BASE_COORD_SIZE, size_t JOINT_COORD_SIZE >
template <typename Derived>
Eigen::Matrix3d ModelKinematicsBase<DerivedClassType, BASE_COORD_SIZE, JOINT_COORD_SIZE>::CrossProductMatrix(const Eigen::DenseBase<Derived>& in)
{

	if (in.innerSize()!=3 || in.outerSize()!=1)  throw std::runtime_error("Input argument should be a 3-by-1 vector.");

	Eigen::Matrix3d out;
	out <<   0.0,   -in(2), +in(1),
			+in(2),  0.0,   -in(0),
			-in(1), +in(0),  0.0;
	return out;
}
