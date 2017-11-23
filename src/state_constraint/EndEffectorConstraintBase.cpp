/*
 * EndEffectorConstraintBase.cpp
 *
 *  Created on: Sep 12, 2016
 *      Author: farbod
 */

#include "c_switched_model_interface/state_constraint/EndEffectorConstraintBase.h"

namespace switched_model {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
constexpr double EndEffectorConstraintBase::plusInf_;
constexpr double EndEffectorConstraintBase::minusInf_;

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool EndEffectorConstraintBase::isActive(const Eigen::Vector3d& vector) {
	bool isInXLimits, isInYLimits, isInZLimits;
	isInXLimits = (xLimits_(0)<=vector(0) && vector(0)<=xLimits_(1)) ? true : false;
	isInYLimits = (yLimits_(0)<=vector(1) && vector(1)<=yLimits_(1)) ? true : false;
	isInZLimits = (zLimits_(0)<=vector(2) && vector(2)<=zLimits_(1)) ? true : false;

	bool isInsideRectangle = isInXLimits && isInYLimits && isInZLimits;

	if (isRepeller_==true)
		return isInsideRectangle;
	else
		return !isInsideRectangle;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void EndEffectorConstraintBase::normalizeVector(const Eigen::Vector3d& vector,
		Eigen::Vector3d& normalizedVector, Eigen::Vector3d& scale) {

	// x direction
	if (-localInf_<xLimits_(0) && xLimits_(1)<localInf_) {
		double center = (xLimits_(1)+xLimits_(0)) / 2.0;
		scale(0) = (xLimits_(1)-xLimits_(0)) / 2.0;
		normalizedVector(0) = (vector(0)-center)/scale(0);
	} else {
		scale(0)  = 1.0;
		normalizedVector(0) = 0.0;
	}

	// y direction
	if (-localInf_<yLimits_(0) && yLimits_(1)<localInf_) {
		double center = (yLimits_(1)+yLimits_(0)) / 2.0;
		scale(1) = (yLimits_(1)-yLimits_(0)) / 2.0;
		normalizedVector(1) = (vector(1)-center)/scale(1);
	} else {
		scale(1)  = 1.0;
		normalizedVector(1) = 0.0;
	}

	// z direction
	if (-localInf_<zLimits_(0) && zLimits_(1)<localInf_) {
		double center = (zLimits_(1)+zLimits_(0)) / 2.0;
		scale(2) = (zLimits_(1)-zLimits_(0)) / 2.0;
		normalizedVector(2) = (vector(2)-center)/scale(2);
	} else {
		scale(2)  = 1.0;
		normalizedVector(2) = 0.0;
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void EndEffectorConstraintBase::setXLimits(const Eigen::Vector2d& xLimits) {
	xLimits_ = xLimits;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void EndEffectorConstraintBase::setYLimits(const Eigen::Vector2d& yLimits) {
	yLimits_ = yLimits;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void EndEffectorConstraintBase::setZLimits(const Eigen::Vector2d& zLimits) {
	zLimits_ = zLimits;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
const Eigen::Vector2d& EndEffectorConstraintBase::getXLimits() const {
	return xLimits_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
const Eigen::Vector2d& EndEffectorConstraintBase::getYLimits() const {
	return yLimits_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
const Eigen::Vector2d& EndEffectorConstraintBase::getZLimits() const {
	return zLimits_;
}

}  // end of switched_model namespace

