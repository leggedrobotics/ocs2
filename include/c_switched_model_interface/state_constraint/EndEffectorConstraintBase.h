/*
 * EndEffectorConstraintBase.h
 *
 *  Created on: Jul 20, 2016
 *      Author: farbod
 */

#ifndef HYQ_ENDEFFECTORCONSTRAINTBASE_H_
#define HYQ_ENDEFFECTORCONSTRAINTBASE_H_

#include <memory>
#include <limits>
#include <cmath>
#include <Eigen/Dense>

namespace hyq {

class EndEffectorConstraintBase
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::shared_ptr<EndEffectorConstraintBase> Ptr;
	static constexpr double plusInf_  = +1e10;
	static constexpr double minusInf_ = -1e10;

	EndEffectorConstraintBase(const bool& isRepeller = true,
			const Eigen::Vector2d& xLimits = Eigen::Vector2d(minusInf_,plusInf_),
			const Eigen::Vector2d& yLimits = Eigen::Vector2d(minusInf_,plusInf_),
			const Eigen::Vector2d& zLimits = Eigen::Vector2d(minusInf_,plusInf_))
	: isRepeller_(isRepeller),
	  xLimits_(xLimits),
	  yLimits_(yLimits),
	  zLimits_(zLimits)
	{}

	virtual ~EndEffectorConstraintBase() {}

	// an approximate constraint indicator. The cube should inscribed the actual constrained area.
	virtual bool isActive(const Eigen::Vector3d& vector);

	void normalizeVector(const Eigen::Vector3d& vector,
			Eigen::Vector3d& normalizedVector, Eigen::Vector3d& scale);

	bool isRepeller() const { return isRepeller_; }

	void setXLimits(const Eigen::Vector2d& xLimits) { xLimits_ = xLimits; }
	void setYLimits(const Eigen::Vector2d& yLimits) { yLimits_ = yLimits; }
	void setZLimits(const Eigen::Vector2d& zLimits) { zLimits_ = zLimits; }

	const Eigen::Vector2d& getXLimits() const { return xLimits_; }
	const Eigen::Vector2d& getYLimits() const { return yLimits_; }
	const Eigen::Vector2d& getZLimits() const { return zLimits_; }

	virtual double constraintValue(const Eigen::Vector3d& vector) = 0;
	virtual Eigen::Vector3d constraintDerivative(const Eigen::Vector3d& vector) = 0;
	virtual Ptr clone() const = 0;

protected:
	const double localInf_ = 1e9;

	bool isRepeller_;

	Eigen::Vector2d xLimits_;
	Eigen::Vector2d yLimits_;
	Eigen::Vector2d zLimits_;

};


}  // end of hyq namespace

#endif /* HYQ_ENDEFFECTORCONSTRAINTBASE_H_ */
