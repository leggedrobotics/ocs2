/*
 * EndEffectorConstraintBase.h
 *
 *  Created on: Jul 20, 2016
 *      Author: farbod
 */

#ifndef ENDEFFECTORCONSTRAINTBASE_H_
#define ENDEFFECTORCONSTRAINTBASE_H_

#include <memory>
#include <limits>
#include <cmath>
#include <Eigen/Dense>

namespace switched_model {

class EndEffectorConstraintBase
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::shared_ptr<EndEffectorConstraintBase>		 Ptr;
	typedef std::shared_ptr<const EndEffectorConstraintBase> ConstPtr;

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

	/**
	 * An approximate constraint indicator function. The cube should inscribed the actual constrained area.
	 */
	virtual bool isActive(const Eigen::Vector3d& vector) const;

	void normalizeVector(const Eigen::Vector3d& vector,
			Eigen::Vector3d& normalizedVector, Eigen::Vector3d& scale) const;

	bool isRepeller() const { return isRepeller_; }

	void setXLimits(const Eigen::Vector2d& xLimits);
	void setYLimits(const Eigen::Vector2d& yLimits);
	void setZLimits(const Eigen::Vector2d& zLimits);

	const Eigen::Vector2d& getXLimits() const;
	const Eigen::Vector2d& getYLimits() const;
	const Eigen::Vector2d& getZLimits() const;

	virtual double constraintValue(const Eigen::Vector3d& vector) const = 0;
	virtual Eigen::Vector3d constraintDerivative(const Eigen::Vector3d& vector) const = 0;

	virtual EndEffectorConstraintBase* clone() const = 0;

protected:
	const double localInf_ = 1e9;

	bool isRepeller_;

	Eigen::Vector2d xLimits_;
	Eigen::Vector2d yLimits_;
	Eigen::Vector2d zLimits_;

};


}  // end of switched_model namespace

#endif /* ENDEFFECTORCONSTRAINTBASE_H_ */
