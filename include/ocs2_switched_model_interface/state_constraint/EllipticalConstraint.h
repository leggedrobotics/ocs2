/*
 * EllipticalConstraint.h
 *
 *  Created on: Jul 20, 2016
 *      Author: farbod
 */

#ifndef ELLIPTICALCONSTRAINT_H_
#define ELLIPTICALCONSTRAINT_H_

#include "ocs2_switched_model_interface/state_constraint/EndEffectorConstraintBase.h"

namespace switched_model {

class EllipticalConstraint : public EndEffectorConstraintBase
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::shared_ptr<EllipticalConstraint> 		Ptr;
	typedef std::shared_ptr<const EllipticalConstraint> ConstPtr;

	EllipticalConstraint(const bool& isRepeller = true,
			const Eigen::Vector2d& xLimits = Eigen::Vector2d(minusInf_,plusInf_),
			const Eigen::Vector2d& yLimits = Eigen::Vector2d(minusInf_,plusInf_),
			const Eigen::Vector2d& zLimits = Eigen::Vector2d(minusInf_,plusInf_))

	: EndEffectorConstraintBase(isRepeller, xLimits, yLimits, zLimits)
	{
		// find a  rectangle that is inscribed in the ellipse
		if (isRepeller==false) {
			double sqrt8 = sqrt(8);
			double a = 0.5+1/sqrt8;
			double b = 0.5-1/sqrt8;
			Eigen::Matrix2d T;
			T << a, b,
				b, a;

			setXLimits(T*xLimits);
			setYLimits(T*yLimits);
			setZLimits(T*zLimits);
		}
	}

	virtual ~EllipticalConstraint() {}


	virtual double constraintValue(const Eigen::Vector3d& vector) const override {

		Eigen::Vector3d normalizedVector, scale;
		normalizeVector(vector, normalizedVector, scale);

		double g2;
		if (isRepeller_==true)
			g2 = 1.0 - normalizedVector.squaredNorm();
		else
			g2 = normalizedVector.squaredNorm() - 1.0;

		if (g2>0)
			return sqrt(g2);
		else
			return 0.0;
	}



	virtual Eigen::Vector3d constraintDerivative(const Eigen::Vector3d& vector) const override {

		Eigen::Vector3d normalizedVector, scale;
		normalizeVector(vector, normalizedVector, scale);

		double g2;
		if (isRepeller_==true)
			g2 = 1.0 - normalizedVector.squaredNorm();
		else
			g2 = normalizedVector.squaredNorm() - 1.0;

		// the constraint is not active
		if (g2<0)
			return Eigen::Vector3d::Zero();

		Eigen::Vector3d gradient;
		if (isRepeller_==true)
			gradient = -normalizedVector.cwiseQuotient(scale) / sqrt(g2);
		else
			gradient = normalizedVector.cwiseQuotient(scale) / sqrt(g2);

		return gradient;
	}

	virtual EllipticalConstraint* clone() const override {
		return new EllipticalConstraint(*this);
	}

private:

};

}  // end of switched_model namespace



#endif /* ELLIPTICALCONSTRAINT_H_ */
