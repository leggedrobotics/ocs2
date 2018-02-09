/*
 * PotentialFieldConstraint.h
 *
 *  Created on: Aug 10, 2016
 *      Author: farbod
 */

#ifndef POTENTIALFIELDCONSTRAINT_H_
#define POTENTIALFIELDCONSTRAINT_H_

#include "c_switched_model_interface/state_constraint/EndEffectorConstraintBase.h"

namespace switched_model {

class PotentialFieldConstraint : public EndEffectorConstraintBase
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::shared_ptr<EllipticalConstraint> 		Ptr;
	typedef std::shared_ptr<const EllipticalConstraint> ConstPtr;

	PotentialFieldConstraint(const bool& isRepeller = true,
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

	virtual ~PotentialFieldConstraint() {}


	virtual double constraintValue(const Eigen::Vector3d& vector) const override {

		Eigen::Vector3d normalizedVector, scale;
		normalizeVector(vector, normalizedVector, scale);

		double d2 = normalizedVector.squaredNorm();

		if (d2>1.0 && isRepeller_==true)   return 0.0;
		if (d2<1.0 && isRepeller_==false)  return 0.0;

		if (isRepeller_==true)
			return 1.0/sqrt(d2);
		else
			return 1.0-1.0/sqrt(d2);
	}



	virtual Eigen::Vector3d constraintDerivative(const Eigen::Vector3d& vector) const override {

		Eigen::Vector3d normalizedVector, scale;
		normalizeVector(vector, normalizedVector, scale);

		double d2 = normalizedVector.squaredNorm();

		if (d2>1.0 && isRepeller_==true)   return Eigen::Vector3d::Zero();
		if (d2<1.0 && isRepeller_==false)  return Eigen::Vector3d::Zero();

		if (isRepeller_==true)
			return -normalizedVector.cwiseQuotient(scale) / (d2*sqrt(d2));
		else
			return  normalizedVector.cwiseQuotient(scale) / (d2*sqrt(d2));
	}

	virtual PotentialFieldConstraint* clone() const override {
		return PotentialFieldConstraint(*this);
	}

private:

};

}  // end of switched_model namespace



#endif /* POTENTIALFIELDCONSTRAINT_H_ */
