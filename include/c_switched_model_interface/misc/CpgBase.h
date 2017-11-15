/*
 * CpgBase.h
 *
 *  Created on: Jul 5, 2016
 *      Author: farbod
 */

#ifndef HYQ_CPGBASE_H_
#define HYQ_CPGBASE_H_

#include <cmath>
#include <limits>
#include <memory>

namespace switched_model {

class CpgBase
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::shared_ptr<CpgBase> Ptr;
	typedef std::array<Ptr,4>        PtrArray;

	CpgBase(const double& swingLegLiftOff = 0.15, const double& swingTimeScale = 1.0)
	: swingLegLiftOff_(swingLegLiftOff),
	  swingTimeScale_(swingTimeScale)
	{}

	virtual ~CpgBase() {}

	virtual void set(const double& startTime, const double& finalTime, const double& maxHight) {
		startTime_ = startTime;
		finalTime_ = finalTime;
		maxHight_ = maxHight;
	}

	virtual double adaptiveSwingLegLiftOff(const double& startTime, const double& finalTime) {
		return swingLegLiftOff_* std::min(1.0, (finalTime-startTime)/swingTimeScale_);
//		return swingLegLiftOff_;
	}

	// calculate the position of the CPG output
	virtual double calculatePosition(const double& time) = 0;

	// calculate the velocity of the CPG output
	virtual double calculateVelocity(const double& time) {

		const double h = sqrt(std::numeric_limits<double>::epsilon());

		double positionPlus  = calculatePosition(time+h/2.0);
		double positionMinus = calculatePosition(time-h/2.0);

		return (positionPlus-positionMinus)/h;
	}

	// calculate the velocity of the CPG output
	virtual double calculateAcceleration(const double& time) {

		const double h = sqrt(std::numeric_limits<double>::epsilon());

		double velocityPlus  = calculateVelocity(time+h/2.0);
		double velocityMinus = calculateVelocity(time-h/2.0);

		return (velocityPlus-velocityMinus)/h;
	}


protected:
	double startTime_;
	double finalTime_;
	double maxHight_;

	double swingLegLiftOff_;
	double swingTimeScale_;
};


}  // end of switched_model namespace

#endif /* HYQ_CPGBASE_H_ */
