/*
 * CPG_BASE.h
 *
 *  Created on: Jul 5, 2016
 *      Author: farbod
 */

#ifndef CPG_BASE_H_
#define CPG_BASE_H_

#include <cmath>
#include <limits>
#include <memory>

namespace switched_model {

template <typename scalar_t = double>
class CPG_BASE
{
public:
	typedef std::shared_ptr<CPG_BASE<scalar_t>> 		Ptr;
	typedef std::shared_ptr<const CPG_BASE<scalar_t>> 	ConstPtr;

	CPG_BASE()
	: CPG_BASE(0.15, 1.0)
	{}

	CPG_BASE(const scalar_t& swingLegLiftOff, const scalar_t& swingTimeScale = 1.0)
	: swingLegLiftOff_(swingLegLiftOff),
	  swingTimeScale_(swingTimeScale)
	{}

	virtual ~CPG_BASE() = default;

	virtual void setConstant() = 0;

	virtual void set(const scalar_t& startTime, const scalar_t& finalTime, const scalar_t& maxHight) {
		startTime_ = startTime;
		finalTime_ = finalTime;
		maxHight_ = maxHight;
	}

	virtual scalar_t adaptiveSwingLegLiftOff(const scalar_t& startTime, const scalar_t& finalTime) {
		return swingLegLiftOff_* std::min(1.0, (finalTime-startTime)/swingTimeScale_);
//		return swingLegLiftOff_;
	}

	// calculate the position of the CPG output
	virtual scalar_t calculatePosition(const scalar_t& time) const = 0;

	// calculate the velocity of the CPG output
	virtual scalar_t calculateVelocity(const scalar_t& time) const {

		const scalar_t h = sqrt(std::numeric_limits<scalar_t>::epsilon());

		scalar_t positionPlus  = calculatePosition(time+h/2.0);
		scalar_t positionMinus = calculatePosition(time-h/2.0);

		return (positionPlus-positionMinus)/h;
	}

	// calculate the velocity of the CPG output
	virtual scalar_t calculateAcceleration(const scalar_t& time) const {

		const scalar_t h = sqrt(std::numeric_limits<scalar_t>::epsilon());

		scalar_t velocityPlus  = calculateVelocity(time+h/2.0);
		scalar_t velocityMinus = calculateVelocity(time-h/2.0);

		return (velocityPlus-velocityMinus)/h;
	}


protected:
	scalar_t startTime_;
	scalar_t finalTime_;
	scalar_t maxHight_;

	scalar_t swingLegLiftOff_;
	scalar_t swingTimeScale_;
};


}  // end of switched_model namespace

#endif /* CPG_BASE_H_ */
