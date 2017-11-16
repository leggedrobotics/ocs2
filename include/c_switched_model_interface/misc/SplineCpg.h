/*
 * SplineCpg.h
 *
 *  Created on: Apr 30, 2017
 *      Author: farbod
 */

#ifndef SPLINECPG_H_
#define SPLINECPG_H_

#include <cmath>

#include "CpgBase.h"
#include "CubicSpline.h"

namespace switched_model {

class SplineCpg : public CpgBase
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::shared_ptr<SplineCpg> Ptr;

	SplineCpg(const double& swingLegLiftOff = 0.15, const double& swingTimeScale = 1.0)
	: CpgBase(swingLegLiftOff, swingTimeScale)
	{}

	~SplineCpg() {}

	void set(const double& startTime, const double& finalTime, const double& maxHight) override {
		CpgBase::set(startTime, finalTime, maxHight);

		midTime_  = 0.5*(startTime_+finalTime_);

		leftSpline_.set(startTime_, startHight_ /*p0*/, 0.0 /*v0*/, midTime_ /*t1*/, maxHight_ /*p1*/, 0.0 /*v1*/);
		rightSpline_.set(midTime_,  maxHight_ /*p0*/, 0.0 /*v0*/, finalTime_ /*t1*/, finalHight_ /*p1*/, 0.0 /*v1*/);
	}

	double calculatePosition(const double& time) override {
		if (time <= midTime_)
			return leftSpline_.evaluateSplinePosition(time);
		else
			return rightSpline_.evaluateSplinePosition(time);
	}

	double calculateVelocity(const double& time) override {
		if (time <= midTime_)
			return leftSpline_.evaluateSplineVelocity(time);
		else
			return rightSpline_.evaluateSplineVelocity(time);
	}

	double calculateAcceleration(const double& time) override {
		if (time <= midTime_)
			return leftSpline_.evaluateSplineAcceleration(time);
		else
			return rightSpline_.evaluateSplineAcceleration(time);
	}

private:
	double midTime_ = 0.0;

	const double startHight_ = 0.0;
	const double finalHight_ = 0.0;

	CubicSpline leftSpline_;
	CubicSpline rightSpline_;
};


}  // end of switched_model namespace

#endif /* SPLINECPG_H_ */
