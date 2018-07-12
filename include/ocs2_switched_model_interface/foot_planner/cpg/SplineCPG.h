/*
 * SplineCPG.h
 *
 *  Created on: Apr 30, 2017
 *      Author: farbod
 */

#ifndef SPLINECPG_H_
#define SPLINECPG_H_

#include <cmath>

#include "ocs2_switched_model_interface/foot_planner/cpg/CPG_BASE.h"
#include "ocs2_switched_model_interface/misc/CubicSpline.h"

namespace switched_model {

template <typename scalar_t = double>
class SplineCPG : public CPG_BASE<scalar_t>
{
public:
	typedef std::shared_ptr<SplineCPG<scalar_t>> Ptr;

	typedef CPG_BASE<scalar_t> Base;

	SplineCPG()
	: Base(0.15, 1.0)
	{}

	SplineCPG(const scalar_t& swingLegLiftOff, const scalar_t& swingTimeScale = 1.0)
	: Base(swingLegLiftOff, swingTimeScale)
	{}

	~SplineCPG() = default;

	void setConstant() override {

		Base::set(0.0, 1.0, 0.0);
		leftSpline_.setConstant(startHight_);
		rightSpline_.setConstant(startHight_);
	}

	void set(const scalar_t& startTime, const scalar_t& finalTime, const scalar_t& maxHight) override {

		Base::set(startTime, finalTime, maxHight);

		midTime_ = 0.5*(Base::startTime_+Base::finalTime_);

		leftSpline_.set(Base::startTime_, startHight_ /*p0*/, 0.0 /*v0*/, midTime_ /*t1*/, Base::maxHight_ /*p1*/, 0.0 /*v1*/);
		rightSpline_.set(midTime_, Base::maxHight_ /*p0*/, 0.0 /*v0*/, Base::finalTime_ /*t1*/, finalHight_ /*p1*/, 0.0 /*v1*/);
	}

	scalar_t calculatePosition(const scalar_t& time) const override {
		if (time <= midTime_)
			return leftSpline_.evaluateSplinePosition(time);
		else
			return rightSpline_.evaluateSplinePosition(time);
	}

	scalar_t calculateVelocity(const scalar_t& time) const override {
		if (time <= midTime_)
			return leftSpline_.evaluateSplineVelocity(time);
		else
			return rightSpline_.evaluateSplineVelocity(time);
	}

	scalar_t calculateAcceleration(const scalar_t& time) const override {
		if (time <= midTime_)
			return leftSpline_.evaluateSplineAcceleration(time);
		else
			return rightSpline_.evaluateSplineAcceleration(time);
	}

private:
	scalar_t midTime_ = 0.0;

	const scalar_t startHight_ = 0.0;
	const scalar_t finalHight_ = 0.0;

	CubicSpline<scalar_t> leftSpline_;
	CubicSpline<scalar_t> rightSpline_;
};


}  // end of switched_model namespace

#endif /* SPLINECPG_H_ */
