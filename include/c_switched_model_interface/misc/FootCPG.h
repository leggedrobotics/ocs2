/*
 * FootCPG.h
 *
 *  Created on: Jul 10, 2017
 *      Author: farbod
 */

#ifndef HYQ_FOOTCPG_H_
#define HYQ_FOOTCPG_H_

#include <cmath>

#include "misc/CubicSpline.h"
#include "misc/SplineCpg.h"

namespace hyq {

class FootCPG
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::shared_ptr<FootCPG> Ptr;
	typedef std::array<Ptr , 4>      PtrArray;

	FootCPG(const double& swingLegLiftOff = 0.15, const double& swingTimeScale = 1.0)
	{
		xSplinePtr_ = CubicSpline::Ptr( new CubicSpline() );
		ySplinePtr_ = CubicSpline::Ptr( new CubicSpline() );
		zDoubleSplinePtr_ = SplineCpg::Ptr( new SplineCpg(swingLegLiftOff, swingTimeScale) );
	}

	~FootCPG() {}

	void set(const double& currentTime,
			const double& liftoffTime,  const double& touchdownTime,
			const Eigen::Vector2d& currentPosXY, const Eigen::Vector2d& currentVelXY,
			const Eigen::Vector2d& touchdownPosXY,
			const double& swingLegLiftOff) {
		xSplinePtr_->set(currentTime,  currentPosXY(0) /*p0*/, currentVelXY(0) /*v0*/, touchdownTime /*t1*/, touchdownPosXY(0) /*p1*/, 0.0 /*v1*/);
		ySplinePtr_->set(currentTime,  currentPosXY(1) /*p0*/, currentVelXY(1) /*v0*/, touchdownTime /*t1*/, touchdownPosXY(1) /*p1*/, 0.0 /*v1*/);
		zDoubleSplinePtr_->set(liftoffTime, touchdownTime, swingLegLiftOff);
	}

	double adaptiveSwingLegLiftOff(const double& startTime, const double& finalTime) {
		return zDoubleSplinePtr_->adaptiveSwingLegLiftOff(startTime, finalTime);
	}

	/**
	 *
	 * @param time
	 * @return
	 */
	Eigen::Vector3d calculatePosition(const double& time) {

		Eigen::Vector3d p;
		p(0) = xSplinePtr_->evaluateSplinePosition(time);
		p(1) = ySplinePtr_->evaluateSplinePosition(time);
		p(2) = zDoubleSplinePtr_->calculatePosition(time);
		return p;
	}

	/**
	 *
	 * @param time
	 * @return
	 */
	Eigen::Vector3d calculateVelocity(const double& time) {

		Eigen::Vector3d v;
		v(0) = xSplinePtr_->evaluateSplineVelocity(time);
		v(1) = ySplinePtr_->evaluateSplineVelocity(time);
		v(2) = zDoubleSplinePtr_->calculateVelocity(time);
		return v;
	}

	/**
	 *
	 * @param time
	 * @return
	 */
	Eigen::Vector3d calculateAcceleration(const double& time) {

		Eigen::Vector3d a;
		a(0) = xSplinePtr_->evaluateSplineAcceleration(time);
		a(1) = ySplinePtr_->evaluateSplineAcceleration(time);
		a(2) = zDoubleSplinePtr_->calculateAcceleration(time);
		return a;
	}

private:
	CubicSpline::Ptr xSplinePtr_;
	CubicSpline::Ptr ySplinePtr_;
	SplineCpg::Ptr 	 zDoubleSplinePtr_;
};


}  // end of hyq namespace




#endif /* HYQ_FOOTCPG_H_ */
