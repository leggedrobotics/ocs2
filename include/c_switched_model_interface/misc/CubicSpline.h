/*
 * CubicSpline.h
 *
 *  Created on: Apr 30, 2017
 *      Author: farbod
 */

#ifndef CUBICSPLINE_H_
#define CUBICSPLINE_H_

#include <limits>
#include <cmath>

namespace hyq {

class CubicSpline
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::shared_ptr<CubicSpline> Ptr;

	CubicSpline() {}
	~CubicSpline() {}

	void set(const double& t0,  const double& p0, const double& v0,
			const double& t1, const double& p1, const double& v1) {

		if (t1 < t0)
			throw std::runtime_error("The final time should be greater than start time");

		t0_ = t0;
		t1_ = t1;
		dt_ = t1-t0;

		double dp = p1-p0;
		double dv = v1-v0;

		c0_ = p0;
		c1_ = v0 * dt_;
		c2_ =  3.0*dp - (3.0*v0 + dv) * dt_;
		c3_ = -2.0*dp + (2.0*v0 + dv) * dt_;
	}

	double evaluateSplinePosition(const double& time) {
		double tn = normalizedTime(time);
		return c3_*std::pow(tn,3) + c2_*std::pow(tn,2) + c1_*tn + c0_;
	}

	double evaluateSplineVelocity(const double& time) {
		double tn = normalizedTime(time);
		return (3.0*c3_*std::pow(tn,2) + 2.0*c2_*tn + c1_) / dt_;
	}

	double evaluateSplineAcceleration(const double& time) {
		double tn = normalizedTime(time);
		return (6.0*c3_*tn + 2.0*c2_) / std::pow(dt_,2);
	}

protected:
	double normalizedTime(const double& t) {
		if (dt_ > std::numeric_limits<double>::epsilon())
			return (t-t0_)/dt_;
		else
			return 0.0;
	}

private:
	double t0_;
	double t1_;
	double dt_;

	double c0_;
	double c1_;
	double c2_;
	double c3_;
};


}  // end of hyq namespace

#endif /* CUBICSPLINE_H_ */
