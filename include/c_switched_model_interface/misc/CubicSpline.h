/*
 * CubicSpline.h
 *
 *  Created on: Apr 30, 2017
 *      Author: farbod
 */

#ifndef CUBICSPLINE_H_
#define CUBICSPLINE_H_

#include <limits>
#include <memory>
#include <cmath>

namespace switched_model {

template <typename scalar_t = double>
class CubicSpline
{
public:
	typedef std::shared_ptr<CubicSpline> Ptr;

	CubicSpline() {}
	~CubicSpline() {}

	void set(const scalar_t& t0,  const scalar_t& p0, const scalar_t& v0,
			const scalar_t& t1, const scalar_t& p1, const scalar_t& v1) {

		if (t1 < t0)
			throw std::runtime_error("The final time should be greater than start time");

		t0_ = t0;
		t1_ = t1;
		dt_ = t1-t0;

		scalar_t dp = p1-p0;
		scalar_t dv = v1-v0;

		c0_ = p0;
		c1_ = v0 * dt_;
		c2_ =  3.0*dp - (3.0*v0 + dv) * dt_;
		c3_ = -2.0*dp + (2.0*v0 + dv) * dt_;
	}

	scalar_t evaluateSplinePosition(const scalar_t& time) const {
		scalar_t tn = normalizedTime(time);
		return c3_*std::pow(tn,3) + c2_*std::pow(tn,2) + c1_*tn + c0_;
	}

	scalar_t evaluateSplineVelocity(const scalar_t& time) const {
		scalar_t tn = normalizedTime(time);
		return (3.0*c3_*std::pow(tn,2) + 2.0*c2_*tn + c1_) / dt_;
	}

	scalar_t evaluateSplineAcceleration(const scalar_t& time) const {
		scalar_t tn = normalizedTime(time);
		return (6.0*c3_*tn + 2.0*c2_) / std::pow(dt_,2);
	}

protected:
	scalar_t normalizedTime(const scalar_t& t) const {
		if (dt_ > std::numeric_limits<scalar_t>::epsilon())
			return (t-t0_)/dt_;
		else
			return 0.0;
	}

private:
	scalar_t t0_;
	scalar_t t1_;
	scalar_t dt_;

	scalar_t c0_;
	scalar_t c1_;
	scalar_t c2_;
	scalar_t c3_;
};


}  // end of switched_model namespace

#endif /* CUBICSPLINE_H_ */
