/*
 * CubicSpline.h
 *
 *  Created on: Apr 30, 2017
 *      Author: farbod
 */

#ifndef CUBICSPLINE_H_
#define CUBICSPLINE_H_

#include <cmath>
#include <limits>
#include <memory>
#include <string>

namespace switched_model {

template <typename scalar_t = double>
class CubicSpline {
 public:
  typedef std::shared_ptr<CubicSpline> Ptr;

  CubicSpline() = default;

  ~CubicSpline() = default;

  void setConstant(const scalar_t& p0) {
    t0_ = 0.0;
    t1_ = 1.0;
    dt_ = 1.0;

    c0_ = p0;
    c1_ = c2_ = c3_ = 0.0;

    dev_c0_ = dev_c1_ = dev_c2_ = dev_c3_ = 0.0;
  }

  void set(const scalar_t& t0, const scalar_t& p0, const scalar_t& v0, const scalar_t& t1, const scalar_t& p1, const scalar_t& v1) {
    if (t1 < t0)
      throw std::runtime_error("The final time should not be less than the start time (" + std::to_string(t0) + "<=" + std::to_string(t1) +
                               ").");

    t0_ = t0;
    t1_ = t1;
    dt_ = t1 - t0;

    scalar_t dp = p1 - p0;
    scalar_t dv = v1 - v0;

    dev_c0_ = 0.0;
    dev_c1_ = v0;
    dev_c2_ = -(3.0 * v0 + dv);
    dev_c3_ = (2.0 * v0 + dv);

    c0_ = dev_c0_ * dt_ + p0;
    c1_ = dev_c1_ * dt_;
    c2_ = dev_c2_ * dt_ + 3.0 * dp;
    c3_ = dev_c3_ * dt_ - 2.0 * dp;
  }

  scalar_t evaluateSplinePosition(const scalar_t& time) const {
    scalar_t tn = normalizedTime(time);
    return c3_ * std::pow(tn, 3) + c2_ * std::pow(tn, 2) + c1_ * tn + c0_;
  }

  scalar_t evaluateSplineVelocity(const scalar_t& time) const {
    scalar_t tn = normalizedTime(time);
    return (3.0 * c3_ * std::pow(tn, 2) + 2.0 * c2_ * tn + c1_) / dt_;
  }

  scalar_t evaluateSplineAcceleration(const scalar_t& time) const {
    scalar_t tn = normalizedTime(time);
    return (6.0 * c3_ * tn + 2.0 * c2_) / std::pow(dt_, 2);
  }

  scalar_t evaluateStartTimeDerivative(const scalar_t& t) const {
    if (dt_ > std::numeric_limits<scalar_t>::epsilon()) {
      scalar_t tn = normalizedTime(t);
      scalar_t dev_coff = -(dev_c3_ * std::pow(tn, 3) + dev_c2_ * std::pow(tn, 2) + dev_c1_ * tn + dev_c0_);

      scalar_t dev_tn = -(t1_ - t) / std::pow(dt_, 2);

      return evaluateSplineVelocity(t) * dt_ * dev_tn + dev_coff;
    } else {
      return 0.0;
    }
  }

  scalar_t evaluateFinalTimeDerivative(const scalar_t& t) const {
    if (dt_ > std::numeric_limits<scalar_t>::epsilon()) {
      scalar_t tn = normalizedTime(t);
      scalar_t dev_coff = (dev_c3_ * std::pow(tn, 3) + dev_c2_ * std::pow(tn, 2) + dev_c1_ * tn + dev_c0_);

      scalar_t dev_tn = -(t - t0_) / std::pow(dt_, 2);

      return evaluateSplineVelocity(t) * dt_ * dev_tn + dev_coff;
    } else {
      return 0.0;
    }
  }

 protected:
  scalar_t normalizedTime(const scalar_t& t) const {
    if (dt_ > std::numeric_limits<scalar_t>::epsilon())
      return (t - t0_) / dt_;
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

  scalar_t dev_c0_;  // derivative w.r.t. dt_
  scalar_t dev_c1_;  // derivative w.r.t. dt_
  scalar_t dev_c2_;  // derivative w.r.t. dt_
  scalar_t dev_c3_;  // derivative w.r.t. dt_
};

}  // namespace switched_model

#endif /* CUBICSPLINE_H_ */
