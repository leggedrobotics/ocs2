/*
 * FootCPG.h
 *
 *  Created on: Jul 10, 2017
 *      Author: farbod
 */

#ifndef FOOTCPG_H_
#define FOOTCPG_H_

#include <Eigen/Core>
#include <cmath>
#include "ocs2_switched_model_interface/foot_planner/cpg/SplineCPG.h"
#include "ocs2_switched_model_interface/foot_planner/CubicSpline.h"

namespace switched_model {

template <typename scalar_t = double>
class FootCPG {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<FootCPG<scalar_t>> Ptr;

  typedef Eigen::Matrix<scalar_t, 3, 1> vector_3d_t;

  FootCPG() : FootCPG(0.15, 1.0) {}

  FootCPG(const scalar_t& swingLegLiftOff, const scalar_t& swingTimeScale = 1.0) {
    xSplinePtr_ = typename CubicSpline<scalar_t>::Ptr(new CubicSpline<scalar_t>());
    ySplinePtr_ = typename CubicSpline<scalar_t>::Ptr(new CubicSpline<scalar_t>());
    zDoubleSplinePtr_ = typename SplineCPG<scalar_t>::Ptr(new SplineCPG<scalar_t>(swingLegLiftOff, swingTimeScale));
  }

  ~FootCPG() = default;

  void setConstant(const vector_3d_t& currentPos) {
    xSplinePtr_->setConstant(currentPos(0));
    ySplinePtr_->setConstant(currentPos(1));
    zDoubleSplinePtr_->setConstant();
  }

  void set(const scalar_t& currentTime, const scalar_t& liftoffTime, const scalar_t& touchdownTime, const vector_3d_t& currentPos,
           const vector_3d_t& currentVel, const vector_3d_t& touchdownPos, const scalar_t& swingLegLiftOff) {
    xSplinePtr_->set(currentTime, currentPos(0) /*p0*/, currentVel(0) /*v0*/, touchdownTime /*t1*/, touchdownPos(0) /*p1*/, 0.0 /*v1*/);
    ySplinePtr_->set(currentTime, currentPos(1) /*p0*/, currentVel(1) /*v0*/, touchdownTime /*t1*/, touchdownPos(1) /*p1*/, 0.0 /*v1*/);
    zDoubleSplinePtr_->set(liftoffTime, touchdownTime, swingLegLiftOff);
  }

  scalar_t adaptiveSwingLegLiftOff(const scalar_t& startTime, const scalar_t& finalTime) {
    return zDoubleSplinePtr_->adaptiveSwingLegLiftOff(startTime, finalTime);
  }

  /**
   *
   * @param time
   * @return
   */
  Eigen::Vector3d calculatePosition(const scalar_t& time) const {
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
  Eigen::Vector3d calculateVelocity(const scalar_t& time) const {
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
  Eigen::Vector3d calculateAcceleration(const scalar_t& time) const {
    Eigen::Vector3d a;
    a(0) = xSplinePtr_->evaluateSplineAcceleration(time);
    a(1) = ySplinePtr_->evaluateSplineAcceleration(time);
    a(2) = zDoubleSplinePtr_->calculateAcceleration(time);
    return a;
  }

  virtual Eigen::Vector3d calculateStartTimeDerivative(const scalar_t& time) const {
    Eigen::Vector3d a;
    a(0) = xSplinePtr_->evaluateStartTimeDerivative(time);
    a(1) = ySplinePtr_->evaluateStartTimeDerivative(time);
    a(2) = zDoubleSplinePtr_->calculateStartTimeDerivative(time);
    return a;
  }

  virtual Eigen::Vector3d calculateFinalTimeDerivative(const scalar_t& time) const {
    Eigen::Vector3d a;
    a(0) = xSplinePtr_->evaluateFinalTimeDerivative(time);
    a(1) = ySplinePtr_->evaluateFinalTimeDerivative(time);
    a(2) = zDoubleSplinePtr_->calculateFinalTimeDerivative(time);
    return a;
  }

 private:
  typename CubicSpline<scalar_t>::Ptr xSplinePtr_;
  typename CubicSpline<scalar_t>::Ptr ySplinePtr_;
  typename SplineCPG<scalar_t>::Ptr zDoubleSplinePtr_;
};

}  // namespace switched_model

#endif /* FOOTCPG_H_ */
