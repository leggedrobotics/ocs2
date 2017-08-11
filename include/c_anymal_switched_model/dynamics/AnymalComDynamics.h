/*
 * AnymalDynamics.h
 *
 *  Created on: Aug 11, 2017
 *      Author: Jan Carius
 */

#ifndef ANYMAL_ANYMALCOMDYNAMICS_H_
#define ANYMAL_ANYMALCOMDYNAMICS_H_

#include <c_switched_model_interface/ComDynamicsBase.h>

#include <quadruped_model/QuadrupedModel.hpp>

namespace anymal
{

class AnymalComDynamics : public ComDynamicsBase<AnymalComDynamics, 12>
{
public:
  AnymalComDynamics();
  ~AnymalComDynamics() {}

  virtual Eigen::Matrix<double,6,6> comInertia(
      const Eigen::Matrix<double,12,1>& q);

  virtual Eigen::Matrix<double,4,4> comHomogeneous(
      const Eigen::Matrix<double,12,1>& q);

  virtual Eigen::Matrix<double,6,6> comInertiaDerivative(
      const Eigen::Matrix<double,12,1>& q,
      const Eigen::Matrix<double,12,1>& dq);

  virtual Eigen::Matrix<double,6,12> comMomentumJacobian(
      const Eigen::Matrix<double,12,1>& q);

  virtual Eigen::Matrix<double,6,12> comMomentumJacobianDerivative(
      const Eigen::Matrix<double,12,1>& q,
      const Eigen::Matrix<double,12,1>& dq);

  virtual Eigen::Matrix<double,3,1> comVelocityInBaseFrame(
      const Eigen::Matrix<double,12,1>& q, const Eigen::Matrix<double,12,1>& dq);

protected:
  void setState(const Eigen::Matrix<double,12,1>& q);
  void setState(const Eigen::Matrix<double,12,1>& q,
                const Eigen::Matrix<double,12,1>& dq);

  quadruped_model::QuadrupedModel model_;
  quadruped_model::QuadrupedState state_;

};

}  // end of anymal namespace

#endif /* ANYMAL_ANYMALCOMDYNAMICS_H_ */
