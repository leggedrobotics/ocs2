/*
 * CoMDynamicsDerivativeNumerical.h
 *
 *  Created on: Feb 10, 2016
 *      Author: farbod
 */

#ifndef HYQ_COMDYNAMICSDERIVATIVENUMERICAL_H_
#define HYQ_COMDYNAMICSDERIVATIVENUMERICAL_H_

#include <dynamics/SystemDynamicsLinearizer.h>

#include "c_hyq_switched_model/CoMDynamics.h"

namespace hyq {

class CoMDynamicsDerivativeNumerical : public ocs2::DerivativesBase<12,12>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	CoMDynamicsDerivativeNumerical(const std::array<bool,4>& stanceLegs, const Eigen::Vector3d& gravity)

	: stanceLegs_(stanceLegs),
	  gravity_(gravity),
	  hyqComDinamicsPtr_( std::allocate_shared<CoMDynamics, Eigen::aligned_allocator<CoMDynamics>>( Eigen::aligned_allocator<CoMDynamics>(),stanceLegs,gravity) ),
	  hyqDynamicsLinearizer_(hyqComDinamicsPtr_)

	{}


	CoMDynamicsDerivativeNumerical(const CoMDynamicsDerivativeNumerical& other)

	: stanceLegs_(other.stanceLegs_),
	  gravity_(other.gravity_),
	  hyqComDinamicsPtr_( std::allocate_shared<CoMDynamics, Eigen::aligned_allocator<CoMDynamics>>( Eigen::aligned_allocator<CoMDynamics>(),stanceLegs_,gravity_) ),
	  hyqDynamicsLinearizer_(hyqComDinamicsPtr_)
	{}


	virtual ~CoMDynamicsDerivativeNumerical(){}

	virtual void initializeModel(const std::vector<size_t>& systemStockIndexes, const std::vector<scalar_t>& switchingTimes,
			const state_vector_t& initState, const size_t& activeSubsystemIndex=0, const char* algorithmName=NULL) override  {

		hyqComDinamicsPtr_->initializeModel(switchingTimes, initState, activeSubsystemIndex, algorithmName);
	}

	virtual void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const control_vector_t& u) override  {

		ocs2::DerivativesBase<12,12>::setCurrentStateAndControl(t, x, u);
		hyqDynamicsLinearizer_.setCurrentStateAndControl(t, x, u);
	}

	virtual void getDerivativeState(state_matrix_t& A)  override {
		hyqDynamicsLinearizer_.getDerivativeState(A);
	}

	virtual void getDerivativesControl(control_gain_matrix_t& B)  override {
		hyqDynamicsLinearizer_.getDerivativesControl(B);
		for (size_t i=0; i<4; i++)
			if (stanceLegs_[i]==0)
				B.block<6,3>(6,i*3).setZero();
	}

	virtual std::shared_ptr<DerivativesBase<12, 12> > clone() const override  {
		return std::allocate_shared<CoMDynamicsDerivativeNumerical, Eigen::aligned_allocator<CoMDynamicsDerivativeNumerical>>(
				Eigen::aligned_allocator<CoMDynamicsDerivativeNumerical>(), *this);
	}

	void setFeetPositions(const std::array<Eigen::Vector3d, 4>& origin_Origin2StanceFeet)  {
		hyqComDinamicsPtr_->setFeetPositions(origin_Origin2StanceFeet);
	}

	void setJointState(const Eigen::Matrix<double,12,1>& q, const Eigen::Matrix<double,12,1>& dq)  {
		hyqComDinamicsPtr_->setJointState(q, dq);
	}


private:
	std::array<bool,4> stanceLegs_;
	Eigen::Vector3d    gravity_;

	std::shared_ptr<CoMDynamics> hyqComDinamicsPtr_;
	SystemDynamicsLinearizer<12,12> hyqDynamicsLinearizer_;


};

}  // end of hyq namespace

#endif /* HYQ_COMDYNAMICSDERIVATIVENUMERICAL_H_ */
