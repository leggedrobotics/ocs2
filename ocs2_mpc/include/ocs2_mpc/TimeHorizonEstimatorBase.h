/*
 * TimeHorizonEstimatorBase.h
 *
 * Created on: 29.08.2016
 * 	   Author: mgiftthaler<mgiftthaler@ethz.ch> 
 * 
 */

#ifndef TIMEHORIZONESTIMATORBASE_H_
#define TIMEHORIZONESTIMATORBASE_H_

#include <memory>
#include <Eigen/Dense>

namespace ocs2 {

/**
 *
 * @tparam STATE_DIM
 */
template<size_t STATE_DIM>
class TimeHorizonEstimatorBase {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::shared_ptr<TimeHorizonEstimatorBase<STATE_DIM> > Ptr;

	TimeHorizonEstimatorBase() :
		currentTimeHorizon_(1000) {}

	virtual ~TimeHorizonEstimatorBase() {}

	/**
	 * Updates the time horizon
	 * @param [in] currentInitialState
	 */
	virtual void updateTimeHorizon(
			const Eigen::Matrix<double, STATE_DIM, 1> &currentInitialState) { currentTimeHorizon_ = 0.0; };

	/**
	 * Gets time horizon
	 * @return double
	 */
	const double getTimeHorizon() const { return currentTimeHorizon_; }

protected:
	double currentTimeHorizon_;
};


}


#endif /* TIMEHORIZONESTIMATORBASE_H_ */
