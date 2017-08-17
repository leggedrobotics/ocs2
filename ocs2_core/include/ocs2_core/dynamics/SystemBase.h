/*
 * SystemBase.h
 *
 *  Created on: 17.06.2015
 *      Author: neunertm
 */

#ifndef SYSTEMBASE_OCS2_H_
#define SYSTEMBASE_OCS2_H_

#include <Eigen/Dense>

namespace ocs2{

/**
 * SystemBase
 * @tparam STATE_DIM
 */
template <int STATE_DIM>
class SystemBase
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * Constructor
     */
	SystemBase()
		: numFunctionCalls_(0) {}

	virtual ~SystemBase() {}

	/**
	 * Gets the number of function calls
	 * @return
	 */
	size_t getNumFunctionCalls() {return numFunctionCalls_;}

	/**
	 * Computes derivative
	 * @param [in] t
	 * @param [in] state
	 * @param [out] derivative
	 */
	virtual void computeDerivative(
			const double& t,
			const Eigen::Matrix<double,STATE_DIM,1>& state,
			Eigen::Matrix<double,STATE_DIM,1>& derivative) = 0;

protected:
	size_t numFunctionCalls_;

private:
};

} // namespace ocs2

#endif /* SYSTEMBASE_H_ */
