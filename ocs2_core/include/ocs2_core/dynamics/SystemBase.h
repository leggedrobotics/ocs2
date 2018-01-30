/*
 * SystemBase.h
 *
 *  Created on: Jan 3, 2016
 *      Author: farbod
 */

#ifndef SYSTEMBASE_OCS2_H_
#define SYSTEMBASE_OCS2_H_

#include <Eigen/Dense>

namespace ocs2{

/**
 * The base class for autonomous system dynamics.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 */
template <int STATE_DIM>
class SystemBase
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * Default constructor
     */
	SystemBase()
		: numFunctionCalls_(0) {}

	/**
	 * Default destructor
	 */
	virtual ~SystemBase() {}

	/**
	 * Gets the number of function calls.
	 *
	 * @return size_t: number of function calls
	 */
	size_t getNumFunctionCalls() {return numFunctionCalls_;}

	/**
	 * Resets the number of function calls to zero.
	 *
	 */
	void resetNumFunctionCalls() {numFunctionCalls_ = 0;}

	/**
	 * Computes the autonomous system dynamics.
	 * @param [in] t: Current time.
	 * @param [in] x: Current state.
	 * @param [out] dxdt: Current state time derivative
	 */
	virtual void computeDerivative(
			const double& t,
			const Eigen::Matrix<double,STATE_DIM,1>& x,
			Eigen::Matrix<double,STATE_DIM,1>& dxdt) = 0;

	/**
	 * State map at the transition time
	 *
	 * @param [in] time: transition time
	 * @param [in] state: transition state
	 * @param [out] mappedState: mapped state after transition
	 */
	virtual void mapState(const double& time, const Eigen::Matrix<double,STATE_DIM,1>& state,
			Eigen::Matrix<double,STATE_DIM,1>& mappedState) {

		mappedState = state;
	}

protected:
	size_t numFunctionCalls_;

private:
};

} // namespace ocs2

#endif /* SYSTEMBASE_H_ */
