/*
 * SystemObservation.h
 *
 *  Created on: May 23, 2018
 *      Author: farbod
 */

#ifndef SYSTEMOBSERVATION_OCS2_H_
#define SYSTEMOBSERVATION_OCS2_H_

#include <ocs2_core/Dimensions.h>

namespace ocs2{

/**
 * This class implements single thread SLQ algorithm.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 * @tparam LOGIC_RULES_T: Logic Rules type (default NullLogicRules).
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class SystemObservation
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef Dimensions<STATE_DIM, INPUT_DIM> DIMENSIONS;
	typedef typename DIMENSIONS::scalar_t			scalar_t;
	typedef typename DIMENSIONS::state_vector_t 	state_vector_t;
	typedef typename DIMENSIONS::input_vector_t 	input_vector_t;

	SystemObservation() = default;
	~SystemObservation() = default;

	void swap(SystemObservation<STATE_DIM, INPUT_DIM>& other) {

		std::swap(time_, other.time_);
		std::swap(state_, other.state_);
		std::swap(input_, other.input_);
		std::swap(subsystem_, other.subsystem_);
	}

	inline scalar_t& time() { return time_; };
	inline const scalar_t& time() const { return time_; };

	inline state_vector_t& state() { return state_; };
	inline const state_vector_t& state() const { return state_; };
	inline scalar_t& state(const size_t& i) { return state_(i); };
	inline const scalar_t& state(const size_t& i) const { return state_(i); };

	inline input_vector_t& input() { return input_; };
	inline const input_vector_t& input() const { return input_; };
	inline scalar_t& input(const size_t& i) { return input_(i); };
	inline const scalar_t& input(const size_t& i) const { return input_(i); };

	inline size_t& subsystem() { return subsystem_; };
	inline const size_t& subsystem() const { return subsystem_; };

private:
	scalar_t 		time_;
	state_vector_t 	state_;
	input_vector_t 	input_;
	size_t			subsystem_;

};

} // namespace ocs2

#endif /* SYSTEMOBSERVATION_OCS2_H_ */
