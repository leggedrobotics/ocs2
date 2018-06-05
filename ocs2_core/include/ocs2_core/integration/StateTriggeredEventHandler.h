/*
 * StateTriggeredEventHandler.h
 *
 *  Created on: Mar 19, 2018
 *      Author: farbod
 */

#ifndef STATETRIGGEREDEVENTHANDLER_OCS2_H_
#define STATETRIGGEREDEVENTHANDLER_OCS2_H_

#include "ocs2_core/integration/SystemEventHandler.h"

namespace ocs2{

template <int STATE_DIM>
class StateTriggeredEventHandler : public SystemEventHandler<STATE_DIM>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::shared_ptr<StateTriggeredEventHandler<STATE_DIM>> Ptr;

	typedef SystemEventHandler<STATE_DIM> BASE;
	typedef typename BASE::scalar_t				scalar_t;
	typedef typename BASE::scalar_array_t 		scalar_array_t;
	typedef typename BASE::state_vector_t		state_vector_t;
	typedef typename BASE::state_vector_array_t state_vector_array_t;
	typedef typename BASE::dynamic_vector_t 	dynamic_vector_t;

	/**
	 * Default constructor
	 */
	StateTriggeredEventHandler()
	: BASE()
	{
		reset();
	}

	/**
	 * Default destructor
	 */
	virtual ~StateTriggeredEventHandler() = default;

	/**
	 * Resets the class.
	 */
	virtual void reset() override {

		BASE::reset();
		setEventTimesGuard();
	}

	/**
	 * Sets parameters to control event times detection.
	 *
	 * @param [in] minEventTimeDifference: Minimum accepted time difference between two consecutive events.
	 * @param [in] lastEventTriggeredTime: Last Time that an event is triggered.
	 * @param [in] lastGuardSurfacesValues: The value of the guard functions at lastEventTriggeredTime.
	 */
	virtual void setEventTimesGuard(
			const scalar_t& minEventTimeDifference = 1e-2,
			const scalar_t& lastEventTriggeredTime = std::numeric_limits<scalar_t>::lowest(),
			const dynamic_vector_t& lastGuardSurfacesValues = dynamic_vector_t::Zero(0)) {

		if (lastEventTriggeredTime > std::numeric_limits<scalar_t>::lowest() && lastGuardSurfacesValues.size()==0)
			throw std::runtime_error("Since the time of the last event is provided, "
					"the value of the guard functions at that time should also be provided.");

		minEventTimeDifference_ = minEventTimeDifference;
		lastEventTriggeredTime_ = lastEventTriggeredTime;
		guardSurfacesValuesPrevious_ = lastGuardSurfacesValues;
	}

	/**
	 * Gets the value of the guard surfaces.
	 *
	 * @return The value of the guard surfaces.
	 */
	const dynamic_vector_t& getGuardSurfacesValues() const {

		return guardSurfacesValuesPrevious_;
	}

	/**
	 * Checks if an event is activated.
	 *
	 * @param [in] state: Current state vector.
	 * @param [in] time: Current time.
	 * @return boolean:
	 */
	virtual bool checkEvent(
			const state_vector_t& state,
			const scalar_t& time) override {

//		std::cout << std::endl << "time: " << time << std::endl;

		// SystemEventHandler event
		systemEventHandlerTriggered_ = BASE::checkEvent(state, time);
		if (systemEventHandlerTriggered_==true)
			return true;

		//** StateTriggered event **//

		// No event will happen if one is recently hanned
		if (time-lastEventTriggeredTime_ < minEventTimeDifference_) {
//			std::cout << "Event Handeling is NOT active." << std::endl;
			return false;
		} else {
//			std::cout << "Event Handeling is active." << std::endl;
		}

		BASE::systemPtr_->computeGuardSurfaces(time, state, guardSurfacesValuesCurrent_);

//		std::cout << "guardSurfacesValue: ";
//		for (size_t i=0; i<guardSurfacesValuesCurrent_.size(); i++)
//			if (guardSurfacesValuesCurrent_(i)<0.6)
//				std::cout << ",   [" << i << "]: " << guardSurfacesValuesCurrent_(i);
//		std::cout << "\n";
//
//		for (size_t i=0; i<guardSurfacesValuesPrevious_.size(); i++)
//			std::cout << "[" << i << "]:\t" << guardSurfacesValuesPrevious_(i) <<
//				"\t-->\t" << guardSurfacesValuesCurrent_(i) << std::endl;

		bool eventTriggered = false;
		for (size_t i=0; i<guardSurfacesValuesPrevious_.size(); i++)
			if (guardSurfacesValuesCurrent_[i]<=0 && guardSurfacesValuesPrevious_(i)>0) {
				eventTriggered = true;
				triggeredEventSurface_ = i;
			}

		if (eventTriggered==false) {
			guardSurfacesValuesPrevious_ = guardSurfacesValuesCurrent_;
		}

		return eventTriggered;
	}

	/**
	 * The operation should be performed if an event is activated. The method gets references to the time and state
	 * trajectories. The current time and state are the last elements of their respective container.
	 * The method should also return a "Non-Negative" ID which indicates the a unique ID for the active events.
	 * Note that will the negative return values are reserved to handle internal events for the program.
	 *
	 * @param [out] stateTrajectory: The state trajectory which contains the current state vector as its last element.
	 * @param [out] timeTrajectory: The time trajectory which contains the current time as its last element.
	 * @retune boolean: A non-negative unique ID for the active events.
	 */
	virtual int handleEvent(
			state_vector_array_t& stateTrajectory,
			scalar_array_t& timeTrajectory) override {

		// SystemEventHandler event
		if (systemEventHandlerTriggered_==true)
			return BASE::handleEvent(stateTrajectory, timeTrajectory);

		// correcting for the zero crossing
		size_t lastIndex;
		scalar_t zeroCrossingTime;
		state_vector_t zeroCrossingState;
		computeZeroCrossing(stateTrajectory, timeTrajectory,
				lastIndex, zeroCrossingState, zeroCrossingTime);

		if (lastIndex>0) {
			timeTrajectory[lastIndex]  = zeroCrossingTime;
			timeTrajectory.erase(timeTrajectory.begin()+lastIndex+1, timeTrajectory.end());

			stateTrajectory[lastIndex] = zeroCrossingState;
			stateTrajectory.erase(stateTrajectory.begin()+lastIndex+1, stateTrajectory.end());
		}

		lastEventTriggeredTime_ = timeTrajectory[lastIndex];
		guardSurfacesValuesPrevious_.swap(guardSurfacesValuesCurrent_);

		// StateTriggered event
		return triggeredEventSurface_;
	}

	/**
	 * Computes the zero crossing.
	 *
	 * @param [in] stateTrajectory: The state trajectory which contains the current state vector as its last element.
	 * @param [in] timeTrajectory: The time trajectory which contains the current time as its last element.
	 * @param [out] lastIndex: The first index after crossing.
	 * @param [out] zeroCrossingState: State at zero crossing.
	 * @param [out] zeroCrossingTime: Time of the zero crossing.
	 */
	void computeZeroCrossing(
			const state_vector_array_t& stateTrajectory,
			const scalar_array_t& timeTrajectory,
			size_t& lastIndex,
			state_vector_t& zeroCrossingState,
			scalar_t& zeroCrossingTime) {

		if (timeTrajectory.size()==1) {
			lastIndex = 0;
			zeroCrossingTime = timeTrajectory.front();
			zeroCrossingState = stateTrajectory.front();

		} else {
			lastIndex = timeTrajectory.size()-1;

			if (timeTrajectory[timeTrajectory.size()-2]-lastEventTriggeredTime_ < minEventTimeDifference_)
				for (int i=timeTrajectory.size()-2; i>=0; i--) {
					BASE::systemPtr_->computeGuardSurfaces(timeTrajectory[i], stateTrajectory[i], guardSurfacesValuesPrevious_);
					if (guardSurfacesValuesPrevious_[triggeredEventSurface_]>0) {
						break;
					} else {
						guardSurfacesValuesCurrent_.swap(guardSurfacesValuesPrevious_);
						lastIndex = i;
					}
				}

			const scalar_t& t1 = timeTrajectory[lastIndex-1];
			const scalar_t& t2 = timeTrajectory[lastIndex];
			const state_vector_t& x1 = stateTrajectory[lastIndex-1];
			const state_vector_t& x2 = stateTrajectory[lastIndex];
			const scalar_t& v1 = guardSurfacesValuesPrevious_[triggeredEventSurface_];
			const scalar_t& v2 = guardSurfacesValuesCurrent_[triggeredEventSurface_];
			scalar_t delta_v = v1 - v2;

			zeroCrossingTime  = v1/delta_v*t2 - v2/delta_v*t1;
			zeroCrossingState = v1/delta_v*x2 - v2/delta_v*x1;
		}

		dynamic_vector_t zeroCrossingGuardSurfacesValues;
		BASE::systemPtr_->computeGuardSurfaces(zeroCrossingTime, zeroCrossingState, zeroCrossingGuardSurfacesValues);
//		std::cout << "\t zero-crossing time: " << zeroCrossingTime << std::endl;
//		std::cout << "\t zero-crossing value[" << triggeredEventSurface_ << "]: " <<
//				zeroCrossingGuardSurfacesValues[triggeredEventSurface_] << std::endl;
	}


protected:
	bool systemEventHandlerTriggered_;

	size_t triggeredEventSurface_;

	dynamic_vector_t guardSurfacesValuesCurrent_;
	dynamic_vector_t guardSurfacesValuesPrevious_;	// memory

	scalar_t minEventTimeDifference_;
	scalar_t lastEventTriggeredTime_;	// memory

};


} // namespace ocs2


#endif /* STATETRIGGEREDEVENTHANDLER_OCS2_H_ */
