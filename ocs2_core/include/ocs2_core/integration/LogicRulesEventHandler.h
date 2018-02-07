/*
 * LogicRulesEventHandler.h
 *
 *  Created on: Dec 1, 2017
 *      Author: farbod
 */

#ifndef LOGICRULESEVENTHANDLER_OCS2_H_
#define LOGICRULESEVENTHANDLER_OCS2_H_

#include <memory>
#include <vector>
#include <exception>

#include "ocs2_core/logic/LogicRulesBase.h"
#include "ocs2_core/integration/EventHandler.h"

namespace ocs2{

template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
class LogicRulesEventHandler : public EventHandler<STATE_DIM>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	static_assert(std::is_base_of<LogicRulesBase<STATE_DIM, INPUT_DIM>, LOGIC_RULES_T>::value, "LOGIC_RULES_T must inherit from LogicRulesBase");

	typedef std::shared_ptr<LogicRulesEventHandler<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>> Ptr;

	typedef Eigen::Matrix<double,STATE_DIM,1> State_T;

	LogicRulesEventHandler()
	: eventTimes_(1, MIN_TIME),
	  prevActiveSeg_(0)
	{}


	~LogicRulesEventHandler() {}

	/**
	 * Initializes the system dynamics.
	 *
	 * @param [in] logicRules: A class containing the logic rules.
	 * @param [in] switchingTimes: The switching time vector.
	 * @param [in] initState: Initial state.
	 * @param [in] activeSubsystemIndex: Current active subsystem index.
	 * @param [in] algorithmName: The algorithm that uses this class.
	 */
	virtual void initializeModel(const LOGIC_RULES_T& logicRules, const std::vector<double>& switchingTimes,
			const State_T& initState, const size_t& activeSubsystemIndex=0, const char* algorithmName=NULL) {

		eventTimes_ = logicRules.logicRulesSwitchingTimes();

		if (eventTimes_.empty()==false) {
			eventTimes_.insert(eventTimes_.begin(), MIN_TIME);
		} else {
			eventTimes_.push_back(MIN_TIME);
		}

		// FIXME: get also the state map
	}

	/**
	 * Checks if an event is activated.
	 *
	 * @param [in] state: Current state vector.
	 * @param [in] time: Current time.
	 * @return boolean:
	 */
	virtual bool checkEvent(const State_T& state, const double& time) override {

		return checkEvent(time) || EventHandler<STATE_DIM>::killIntegration_;
	}

	/**
	 * The operation should be performed if an event is activated.
	 *
	 * @param [in] state: Current state vector.
	 * @param [in] time: Current time.
	 */
	virtual void handleEvent(const State_T& state, const double& time) override {

		if (EventHandler<STATE_DIM>::killIntegration_==true) {
			/* throw an exception which stops the integration */
			throw std::runtime_error("Integration terminated due to external event specified by user.");

		} else {
			/* throw a LogicRulesException exception which will be catch later */
			throw EventHandlerException(eventTimes_[prevActiveSeg_]);
		}
	}

protected:

	bool checkEvent(const double& time) {

		int currActiveSeg = find(time);

		if (currActiveSeg==prevActiveSeg_){
			return false;

		} else {
			prevActiveSeg_ = currActiveSeg;
			return true;
		}
	}

    /**
     * Finds the index of the greatest smaller time stamp index for the enquiry time.
     *
     * @param [in] enquiryTime: The enquiry time for interpolation.
     * @return The greatest smaller time stamp index.
     */
	int find(const double& enquiryTime) {

		int index = -1;

		if (eventTimes_.at(prevActiveSeg_) > enquiryTime) {
			for (int i=prevActiveSeg_; i>=0; i--)  {
				index = i;
				if (eventTimes_.at(i) <= enquiryTime)
					break;
			}
		} else {
			for (int i=prevActiveSeg_; i<eventTimes_.size(); i++) {
				index = i;
				if (eventTimes_.at(i) > enquiryTime) {
					index = i-1;
					break;
				}
			}
		}

		// throw error if index is wrong
		if(index < 0)
			throw std::runtime_error("LinearInterpolation.h : index in protected member find((const double& enquiryTime) not computed properly");

		return index;
	}

private:
	const double MIN_TIME = 0.0;

	std::vector<double> eventTimes_;

	int prevActiveSeg_;

};

} // namespace ocs2

#endif /* LOGICRULESEVENTHANDLER_OCS2_H_ */
