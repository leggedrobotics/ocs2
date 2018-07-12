/*
 * FeetPlannerBase.h
 *
 *  Created on: Feb 5, 2018
 *      Author: farbod
 */

#ifndef FEETPLANNERBASE_H_
#define FEETPLANNERBASE_H_

#include <memory>
#include <array>
#include <vector>

#include "ocs2_switched_model_interface/core/MotionPhaseDefinition.h"

namespace switched_model {

template <typename scalar_t, class cpg_t>
class FeetPlannerBase
{
public:
	typedef std::shared_ptr<FeetPlannerBase<scalar_t, cpg_t>> Ptr;

	typedef std::vector<bool> 		bool_array_t;
	typedef std::vector<int>	 	int_array_t;
	typedef std::vector<size_t> 	size_array_t;
	typedef std::vector<scalar_t> 	scalar_array_t;

	typedef cpg_t												foot_cpg_t;
	typedef std::array<typename std::shared_ptr<cpg_t>, 4> 		feet_cpg_ptr_t;
	typedef std::array<typename std::shared_ptr<const cpg_t>,4> feet_cpg_const_ptr_t;

	/**
	 * default constructor
	 */
	FeetPlannerBase() = default;

	/**
	 * destructor.
	 */
	virtual ~FeetPlannerBase() = default;

	/**
	 * Get the start times indices.
	 *
	 * @param startTimesIndices: start times indices
	 */
	virtual void getStartTimesIndices(std::array<int_array_t,4>& startTimesIndices) const;

	/**
	 * Get the final times indices.
	 *
	 * @param finalTimesIndices: final times indices
	 */
	virtual void getFinalTimesIndices(std::array<int_array_t,4>& finalTimesIndices) const;

	/**
	 * Plans the CPG for the swing legs in the indexed mode.
	 *
	 * @param [in] index: The index of the subsystem for which the CPG should be designed.
	 * @param [in] phaseIDsStock: An array of the natural number which gives a unique ID to 2^n (e.g. for
	 * a quadruped 2^4) possible stance leg choices.
	 * @param [in] eventTimes: The event times.
	 * @param [out] plannedCPG: An array of pointers to the CPG class for each endeffector.
	 */
	virtual void planSingleMode(
			const size_t& index,
			const size_array_t& phaseIDsStock,
			const scalar_array_t& eventTimes,
			feet_cpg_ptr_t& plannedCPG) = 0;

	/**
	 * clone the class
	 */
	virtual FeetPlannerBase<scalar_t, cpg_t>* clone() const = 0;

protected:

	/**
	 * based on the input phaseIDsStock finds the start subsystem and final subsystem of the swing
	 * phases of the a foot in each subsystem.
	 *
	 * @param [in] footIndex: Foot index
	 * @param [in] phaseIDsStock: The sequence of the motion phase IDs.
	 * @param [out] contactFlagStock: The sequence of the contact status for the requested leg.
	 * @param [out] startTimeIndexStock: eventTimes[startTimesIndex] will be the take-off time for the requested leg.
	 * @param [out] finalTimeIndexStock: eventTimes[finalTimesIndex] will be the touch-down time for the requested leg.
	 */
	void updateFootSchedule(const size_t& footIndex,
			const size_array_t& phaseIDsStock,
			const bool_array_t& contactFlagStock,
			int_array_t& startTimeIndexStock,
			int_array_t& finalTimeIndexStock) const;

	/**
	 * Extracts for each leg the contact sequence over the motion phase sequence.
	 *
	 * @param [in] phaseIDsStock: The sequence of the motion phase IDs.
	 * @param [out] contactFlagStock: The sequence of the contact status for each leg.
	 */
	void extractContactFlags(const size_array_t& phaseIDsStock,
			std::array<bool_array_t,4>& contactFlagStock) const;

	/**
	 * Finds the take-off and touch-down times indices for a specific leg.
	 *
	 * @param [in] index: The index of the subsystem for which the take-off and touch-down times indices should be found.
	 * @param [in] contactFlagStock: The sequence of the contact flag status.
	 * @param [out] startTimesIndex: The take-off time index for swing legs.
	 * @param [out] finalTimesIndex: The touch-down time index for swing legs.
	 */
	void findIndex (const size_t& index,
			const bool_array_t& contactFlagStock,
			int& startTimesIndex,
			int& finalTimesIndex) const;

	/*
	 * Variables
	 */
	std::array<int_array_t,4> startTimesIndices_;
	std::array<int_array_t,4> finalTimesIndices_;

};

}  // end of switched_model namespace

#include "implementation/FeetPlannerBase.h"

#endif /* FEETPLANNERBASE_H_ */
