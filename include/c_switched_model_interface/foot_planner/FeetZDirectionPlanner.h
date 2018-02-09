/*
 * FeetZDirectionPlanner.h
 *
 *  Created on: Jul 5, 2016
 *      Author: farbod
 */

#ifndef FEETZDIRECTIONPLANNER_H_
#define FEETZDIRECTIONPLANNER_H_

#include "c_switched_model_interface/core/MotionPhaseDefinition.h"
#include "c_switched_model_interface/foot_planner/FeetZDirectionPlannerBase.h"

namespace switched_model {

template <class CPG_T, typename scalar_t=double>
class FeetZDirectionPlanner : public FeetZDirectionPlannerBase<scalar_t>
{
public:
	static_assert(std::is_base_of<CPG_BASE<scalar_t>, CPG_T>::value, "CPG_T must inherit from CPG_BASE");

	typedef std::shared_ptr<FeetZDirectionPlanner<CPG_T,scalar_t>> Ptr;

	typedef FeetZDirectionPlannerBase<scalar_t> BASE;

	typedef typename BASE::bool_array_t 	bool_array_t;
	typedef typename BASE::size_array_t 	size_array_t;
	typedef typename BASE::scalar_array_t 	scalar_array_t;

	typedef typename BASE::feet_cpg_ptr_t		feet_cpg_ptr_t;
	typedef typename BASE::feet_cpg_const_ptr_t	feet_cpg_const_ptr_t;

	/**
	 * default constructor
	 */
	FeetZDirectionPlanner() = delete;

	/**
	 * constructor
	 *
	 * @param [in] swingLegLiftOff: Maximum swing leg lift-off
	 * @param [in] swingTimeScale: The scaling factor for adapting swing leg's lift-off
	 */
	FeetZDirectionPlanner(const scalar_t& swingLegLiftOff = 0.15, const scalar_t& swingTimeScale = 1.0);

	/**
	 * copy constructor
	 *
	 * @param [in] rhs
	 */
	FeetZDirectionPlanner(const FeetZDirectionPlanner& rhs);

	/**
	 * destructor.
	 */
	virtual ~FeetZDirectionPlanner()
	{}

	/**
	 * Plans the CPG for the swing legs in the mode indexed by activeSubsystemIndex.
	 *
	 * @param [in] index: The index of the subsystem for which the CPG should be designed.
	 * @param [in] phaseIDsStock: An array of the natural number which gives a unique ID to 2^n (e.g. for
	 * a quadruped 2^4) possible stance leg choices.
	 * @param [in] switchingTimes: The switching times.
	 * @param [out] plannedCPG: An array of pointers to the CPG class for each endeffector.
	 */
	void planSingleMode(const size_t& index,
			const size_array_t& phaseIDsStock,
			const scalar_array_t& switchingTimes,
			feet_cpg_ptr_t& plannedCPG) override;

	/**
	 * clone the class
	 */
	virtual FeetZDirectionPlanner<CPG_T, scalar_t>* clone() const override;

protected:

	/**
	 * based on the input phaseIDsStock findes the start susbsystem and final subsystem of the swing
	 * phases for each subsystem.
	 *
	 * @param [in] phaseIDsStock: The sequence of the motion phase IDs
	 * @param [out] eeContactFlagStocks: The sequence of the stance leges.
	 * @param [out] startTimesIndices: switchingTimes[startTimesIndex] will be the takoff time for swing legs.
	 * @param [out] finalTimesIndices: switchingTimes[finalTimesIndex] will be the touch-down time for swing legs.
	 */
	void updateFeetZDirectionPlanner(
			const size_array_t& phaseIDsStock,
			std::array<bool_array_t,4>& eeContactFlagStocks,
			std::array<size_array_t,4>& startTimesIndices,
			std::array<size_array_t,4>& finalTimesIndices);

	/**
	 *
	 * @param [in] activeSubsystemIndex: The index of the subsystem for which the CPG should be designed.
	 * @param [in] defaultStartMode: The stances leg before the switchingTimes.front()
	 * @param [in] defaultFinalMode: The stances leg after the switchingTimes.back()
	 * @param [in] stanceLegsStock: The sequence of the stance leges.
	 * @param [out] startTimesIndex: The takoff time index for swing legs.
	 * @param [out] finalTimesIndex: The touch-down time index for swing legs.
	 */
	static void FindIndex (const size_t& index,
			const bool_array_t& contactFlagStock,
			int& startTimesIndex,
			int& finalTimesIndex);

private:
	scalar_t swingLegLiftOff_ = 0.15;
	scalar_t swingTimeScale_ = 1.0;

	size_array_t 	phaseIDsStock_;
	scalar_array_t 	switchingTimes_;
	std::array<bool_array_t,4> eesContactFlagStocks_;
};

}  // end of switched_model namespace

#include "implementation/FeetZDirectionPlanner.h"

#endif /* FEETZDIRECTIONPLANNER_H_ */
