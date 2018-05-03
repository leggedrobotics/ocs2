/*
 * FeetZDirectionPlanner.h
 *
 *  Created on: Jul 5, 2016
 *      Author: farbod
 */

#ifndef FEETZDIRECTIONPLANNER_H_
#define FEETZDIRECTIONPLANNER_H_

#include "c_switched_model_interface/foot_planner/FeetPlannerBase.h"
#include "c_switched_model_interface/foot_planner/cpg/CPG_BASE.h"

namespace switched_model {

template <typename scalar_t, class cpg_t>
class FeetZDirectionPlanner : public FeetPlannerBase<scalar_t, CPG_BASE<scalar_t>>
{
public:
	static_assert(std::is_base_of<CPG_BASE<scalar_t>, cpg_t>::value, "cpg_t must be inherited from CPG_BASE");

	typedef std::shared_ptr<FeetZDirectionPlanner<scalar_t, cpg_t>> Ptr;

	typedef FeetPlannerBase<scalar_t, CPG_BASE<scalar_t>> BASE;

	typedef typename BASE::bool_array_t 	bool_array_t;
	typedef typename BASE::size_array_t 	size_array_t;
	typedef typename BASE::scalar_array_t 	scalar_array_t;

	typedef typename BASE::feet_cpg_ptr_t		feet_cpg_ptr_t;
	typedef typename BASE::feet_cpg_const_ptr_t	feet_cpg_const_ptr_t;

	/**
	 * default constructor
	 */
	FeetZDirectionPlanner()
	: FeetZDirectionPlanner(0.15, 1.0)
	{}

	/**
	 * Constructor
	 *
	 * @param [in] swingLegLiftOff: Maximum swing leg lift-off
	 * @param [in] swingTimeScale: The scaling factor for adapting swing leg's lift-off
	 */
	FeetZDirectionPlanner(const scalar_t& swingLegLiftOff, const scalar_t& swingTimeScale = 1.0);

	/**
	 * copy constructor
	 *
	 * @param [in] rhs
	 */
	FeetZDirectionPlanner(const FeetZDirectionPlanner& rhs);

	/**
	 * destructor.
	 */
	~FeetZDirectionPlanner() override = default;

	/**
	 * Plans the CPG for the swing legs in the indexed mode.
	 *
	 * @param [in] index: The index of the subsystem for which the CPG should be designed.
	 * @param [in] phaseIDsStock: An array of the natural number which gives a unique ID to 2^n (e.g. for
	 * a quadruped 2^4) possible stance leg choices.
	 * @param [in] eventTimes: The event times.
	 * @param [out] plannedCPG: An array of pointers to the CPG class for each endeffector.
	 */
	void planSingleMode(const size_t& index,
			const size_array_t& phaseIDsStock,
			const scalar_array_t& eventTimes,
			feet_cpg_ptr_t& plannedCPG) override;

	/**
	 * clone the class
	 */
	FeetZDirectionPlanner<scalar_t, cpg_t>* clone() const override;


private:
	scalar_t swingLegLiftOff_ = 0.15;
	scalar_t swingTimeScale_ = 1.0;

	size_array_t phaseIDsStock_;
	scalar_array_t eventTimes_;
	std::array<bool_array_t,4> eesContactFlagStocks_;
};

}  // end of switched_model namespace

#include "implementation/FeetZDirectionPlanner.h"

#endif /* FEETZDIRECTIONPLANNER_H_ */
