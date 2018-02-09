/*
 * FeetZDirectionPlannerBase.h
 *
 *  Created on: Feb 5, 2018
 *      Author: farbod
 */

#ifndef FEETZDIRECTIONPLANNERBASE_H_
#define FEETZDIRECTIONPLANNERBASE_H_

#include <memory>
#include <array>
#include <vector>

#include "c_switched_model_interface/foot_planner/cpg/CPG_BASE.h"

namespace switched_model {

template <typename scalar_t=double>
class FeetZDirectionPlannerBase
{
public:
	typedef std::shared_ptr<FeetZDirectionPlannerBase> Ptr;

	typedef std::vector<bool> 		bool_array_t;
	typedef std::vector<size_t> 	size_array_t;
	typedef std::vector<scalar_t> 	scalar_array_t;

	typedef std::array<typename CPG_BASE<scalar_t>::Ptr,4> 		feet_cpg_ptr_t;
	typedef std::array<typename CPG_BASE<scalar_t>::ConstPtr,4> feet_cpg_const_ptr_t;

	/**
	 * default constructor
	 */
	FeetZDirectionPlannerBase()
	{}

	/**
	 * destructor.
	 */
	virtual ~FeetZDirectionPlannerBase()
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
	virtual void planSingleMode(const size_t& index,
			const size_array_t& phaseIDsStock,
			const scalar_array_t& switchingTimes,
			feet_cpg_ptr_t& plannedCPG) = 0;

	/**
	 * clone the class
	 */
	virtual FeetZDirectionPlannerBase<scalar_t>* clone() const = 0;

	virtual void getStartTimesIndices(std::array<size_array_t,4>& startTimesIndices) const {
		startTimesIndices = startTimesIndices_;
	}

	virtual void getFinalTimesIndices(std::array<size_array_t,4>& finalTimesIndices) const {
		finalTimesIndices = finalTimesIndices_;
	}

protected:
	std::array<size_array_t,4> startTimesIndices_;
	std::array<size_array_t,4> finalTimesIndices_;

};

}  // end of switched_model namespace


#endif /* FEETZDIRECTIONPLANNERBASE_H_ */
