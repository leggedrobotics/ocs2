/*
 * Feet3DPlanner.h
 *
 *  Created on: Mar 8, 2018
 *      Author: farbod
 */

namespace switched_model {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename scalar_t>
Feet3DPlanner<scalar_t>::Feet3DPlanner(
		const scalar_t& swingLegLiftOff,
		const scalar_t& swingTimeScale /*= 1.0*/)

	: swingLegLiftOff_(swingLegLiftOff),
	  swingTimeScale_(swingTimeScale),
	  phaseIDsStock_(0),
	  eventTimes_(0)
{}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename scalar_t>
Feet3DPlanner<scalar_t>::Feet3DPlanner(const Feet3DPlanner& rhs)

	: BASE(rhs),
	  swingLegLiftOff_(rhs.swingLegLiftOff_),
	  swingTimeScale_(rhs.swingTimeScale_),
	  phaseIDsStock_(0),
	  eventTimes_(0)
{}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename scalar_t>
Feet3DPlanner<scalar_t>* Feet3DPlanner<scalar_t>::clone() const {

	return new Feet3DPlanner<scalar_t>(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename scalar_t>
void Feet3DPlanner<scalar_t>::setFootholdsPlan(
		const scalar_array_t& touchdownTimeStock,
		const std::vector<vector_3d_array_t>& touchdownPosStock,
		const std::vector<vector_3d_array_t>& touchdownVelStock) {

	touchdownTimeStock_  = touchdownTimeStock;
	touchdownPosStock_ = touchdownPosStock;
	touchdownVelStock_ = touchdownVelStock;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename scalar_t>
void Feet3DPlanner<scalar_t>::planSingleMode(
		const size_t& index,
		const size_array_t& phaseIDsStock,
		const scalar_array_t& eventTimes,
		feet_cpg_ptr_t& plannedCPG)  {

	// update Feet3DPlanner
	if (phaseIDsStock_!=phaseIDsStock) {
		phaseIDsStock_ = phaseIDsStock;

		BASE::extractContactFlags(phaseIDsStock_, eesContactFlagStocks_);

		for (size_t j=0; j<4; j++) {
			BASE::updateFootSchedule(j, phaseIDsStock_, eesContactFlagStocks_[j],
					BASE::startTimesIndices_[j], BASE::finalTimesIndices_[j]);
		}
	}

	const size_t numSubsystems = phaseIDsStock_.size();

	if (eventTimes.size()+1 != numSubsystems)
		throw std::runtime_error("The number of switching modes should be 1 plus number of event times.");

	// update eventTimes
	eventTimes_ = eventTimes;

	for (size_t j=0; j<4; j++)  {

		// create the CPG from the given times
		plannedCPG[j] = std::shared_ptr<foot_cpg_t>( new foot_cpg_t(swingLegLiftOff_, swingTimeScale_) );

		// for a swing leg
		if (eesContactFlagStocks_[j][index] == false)  {

			const int& swingStartIndex = BASE::startTimesIndices_[j][index];
			const int& swingFinalIndex = BASE::finalTimesIndices_[j][index];

			if (swingStartIndex==-1)
				throw std::runtime_error("The time of take-off for the first swing of the EE with ID "
						+ std::to_string(j) + " is not defined.");
			if (swingFinalIndex==numSubsystems-1)
				throw std::runtime_error("The time of touch-down for the last swing of the EE with ID "
						+ std::to_string(j) + " is not defined.");

			const scalar_t& swingStartTime = eventTimes_[swingStartIndex];
			const scalar_t& swingFinalTime = eventTimes_[swingFinalIndex];

			const scalar_t adaptedLiftOff = plannedCPG[j]->adaptiveSwingLegLiftOff(swingStartTime, swingFinalTime);

			plannedCPG[j]->set(
					touchdownTimeStock_[index],
					swingStartTime,
					swingFinalTime,
					touchdownPosStock_[index][j],
					touchdownVelStock_[index][j],
					touchdownPosStock_[swingFinalIndex][j],
					adaptedLiftOff);

		} else { // for a stance leg

			plannedCPG[j]->setConstant(touchdownPosStock_[index][j]);
		}

	}  // end of j loop
}


}  // end of switched_model namespace
