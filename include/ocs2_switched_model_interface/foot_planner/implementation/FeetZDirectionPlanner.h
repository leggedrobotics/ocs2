/*
 * FeetZDirectionPlanner.h
 *
 *  Created on: Feb 5, 2018
 *      Author: farbod
 */

namespace switched_model {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename scalar_t, class cpg_t>
FeetZDirectionPlanner<scalar_t, cpg_t>::FeetZDirectionPlanner(
		const scalar_t& swingLegLiftOff,
		const scalar_t& swingTimeScale /*= *1.0*/,
		const scalar_t& liftOffVelocity /*= *0.0*/,
		const scalar_t& touchDownVelocity /*= *0.0*/)

	: swingLegLiftOff_(swingLegLiftOff),
	  swingTimeScale_(swingTimeScale),
		liftOffVelocity_(liftOffVelocity),
		touchDownVelocity_(touchDownVelocity),
	  phaseIDsStock_(0),
	  eventTimes_(0)
{}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename scalar_t, class cpg_t>
FeetZDirectionPlanner<scalar_t, cpg_t>::FeetZDirectionPlanner(const FeetZDirectionPlanner& rhs)

	: BASE(rhs),
	  swingLegLiftOff_(rhs.swingLegLiftOff_),
	  swingTimeScale_(rhs.swingTimeScale_),
		liftOffVelocity_(rhs.liftOffVelocity_),
		touchDownVelocity_(rhs.touchDownVelocity_),
	  phaseIDsStock_(0),
	  eventTimes_(0)
{}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename scalar_t, class cpg_t>
FeetZDirectionPlanner<scalar_t, cpg_t>* FeetZDirectionPlanner<scalar_t, cpg_t>::clone() const {

	return new FeetZDirectionPlanner<scalar_t, cpg_t>(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename scalar_t, class cpg_t>
void FeetZDirectionPlanner<scalar_t, cpg_t>::planSingleMode(
		const size_t& index,
		const size_array_t& phaseIDsStock,
		const scalar_array_t& eventTimes,
		feet_cpg_ptr_t& plannedCPG)  {

	// update FeetZDirectionPlanner if a new phaseIDsStock is set
	if (phaseIDsStock_ != phaseIDsStock) {
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
		plannedCPG[j] = std::shared_ptr<cpg_t>( new cpg_t(swingLegLiftOff_,
																											swingTimeScale_,
																											liftOffVelocity_,
																											touchDownVelocity_) );

		// skip if it is a stance leg
		if (eesContactFlagStocks_[j][index]==false)  {

			const int& swingStartIndex = BASE::startTimesIndices_[j][index];
			const int& swingFinalIndex = BASE::finalTimesIndices_[j][index];

			if (swingStartIndex == -1) {
				std::cout << "Subsystem: " << index << " out of " << numSubsystems-1 << std::endl;
				for (size_t i=0; i<phaseIDsStock_.size(); i++)
					std::cout << "[" << i << "]: " << phaseIDsStock_[i] << ",  ";
				std::cout << std::endl;

				throw std::runtime_error("The time of take-off for the first swing of the EE with ID "
						+ std::to_string(j) + " is not defined.");
			}
			if (swingFinalIndex == numSubsystems-1) {
				std::cout << "Subsystem: " << index << " out of " << numSubsystems-1 << std::endl;
				for (size_t i=0; i<phaseIDsStock_.size(); i++)
					std::cout << "[" << i << "]: " << phaseIDsStock_[i] << ",  ";
				std::cout << std::endl;

				throw std::runtime_error("The time of touch-down for the last swing of the EE with ID "
						+ std::to_string(j) + " is not defined.");
			}

			const scalar_t& swingStartTime = eventTimes_[swingStartIndex];
			const scalar_t& swingFinalTime = eventTimes_[swingFinalIndex];

			const scalar_t adaptedLiftOff = plannedCPG[j]->adaptiveSwingLegLiftOff(swingStartTime, swingFinalTime);

			plannedCPG[j]->set(
					swingStartTime,
					swingFinalTime,
					adaptedLiftOff);

		} else { // for a stance leg

			plannedCPG[j]->setConstant();
		}

	}  // end of j loop
}


}  // end of switched_model namespace

