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
template <class CPG_T, typename scalar_t>
FeetZDirectionPlanner<CPG_T, scalar_t>::FeetZDirectionPlanner(
		const scalar_t& swingLegLiftOff /*= 0.15*/,
		const scalar_t& swingTimeScale /*= *1.0*/)

	: swingLegLiftOff_(swingLegLiftOff),
	  swingTimeScale_(swingTimeScale)
{}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class CPG_T, typename scalar_t>
FeetZDirectionPlanner<CPG_T, scalar_t>::FeetZDirectionPlanner(const FeetZDirectionPlanner& rhs)

	: swingLegLiftOff_(rhs.swingLegLiftOff_),
	  swingTimeScale_(rhs.swingTimeScale_),
	  phaseIDsStock_(0),
	  switchingTimes_(0)
{}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class CPG_T, typename scalar_t>
FeetZDirectionPlanner<CPG_T, scalar_t>* FeetZDirectionPlanner<CPG_T, scalar_t>::clone() const {

	return new FeetZDirectionPlanner<CPG_T, scalar_t>(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class CPG_T, typename scalar_t>
void FeetZDirectionPlanner<CPG_T, scalar_t>::planSingleMode(
		const size_t& index,
		const size_array_t& phaseIDsStock,
		const scalar_array_t& switchingTimes,
		feet_cpg_ptr_t& plannedCPG)  {

	// update FeetZDirectionPlanner if a new phaseIDsStock is set
	if (phaseIDsStock_!=phaseIDsStock) {
		phaseIDsStock_ = phaseIDsStock;
		updateFeetZDirectionPlanner(phaseIDsStock_,
				eesContactFlagStocks_, BASE::startTimesIndices_, BASE::finalTimesIndices_);
	}

	size_t numSubsystems = phaseIDsStock_.size();

	if (switchingTimes.size() != numSubsystems-1)
		throw std::runtime_error("The number of switching modes should be 1 plus number of switching times.");

	// update switchingTimes
	switchingTimes_ = switchingTimes;

	for (size_t j=0; j<4; j++)  {

		// skip if it is a stance leg
		if (eesContactFlagStocks_[j][index]==true)  continue;

		const size_t& startIndex = BASE::startTimesIndices_[j][index];
		const size_t& finalIndex = BASE::finalTimesIndices_[j][index];

		const scalar_t& startTime = switchingTimes_[startIndex];
		const scalar_t& finalTime = switchingTimes_[finalIndex];

		// create the CPG from the given times
		plannedCPG[j] = std::shared_ptr<CPG_T>( new CPG_T(swingLegLiftOff_, swingTimeScale_) );
		scalar_t adaptedLiftOff = plannedCPG[j]->adaptiveSwingLegLiftOff(startTime, finalTime);
		plannedCPG[j]->set(startTime, finalTime, adaptedLiftOff);

	}  // end of j loop
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class CPG_T, typename scalar_t>
void FeetZDirectionPlanner<CPG_T, scalar_t>::updateFeetZDirectionPlanner(
		const size_array_t& phaseIDsStock,
		std::array<bool_array_t,4>& eeContactFlagStocks,
		std::array<size_array_t,4>& startTimesIndices,
		std::array<size_array_t,4>& finalTimesIndices) {

	size_t numSubsystems = phaseIDsStock.size();

	for (size_t j=0; j<4; j++) {
		eeContactFlagStocks[j].resize(numSubsystems);
		startTimesIndices[j].resize(numSubsystems);
		finalTimesIndices[j].resize(numSubsystems);
	}

	for (size_t i=0; i<numSubsystems; i++) {
		std::array<bool,4> contactFlags = modeNumber2StanceLeg(phaseIDsStock[i]);
		for (size_t j=0; j<4; j++)
			eeContactFlagStocks[j][i] = contactFlags[j];
	}

	// find the startTime and finalTime indices
	for (size_t j=0; j<4; j++)
		for (size_t i=0; i<numSubsystems; i++) {

			int startTimeIndex, finalTimeIndex;
			FindIndex(i, eeContactFlagStocks[j], startTimeIndex, finalTimeIndex);

			if (startTimeIndex==-1)
				throw std::runtime_error("The time of take-off for the first swing of the EE with ID "
						+ std::to_string(j) + " is not defined.");
			else
				startTimesIndices[j][i] = startTimeIndex;

			if (finalTimeIndex==numSubsystems-1)
				throw std::runtime_error("The time of touch-down for the last swing of the EE with ID "
										+ std::to_string(j) + " is not defined.");
			else
				finalTimesIndices[j][i] = finalTimeIndex;
		}
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
//template <class CPG_T, typename scalar_t>
//static void FeetZDirectionPlanner<CPG_T, scalar_t>::FindIndex (
//		const size_t& activeSubsystemIndex,
//		const std::array<bool,4>& defaultStartMode,
//		const std::array<bool,4>& defaultFinalMode,
//		const std::vector<std::array<bool,4>>& stanceLegsStock,
//		std::array<size_t,4>& startTimesIndex,
//		std::array<size_t,4>& finalTimesIndex)  {
//
//	const size_t numSubsystems = stanceLegsStock.size();
//
//	if (activeSubsystemIndex>=numSubsystems)
//		throw  std::runtime_error("activeSubsystem index should be smaller than the number of subsystems.");
//
//	// augment the stock with the defaultStartMode and defaultFinalMode
//	std::vector<std::array<bool,4>> stanceLegsStockTemp(numSubsystems+2);
//	stanceLegsStockTemp[0] = defaultStartMode;
//	for (size_t i=0; i<numSubsystems; i++)
//		stanceLegsStockTemp[i+1] = stanceLegsStock[i];
//	stanceLegsStockTemp[numSubsystems+1] = defaultFinalMode;
//
//	// since one system in appended to the beginning
//	activeSubsystemIndex++;
//
//	for (size_t j=0; j<4; j++)  {
//
//		// skip if it is a stance leg
//		if (stanceLegsStockTemp[activeSubsystemIndex][j]==true)
//			continue;
//
//		// find the starting time
//		startTimesIndex[j] = 0;
//		for (int ip=activeSubsystemIndex-1; ip>=0; ip--)
//			if (stanceLegsStockTemp[ip][j]==true) {
//				startTimesIndex[j] = ip;
//				break;
//			}
//
//		// find the final time
//		finalTimesIndex[j] = numSubsystems;
//		for (size_t ip=activeSubsystemIndex+1; ip<numSubsystems+2; ip++)
//			if (stanceLegsStockTemp[ip][j]==true) {
//				finalTimesIndex[j] = ip-1;
//				break;
//			}
//	}  // end of j loop
//}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class CPG_T, typename scalar_t>
void FeetZDirectionPlanner<CPG_T, scalar_t>::FindIndex (
		const size_t& index,
		const bool_array_t& contactFlagStock,
		int& startTimesIndex,
		int& finalTimesIndex)  {

	const size_t numSubsystems = contactFlagStock.size();

	// skip if it is a stance leg
	if (contactFlagStock[index]==true)  return;

	// find the starting time
	startTimesIndex = -1;
	for (int ip=index-1; ip>=0; ip--)
		if (contactFlagStock[ip]==true) {
			startTimesIndex = ip;
			break;
		}

	// find the final time
	finalTimesIndex = numSubsystems-1;
	for (size_t ip=index+1; ip<numSubsystems; ip++)
		if (contactFlagStock[ip]==true) {
			finalTimesIndex = ip-1;
			break;
		}
}

}  // end of switched_model namespace




