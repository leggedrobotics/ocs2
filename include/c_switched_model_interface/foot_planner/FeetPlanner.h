/*
 * FeetPlanner.h
 *
 *  Created on: Jul 10, 2017
 *      Author: farbod
 */

#ifndef FEETPLANNER_H_
#define FEETPLANNER_H_

#include <memory>
#include <array>
#include <vector>

#include "c_switched_model_interface/foot_planner/cpg/FootCPG.h"

namespace switched_model {

class FeetPlanner
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	typedef std::shared_ptr<FeetPlanner>  Ptr;
	typedef std::array<Eigen::Vector2d,4> vector_2d_array_t;

	FeetPlanner(const std::vector<std::array<bool,4> >& stanceLegsSequene,
			const double& swingLegLiftOff = 0.15,
			const double& swingTimeScale = 1.0,
			const std::array<bool,4>& defaultStartMode = std::array<bool,4>{1,1,1,1},
			const std::array<bool,4>& defaultFinalMode = std::array<bool,4>{1,1,1,1})
	: stanceLegs_(stanceLegsSequene),
	  swingLegLiftOff_(swingLegLiftOff),
	  swingTimeScale_(swingTimeScale),
	  defaultStartMode_(defaultStartMode),
	  defaultFinalMode_(defaultFinalMode)
	{}

	~FeetPlanner()  {}

	/**
	 *
	*/
	void planSingleMode(const std::vector<size_t>& systemStockIndexes, const std::vector<double>& switchingTimes,
			const size_t& activeSubsystemIndex,
			const double& startTime, const vector_2d_array_t& startPosXY, const vector_2d_array_t& startVelXY,
			const std::vector<vector_2d_array_t>& touchdownPosXYStock,
			FootCPG::PtrArray& plannedCPG)  {

		// update FeetPlanner if a new systemStockIndexes is set
		if (systemStockIndexes!=systemStockIndexes_) {
			systemStockIndexes_ = systemStockIndexes;
			updateFeetPlanner(systemStockIndexes_, stanceLegsStock_, startTimesIndices_, finalTimesIndices_);
		}

		size_t numSubsystems = systemStockIndexes_.size();

		if (switchingTimes.size() != numSubsystems+1)
			throw std::runtime_error("Number of switchingTimes_ is not compatible with the number of switchingModes.");
		if (activeSubsystemIndex>=numSubsystems)
			throw  std::runtime_error("activeSubsystem index should be smaller than the number of subsystems.");

		// update switchingTimes
		switchingTimes_ = switchingTimes;

		for (size_t j=0; j<4; j++)  {

			// create the CPG from the given times
			plannedCPG[j] = FootCPG::Ptr( new FootCPG(swingLegLiftOff_, swingTimeScale_) );

			// for a swing leg
			if (stanceLegsStock_[activeSubsystemIndex][j]==false)  {

				const size_t& startIndex = startTimesIndices_[activeSubsystemIndex][j];
				const size_t& finalIndex = finalTimesIndices_[activeSubsystemIndex][j];
				plannedCPG[j]->set(startTime,
						switchingTimes_[startIndex],
						switchingTimes_[finalIndex],
						startPosXY[j], startVelXY[j], touchdownPosXYStock[finalIndex][j],
						plannedCPG[j]->adaptiveSwingLegLiftOff(switchingTimes_[startIndex], switchingTimes_[finalIndex]) );

			} else { // for a stance leg

				plannedCPG[j]->set(startTime,
						switchingTimes_[activeSubsystemIndex],
						switchingTimes_[activeSubsystemIndex+1],
						startPosXY[j], Eigen::Vector2d::Zero() /*startVelXY*/, startPosXY[j] /*touchdownPosXY*/,
						0.0 /*Z direction lift*/);
			}

		}  // end of j loop
	}


protected:
	/**
	 * based on the input systemStockIndexes and the memeber variable stanceLegs_ findes the
	 * star susbsystem and final subsystem of the swing phases for each subsystem.
	 *
	 * @param systemStockIndexes: the shuffeling indeces for the stanceLegs_
	 * @param stanceLegsStock: the shuffeled stance leges.
	 * @param startTimesIndices:
	 * @param finalTimesIndices:
	 */
	void updateFeetPlanner(const std::vector<size_t>& systemStockIndexes,
			std::vector<std::array<bool,4>>& stanceLegsStock,
			std::vector<std::array<size_t,4>>& startTimesIndices,
			std::vector<std::array<size_t,4>>& finalTimesIndices) {

		size_t numSubsystems = systemStockIndexes.size();

		stanceLegsStock.resize(numSubsystems);
		for (size_t i=0; i<numSubsystems; i++)
			stanceLegsStock[i] = stanceLegs_[systemStockIndexes[i]];

		// find the startTime and finalTime indices
		startTimesIndices.resize(numSubsystems);
		finalTimesIndices.resize(numSubsystems);
		for (size_t i=0; i<numSubsystems; i++) {
			FindIndex (i, defaultStartMode_, defaultFinalMode_, stanceLegsStock,
					startTimesIndices[i], finalTimesIndices[i]);
		}
	}

	/**
	 *
	 * @param activeSubsystemIndex
	 * @param defaultStartMode
	 * @param defaultFinalMode
	 * @param stanceLegsStock
	 *
	 * @param startTimesIndex: switchingTimes[startTimesIndex] will be the takoff time for swing legs
	 * @param finalTimesIndex: switchingTimes[finalTimesIndex] will be the touch-down time for swing legs
	 */
	static void FindIndex (size_t activeSubsystemIndex,
			const std::array<bool,4>& defaultStartMode,
			const std::array<bool,4>& defaultFinalMode,
			const std::vector<std::array<bool,4>>& stanceLegsStock,
			std::array<size_t,4>& startTimesIndex,
			std::array<size_t,4>& finalTimesIndex)  {

		const size_t numSubsystems = stanceLegsStock.size();

		if (activeSubsystemIndex>=numSubsystems)
			throw  std::runtime_error("activeSubsystem index should be smaller than the number of subsystems.");

		// augment the stock with the defaultStartMode and defaultFinalMode
		std::vector<std::array<bool,4>> stanceLegsStockTemp(numSubsystems+2);
		stanceLegsStockTemp[0] = defaultStartMode;
		for (size_t i=0; i<numSubsystems; i++)
			stanceLegsStockTemp[i+1] = stanceLegsStock[i];
		stanceLegsStockTemp[numSubsystems+1] = defaultFinalMode;

		// since one system in appended to the beginning
		activeSubsystemIndex++;

		for (size_t j=0; j<4; j++)  {

			// skip if it is a stance leg
			if (stanceLegsStockTemp[activeSubsystemIndex][j]==true)
				continue;

			// find the starting time
			startTimesIndex[j] = 0;
			for (int ip=activeSubsystemIndex-1; ip>=0; ip--)
				if (stanceLegsStockTemp[ip][j]==true) {
					startTimesIndex[j] = ip;
					break;
				}

			// find the final time
			finalTimesIndex[j] = numSubsystems;
			for (size_t ip=activeSubsystemIndex+1; ip<numSubsystems+2; ip++)
				if (stanceLegsStockTemp[ip][j]==true) {
					finalTimesIndex[j] = ip-1;
					break;
				}

		}  // end of j loop

	}


private:
	std::vector<std::array<bool,4> > stanceLegs_;
	double swingLegLiftOff_;
	double swingTimeScale_;
	std::array<bool,4> defaultStartMode_;
	std::array<bool,4> defaultFinalMode_;

	std::vector<double> switchingTimes_;
	std::vector<size_t> systemStockIndexes_;
	std::vector<std::array<bool,4> > stanceLegsStock_;

	std::vector<std::array<size_t,4> > startTimesIndices_;
	std::vector<std::array<size_t,4> > finalTimesIndices_;
};

}  // end of switched_model namespace


#endif /* FEETPLANNER_H_ */
